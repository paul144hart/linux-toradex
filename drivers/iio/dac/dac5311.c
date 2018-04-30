/*
 * TI DAC8554 Digital to Analog Converter SPI driver
 *
 * Copyright (C) 2014 Avionic Design GmbH
 *
 * Based on ad5446r_spi.c
 * Copyright (C) 2010,2011 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/of.h>
#define DAC8554_DRIVERNAME			"ti-dac8554"
#define DAC8554_DAC_CHANNELS			4
#define DAC8554_DATABITS			16

/* Load commands */
#define DAC8554_CMD_STORE_DAC_N			0x0
#define DAC8554_CMD_UPDATE_DAC_N		0x1
#define DAC8554_CMD_STORE_DAC_N_UPDATE_ALL	0x2
#define DAC8554_CMD_UPDATE_BROADCAST		0x3

#define DAC8554_BROADCAST_USE_SRDATA		0x2

/* Powerdown modes (PD1 | PD2 bits) */
#define DAC8554_PWRDN_HIZ			0x0
#define DAC8554_PWRDN_1K			0x1
#define DAC8554_PWRDN_100K			0x2

/* Input shift register composition */
#define DAC8554_ADDR_TO_SR(addr)		((addr) << 22)
#define DAC8554_CMD_TO_SR(cmd)			((cmd) << 20)
#define DAC8554_CHAN_TO_SR(chan)		((chan) << 17)
#define DAC8554_PWRDN_TO_SR(mode)		(BIT(16) | (mode) << 14)

/**
 * struct dac8554_state - driver instance specific data
 * @spi:		SPI device
 * @reg:		supply regulator
 * @addr:		two-bit chip address
 * @val:		channel data
 * @powerdown:		channel powerdown flag
 * @powerdown_mode:	channel powerdown mode
 * @xfer:		SPI transfer buffer
 */
struct dac8554_state {
	struct spi_device		*spi;
	struct regulator		*reg;
	unsigned			addr;
	u16				val[DAC8554_DAC_CHANNELS];
	bool				powerdown[DAC8554_DAC_CHANNELS];
	u8				powerdown_mode[DAC8554_DAC_CHANNELS];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8				xfer[3] ____cacheline_aligned;
};

static int dac8554_spi_write(struct dac8554_state *st,
			     unsigned cmd,
			     unsigned chan_addr,
			     unsigned val)
{
	u32 data;

	/*
	 * The input shift register is 24 bits wide. The 8 MSB are
	 * control bits, followed by 16 data bits.
	 * The first two bits A1 and A0 address a DAC8554 chip.
	 * The next two are the command bits, LD1 and LD0.
	 * After a don't-care-bit, the next two bits select the channel.
	 * The final control bit PD0 is a flag signalling if the data
	 * bits encode a powerdown mode. We merge PD0 with the adjacent
	 * data bits.
	 */

	if (cmd > 3 || chan_addr > 3 ||
			(val > 0xffff && (val & ~DAC8554_PWRDN_TO_SR(3))))
		return -EINVAL;

data = DAC8554_ADDR_TO_SR(st->addr) | DAC8554_CMD_TO_SR(cmd) |
	       DAC8554_CHAN_TO_SR(chan_addr) | val;

	st->xfer[0] = data >> 16;
	st->xfer[1] = data >> 8;
st->xfer[2] = data;

	return spi_write(st->spi, st->xfer, sizeof(st->xfer));
}

static int dac8554_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long info)
{
	struct dac8554_state *st = iio_priv(indio_dev);
	int voltage_uv;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		*val = st->val[chan->address];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		voltage_uv = regulator_get_voltage(st->reg);
		if (voltage_uv < 0)
			return voltage_uv;
		*val = voltage_uv / 1000;
		*val2 = DAC8554_DATABITS;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static int dac8554_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long info)
{
	struct dac8554_state *st = iio_priv(indio_dev);
	int err;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (val > 0xffff || val < 0)
			return -EINVAL;

		err = dac8554_spi_write(st, DAC8554_CMD_UPDATE_DAC_N,
					chan->address, val);
		if (err)
			return err;

		st->val[chan->address] = val;

		/* By hw design, DAC updates automatically trigger powerup. */
		st->powerdown[chan->address] = false;

		return 0;

	default:
		return -EINVAL;
	}
}

static int dac8554_get_powerdown_mode(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct dac8554_state *st = iio_priv(indio_dev);

	return st->powerdown_mode[chan->address];
}

static int dac8554_set_powerdown_mode(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int mode)
{
	struct dac8554_state *st = iio_priv(indio_dev);

	st->powerdown_mode[chan->address] = mode;

	return 0;
}

static ssize_t dac8554_read_dac_powerdown(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  char *buf)
{
	struct dac8554_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->powerdown[chan->address]);
}

static ssize_t dac8554_write_dac_powerdown(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   const char *buf,
					   size_t len)
{
	bool powerdown;
	int ret;
	struct dac8554_state *st = iio_priv(indio_dev);
	u8 powerdown_mode;

	ret = strtobool(buf, &powerdown);
	if (ret)
		return ret;

	st->powerdown[chan->address] = powerdown;

	if (powerdown) {
		powerdown_mode = st->powerdown_mode[chan->address];
		ret = dac8554_spi_write(st,
					DAC8554_CMD_UPDATE_DAC_N,
					chan->address,
					DAC8554_PWRDN_TO_SR(powerdown_mode));
	} else {
		/* Load DAC with cached value. This triggers a powerup. */
		ret = dac8554_spi_write(st,
					DAC8554_CMD_UPDATE_DAC_N,
					chan->address,
					st->val[chan->address]);
	}

	if (ret)
		return ret;

	return len;
}

static int dac8554_powerdown(struct dac8554_state *st,
			     u8 powerdown_mode)
{
	int chan, cmd, ret;

	for (chan = DAC8554_DAC_CHANNELS - 1; chan >= 0; --chan) {
		cmd = chan ? DAC8554_CMD_STORE_DAC_N
			   : DAC8554_CMD_STORE_DAC_N_UPDATE_ALL;
		ret = dac8554_spi_write(st, cmd, chan,
					DAC8554_PWRDN_TO_SR(powerdown_mode));
		if (ret)
			return ret;
	}

	for (chan = 0; chan < DAC8554_DAC_CHANNELS; ++chan) {
		st->powerdown_mode[chan] = powerdown_mode;
		st->powerdown[chan] = true;
	}

	return 0;
}

static const struct iio_info dac8554_info = {
	.write_raw = dac8554_write_raw,
	.read_raw = dac8554_read_raw,
	.driver_module = THIS_MODULE,
};

static const char * const dac8554_powerdown_modes[] = {
	"three_state",
	"1kohm_to_gnd",
	"100kohm_to_gnd"
};

static const struct iio_enum dac8854_powerdown_mode_enum = {
	.items = dac8554_powerdown_modes,
	.num_items = ARRAY_SIZE(dac8554_powerdown_modes),
	.get = dac8554_get_powerdown_mode,
	.set = dac8554_set_powerdown_mode,
};

static const struct iio_chan_spec_ext_info dac8554_ext_info[] = {
	{
		.name = "powerdown",
		.read = dac8554_read_dac_powerdown,
		.write = dac8554_write_dac_powerdown,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE,
		 &dac8854_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", &dac8854_powerdown_mode_enum),
	{ },
};

#define DAC8554_CHANNEL(chan) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.output = 1, \
	.channel = (chan), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.address = (chan), \
	.ext_info = dac8554_ext_info, \
}
static const struct iio_chan_spec dac8554_channels[] = {
	DAC8554_CHANNEL(0),
	DAC8554_CHANNEL(1),
	DAC8554_CHANNEL(2),
	DAC8554_CHANNEL(3),
};
#undef DAC8554_CHANNEL

static int dac8554_probe(struct spi_device *spi)
{
	struct dac8554_state *st;
	struct iio_dev *indio_dev;
	int ret;
	u32 addr;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = DAC8554_DRIVERNAME;
	indio_dev->info = &dac8554_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = dac8554_channels;
	indio_dev->num_channels = ARRAY_SIZE(dac8554_channels);

	spi_set_drvdata(spi, indio_dev);

	st = iio_priv(indio_dev);

	if (!spi->dev.of_node) {
		dev_err(&spi->dev, "missing OF node");
		return -EINVAL;
	}
	ret = of_property_read_u32(spi->dev.of_node, "ti,address", &addr);
	if (ret || addr < 0 || addr > 2) {
		dev_err(&spi->dev, "no or invalid chip address");
		return -EINVAL;
	}

	st->spi = spi;
	st->addr = addr;

	st->reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->reg))
		return PTR_ERR(st->reg);

	ret = regulator_enable(st->reg);
	if (ret)
		return ret;

	ret = dac8554_powerdown(st, DAC8554_PWRDN_100K);
	if (ret)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	regulator_disable(st->reg);
	return ret;
}

static int dac8554_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct dac8554_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	regulator_disable(st->reg);

	return 0;
}

static const struct of_device_id dac8554_of_match[] = {
	{ .compatible = "ti,dac8554" },
	{ }
};
MODULE_DEVICE_TABLE(of, dac8554_of_match);

static struct spi_driver dac8554_driver = {
	.driver = {
		.name = DAC8554_DRIVERNAME,
		.owner = THIS_MODULE,
		.of_match_table = dac8554_of_match,
	 },
	.probe = dac8554_probe,
	.remove = dac8554_remove,
};
module_spi_driver(dac8554_driver);

MODULE_AUTHOR("Nikolaus Schulz <nikolaus.schulz@avionic-design.de>");
MODULE_DESCRIPTION("Texas Instruments DAC8554 SPI driver");
MODULE_LICENSE("GPL v2");
