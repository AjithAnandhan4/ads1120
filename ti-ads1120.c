// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI ADS1120 4-channel, 2kSPS, 16-bit delta-sigma ADC
 *
 * Datasheet: https://www.ti.com/lit/gpn/ads1120
 *
 * Copyright (C) 2025 Ajith Anandhan <ajithanandhan0406@gmail.com>
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

#include <asm/unaligned.h>

/* Commands */
#define ADS1120_CMD_RESET		0x06
#define ADS1120_CMD_START		0x08
#define ADS1120_CMD_PWRDWN		0x02
#define ADS1120_CMD_RDATA		0x10
#define ADS1120_CMD_RREG		0x20
#define ADS1120_CMD_WREG		0x40

/* Registers */
#define ADS1120_REG_CONFIG0		0x00
#define ADS1120_REG_CONFIG1		0x01
#define ADS1120_REG_CONFIG2		0x02
#define ADS1120_REG_CONFIG3		0x03

/* Config Register 0 bit definitions */
#define ADS1120_CFG0_MUX_MASK		GENMASK(7, 4)
#define ADS1120_CFG0_MUX_AIN0_AVSS	8
#define ADS1120_CFG0_MUX_AIN1_AVSS	9
#define ADS1120_CFG0_MUX_AIN2_AVSS	10
#define ADS1120_CFG0_MUX_AIN3_AVSS	11

#define ADS1120_CFG0_GAIN_MASK		GENMASK(3, 1)
#define ADS1120_CFG0_GAIN_1		0
#define ADS1120_CFG0_GAIN_2		1
#define ADS1120_CFG0_GAIN_4		2
#define ADS1120_CFG0_GAIN_8		3
#define ADS1120_CFG0_GAIN_16		4
#define ADS1120_CFG0_GAIN_32		5
#define ADS1120_CFG0_GAIN_64		6
#define ADS1120_CFG0_GAIN_128		7

#define ADS1120_CFG0_PGA_BYPASS		BIT(0)

/* Config Register 1 bit definitions */
#define ADS1120_CFG1_DR_MASK		GENMASK(7, 5)
#define ADS1120_CFG1_DR_20SPS		0
#define ADS1120_CFG1_DR_45SPS		1
#define ADS1120_CFG1_DR_90SPS		2
#define ADS1120_CFG1_DR_175SPS		3
#define ADS1120_CFG1_DR_330SPS		4
#define ADS1120_CFG1_DR_600SPS		5
#define ADS1120_CFG1_DR_1000SPS		6

#define ADS1120_CFG1_MODE_MASK		GENMASK(4, 3)
#define ADS1120_CFG1_MODE_NORMAL	0
#define ADS1120_CFG1_MODE_DUTY		1
#define ADS1120_CFG1_MODE_TURBO		2

#define ADS1120_CFG1_CM_MASK		BIT(2)
#define ADS1120_CFG1_CM_SINGLE		0
#define ADS1120_CFG1_CM_CONTINUOUS	1

#define ADS1120_CFG1_TS_EN		BIT(1)
#define ADS1120_CFG1_BCS_EN		BIT(0)

/* Config Register 2 bit definitions */
#define ADS1120_CFG2_VREF_MASK		GENMASK(7, 6)
#define ADS1120_CFG2_VREF_INTERNAL	0
#define ADS1120_CFG2_VREF_EXT_REFP0	1
#define ADS1120_CFG2_VREF_EXT_AIN0	2
#define ADS1120_CFG2_VREF_AVDD		3

#define ADS1120_CFG2_REJECT_MASK	GENMASK(5, 4)
#define ADS1120_CFG2_REJECT_OFF		0
#define ADS1120_CFG2_REJECT_50_60	1
#define ADS1120_CFG2_REJECT_50		2
#define ADS1120_CFG2_REJECT_60		3

#define ADS1120_CFG2_PSW_EN		BIT(3)

#define ADS1120_CFG2_IDAC_MASK		GENMASK(2, 0)
#define ADS1120_CFG2_IDAC_OFF		0
#define ADS1120_CFG2_IDAC_10UA		1
#define ADS1120_CFG2_IDAC_50UA		2
#define ADS1120_CFG2_IDAC_100UA		3
#define ADS1120_CFG2_IDAC_250UA		4
#define ADS1120_CFG2_IDAC_500UA		5
#define ADS1120_CFG2_IDAC_1000UA	6
#define ADS1120_CFG2_IDAC_1500UA	7

/* Config Register 3 bit definitions */
#define ADS1120_CFG3_IDAC1_MASK		GENMASK(7, 5)
#define ADS1120_CFG3_IDAC1_DISABLED	0
#define ADS1120_CFG3_IDAC1_AIN0		1
#define ADS1120_CFG3_IDAC1_AIN1		2
#define ADS1120_CFG3_IDAC1_AIN2		3
#define ADS1120_CFG3_IDAC1_AIN3		4
#define ADS1120_CFG3_IDAC1_REFP0	5
#define ADS1120_CFG3_IDAC1_REFN0	6

#define ADS1120_CFG3_IDAC2_MASK		GENMASK(4, 2)
#define ADS1120_CFG3_IDAC2_DISABLED	0
#define ADS1120_CFG3_IDAC2_AIN0		1
#define ADS1120_CFG3_IDAC2_AIN1		2
#define ADS1120_CFG3_IDAC2_AIN2		3
#define ADS1120_CFG3_IDAC2_AIN3		4
#define ADS1120_CFG3_IDAC2_REFP0	5
#define ADS1120_CFG3_IDAC2_REFN0	6

#define ADS1120_CFG3_DRDYM_MASK		BIT(1)
#define ADS1120_CFG3_DRDYM_DRDY_ONLY	0
#define ADS1120_CFG3_DRDYM_BOTH		1

struct ads1120_state {
	struct spi_device	*spi;
	/*
	 * Protects chip configuration and ADC reads to ensure
	 * consistent channel/gain settings during conversions.
	 */
	struct mutex		lock;

	u8 config[4];

	/* DMA-safe buffer for SPI transfers. */
	u8 data[4] __aligned(IIO_DMA_MINALIGN);
};

struct ads1120_datarate {
	int rate;
	int conv_time_ms;
	u8 reg_value;
};

static const struct ads1120_datarate ads1120_datarates[] = {
	{ 20,   51, ADS1120_CFG1_DR_20SPS },
	{ 45,   24, ADS1120_CFG1_DR_45SPS },
	{ 90,   13, ADS1120_CFG1_DR_90SPS },
	{ 175,   7, ADS1120_CFG1_DR_175SPS },
	{ 330,   4, ADS1120_CFG1_DR_330SPS },
	{ 600,   3, ADS1120_CFG1_DR_600SPS },
	{ 1000,  2, ADS1120_CFG1_DR_1000SPS },
};

static const int ads1120_gain_values[] = { 1, 2, 4, 8, 16, 32, 64, 128 };
static const int ads1120_datarates_list[] = { 20, 45, 90, 175, 330, 600, 1000 };

#define ADS1120_CHANNEL(index)					\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_separate_available = BIT(IIO_CHAN_INFO_SCALE), \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
				   BIT(IIO_CHAN_INFO_SAMP_FREQ_AVAIL), \
}

static const struct iio_chan_spec ads1120_channels[] = {
	ADS1120_CHANNEL(0),
	ADS1120_CHANNEL(1),
	ADS1120_CHANNEL(2),
	ADS1120_CHANNEL(3),
};

static int ads1120_get_conv_time_ms(struct ads1120_state *st)
{
	u8 dr_bits = FIELD_GET(ADS1120_CFG1_DR_MASK, st->config[1]);
	int i;

	for (i = 0; i < ARRAY_SIZE(ads1120_datarates); i++) {
		if (ads1120_datarates[i].reg_value == dr_bits)
			return ads1120_datarates[i].conv_time_ms;
	}

	/* Should never happen with valid config */
	return 7;  /* Default to 175 SPS timing */
}

static int ads1120_write_cmd(struct ads1120_state *st, u8 cmd)
{
	st->data[0] = cmd;
	return spi_write(st->spi, st->data, 1);
}

static int ads1120_write_reg(struct ads1120_state *st, u8 reg, u8 value)
{
	if (reg > ADS1120_REG_CONFIG3)
		return -EINVAL;

	st->data[0] = ADS1120_CMD_WREG | (reg << 2);
	st->data[1] = value;

	return spi_write(st->spi, st->data, 2);
}

static int ads1120_read_reg(struct ads1120_state *st, u8 reg, u8 *value)
{
	int ret;

	if (reg > ADS1120_REG_CONFIG3)
		return -EINVAL;

	st->data[0] = ADS1120_CMD_RREG | (reg << 2);

	ret = spi_write_then_read(st->spi, st->data, 1, st->data, 1);
	if (ret)
		return ret;

	*value = st->data[0];
	return 0;
}

static int ads1120_reset(struct ads1120_state *st)
{
	int ret;

	ret = ads1120_write_cmd(st, ADS1120_CMD_RESET);
	if (ret)
		return ret;

	/*
	 * Datasheet specifies reset takes 50us + 32 * t(CLK)
	 * where t(CLK) = 1/4.096MHz (~0.24us).
	 * 50us + 32 * 0.24us = ~58us. Use 200us to be safe.
	 */
	fsleep(200);

	return 0;
}

static int ads1120_set_channel(struct ads1120_state *st, int channel)
{
	u8 mux_val, config0;

	if (channel < 0 || channel > 3)
		return -EINVAL;

	/* Map channel to AINx/AVSS single-ended input */
	mux_val = ADS1120_CFG0_MUX_AIN0_AVSS + channel;

	config0 = FIELD_MODIFY(st->config[0], ADS1120_CFG0_MUX_MASK, mux_val);
	st->config[0] = config0;
	
	return ads1120_write_reg(st, ADS1120_REG_CONFIG0, config0);
}

static int ads1120_set_gain(struct ads1120_state *st, int gain_val)
{
	u8 config0;
	int i;

	/* Find gain in supported values */
	for (i = 0; i < ARRAY_SIZE(ads1120_gain_values); i++) {
		if (ads1120_gain_values[i] == gain_val)
			break;
	}

	if (i == ARRAY_SIZE(ads1120_gain_values))
		return -EINVAL;

	config0 = FIELD_MODIFY(st->config[0], ADS1120_CFG0_GAIN_MASK, i);
	st->config[0] = config0;
	
	return ads1120_write_reg(st, ADS1120_REG_CONFIG0, config0);
}

static int ads1120_set_datarate(struct ads1120_state *st, int rate)
{
	u8 config1;
	int i;

	/* Find data rate in supported values */
	for (i = 0; i < ARRAY_SIZE(ads1120_datarates); i++) {
		if (ads1120_datarates[i].rate != rate)
			continue;
		
		config1 = FIELD_MODIFY(st->config[1], ADS1120_CFG1_DR_MASK,
				       ads1120_datarates[i].reg_value);
		st->config[1] = config1;

		return ads1120_write_reg(st, ADS1120_REG_CONFIG1, config1);
	}

	return -EINVAL;
}

static int ads1120_read_raw_adc(struct ads1120_state *st, int *val)
{
	int ret;
	struct spi_transfer xfer[2] = {
		{
			.tx_buf = st->data,
			.len = 1,
		}, {
			.rx_buf = st->data,
			.len = 2,
		}
	};

	st->data[0] = ADS1120_CMD_RDATA;

	ret = spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
		return ret;

	/*
	 * Data format: 16-bit two's complement, MSB first
	 * st->data[0] = MSB
	 * st->data[1] = LSB
	 */
	*val = sign_extend32(get_unaligned_be16(st->data), 15);

	return 0;
}

static int ads1120_read_measurement(struct ads1120_state *st, int channel,
				    int *val)
{
	int ret;

	ret = ads1120_set_channel(st, channel);
	if (ret)
		return ret;

	/* Start single-shot conversion */
	ret = ads1120_write_cmd(st, ADS1120_CMD_START);
	if (ret)
		return ret;

	/* Wait for conversion to complete. */
	msleep(ads1120_get_conv_time_ms(st));

	ret = ads1120_read_raw_adc(st, val);
	if (ret)
		return ret;

	return 0;
}

static int ads1120_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ads1120_state *st = iio_priv(indio_dev);
	int ret;
	int gain_index;
	u8 dr_bits;
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		guard(mutex)(&st->lock);
		ret = ads1120_read_measurement(st, chan->channel, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* Look up gain value from config register */
		gain_index = FIELD_GET(ADS1120_CFG0_GAIN_MASK, st->config[0]);
		*val = ads1120_gain_values[gain_index];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		/* Look up data rate from config register */
		dr_bits = FIELD_GET(ADS1120_CFG1_DR_MASK, st->config[1]);
		for (i = 0; i < ARRAY_SIZE(ads1120_datarates); i++) {
			if (ads1120_datarates[i].reg_value == dr_bits) {
				*val = ads1120_datarates[i].rate;
				return IIO_VAL_INT;
			}
		}
		return -EINVAL;

	default:
		return -EINVAL;
	}
}

static int ads1120_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ads1120_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		guard(mutex)(&st->lock);
		return ads1120_set_gain(st, val);

	case IIO_CHAN_INFO_SAMP_FREQ:
		guard(mutex)(&st->lock);
		return ads1120_set_datarate(st, val);

	default:
		return -EINVAL;
	}
}

static int ads1120_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*vals = ads1120_gain_values;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(ads1120_gain_values);
		return IIO_AVAIL_LIST;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = ads1120_datarates_list;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(ads1120_datarates_list);
		return IIO_AVAIL_LIST;

	default:
		return -EINVAL;
	}
}

static const struct iio_info ads1120_info = {
	.read_raw = ads1120_read_raw,
	.write_raw = ads1120_write_raw,
	.read_avail = ads1120_read_avail,
};

static int ads1120_init(struct ads1120_state *st)
{
	int ret, config2, config3;

	ret = ads1120_reset(st);
	if (ret)
		return dev_err_probe(&st->spi->dev, ret,
				     "Failed to reset device\n");

	st->config[0] = FIELD_PREP(ADS1120_CFG0_MUX_MASK,
				   ADS1120_CFG0_MUX_AIN0_AVSS) |
			FIELD_PREP(ADS1120_CFG0_GAIN_MASK,
				   ADS1120_CFG0_GAIN_1) |
			ADS1120_CFG0_PGA_BYPASS;
	ret = ads1120_write_reg(st, ADS1120_REG_CONFIG0, st->config[0]);
	if (ret)
		return ret;

	st->config[1] = FIELD_PREP(ADS1120_CFG1_DR_MASK,
				   ADS1120_CFG1_DR_175SPS) |
			FIELD_PREP(ADS1120_CFG1_MODE_MASK,
				   ADS1120_CFG1_MODE_NORMAL) |
			FIELD_PREP(ADS1120_CFG1_CM_MASK,
				   ADS1120_CFG1_CM_SINGLE) |
			FIELD_PREP(ADS1120_CFG1_TS_EN, 0) |
			FIELD_PREP(ADS1120_CFG1_BCS_EN, 0);
	ret = ads1120_write_reg(st, ADS1120_REG_CONFIG1, st->config[1]);
	if (ret)
		return ret;

	config2 = FIELD_PREP(ADS1120_CFG2_VREF_MASK,
		   	     ADS1120_CFG2_VREF_AVDD) |
		  FIELD_PREP(ADS1120_CFG2_REJECT_MASK,
		             ADS1120_CFG2_REJECT_OFF) |
		  FIELD_PREP(ADS1120_CFG2_PSW_EN, 0) |
		  FIELD_PREP(ADS1120_CFG2_IDAC_MASK,
		             ADS1120_CFG2_IDAC_OFF);
	ret = ads1120_write_reg(st, ADS1120_REG_CONFIG2, config2);
	if (ret)
		return ret;

	config3 = FIELD_PREP(ADS1120_CFG3_IDAC1_MASK,
			     ADS1120_CFG3_IDAC1_DISABLED) |
		  FIELD_PREP(ADS1120_CFG3_IDAC2_MASK,
			     ADS1120_CFG3_IDAC2_DISABLED) |
		  FIELD_PREP(ADS1120_CFG3_DRDYM_MASK,
			     ADS1120_CFG3_DRDYM_DRDY_ONLY);
	ret = ads1120_write_reg(st, ADS1120_REG_CONFIG3, config3);
	if (ret)
		return ret;

	return 0;
}

static int ads1120_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ads1120_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	indio_dev->name = "ads1120";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads1120_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads1120_channels);
	indio_dev->info = &ads1120_info;

	ret = ads1120_init(st);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ads1120_of_match[] = {
	{ .compatible = "ti,ads1120" },
	{ }
};
MODULE_DEVICE_TABLE(of, ads1120_of_match);

static const struct spi_device_id ads1120_id[] = {
	{ "ads1120" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads1120_id);

static struct spi_driver ads1120_driver = {
	.driver = {
		.name = "ads1120",
		.of_match_table = ads1120_of_match,
	},
	.probe = ads1120_probe,
	.id_table = ads1120_id,
};
module_spi_driver(ads1120_driver);

MODULE_AUTHOR("Ajith Anandhan <ajithanandhan0406@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADS1120 ADC driver");
MODULE_LICENSE("GPL");
