// SPDX-License-Identifier: GPL-2.0
/*
 * envcombo.c - IIO driver for the ENV-COMBO sensor (ambient light channel)
 *
 * Only the ALS (IIO_LIGHT) functionality of the ENV-COMBO is implemented;
 * the temperature and humidity channels are out of scope for this driver.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define ENVCOMBO_WHO_AM_I_VAL			0xEB
#define ENVCOMBO_REG_WHO_AM_I			0x00
#define ENVCOMBO_REG_ALS_MSB			0x04
#define ENVCOMBO_REG_CFG				0x06
#define ENVCOMBO_REG_INT_CFG			0x07
#define ENVCOMBO_REG_ALS_TH_LOW			0x08
#define ENVCOMBO_REG_ALS_TH_HIGH		0x0A
#define ENVCOMBO_REG_STATUS				0x0C
#define ENVCOMBO_REG_CAL_ALS_GAIN		0x10
#define ENVCOMBO_REG_CAL_ALS_TIME		0x11
#define ENVCOMBO_REG_PWR_MODE			0x12

#define ENVCOMBO_PWR_SLEEP				0x01
#define ENVCOMBO_PWR_ONE_SHOT			0x02
#define ENVCOMBO_PWR_CONTINUOUS			0x03

#define ENVCOMBO_STATUS_ALS_INT			BIT(0)
#define ENVCOMBO_STATUS_ALS_RDY			BIT(3)
#define ENVCOMBO_INT_CFG_LATCH			BIT(6)
#define ENVCOMBO_INT_CFG_EN				BIT(7)
#define ENVCOMBO_CFG_ALS_EN				BIT(7)

#define ENVCOMBO_CFG_ALS_GAIN_MASK		GENMASK(4, 3)
#define ENVCOMBO_CFG_ALS_TIME_MASK		GENMASK(2, 1)
#define ENVCOMBO_PWR_MODE_MASK			GENMASK(1, 0)

/* Defaults written to CFG[4:3]/[2:1] at probe: gain x1, integration time 200ms */
#define ENVCOMBO_DEFAULT_GAIN_IDX		0
#define ENVCOMBO_DEFAULT_TIME_IDX		2
#define ENVCOMBO_RAW_READ_TIMEOUT_MS	500

static const int envcombo_als_gain_table[] = { 1, 4, 16, 64 };
static const int envcombo_als_time_table_us[] = {
	50000, 100000, 200000, 400000,
};

/* IIO_VAL_INT_PLUS_MICRO available list: (integer, micro) pairs */
static const int envcombo_als_time_avail[] = {
	0, 50000,
	0, 100000,
	0, 200000,
	0, 400000,
};

/*
 * Plain i2c_smbus access: the device exposes only 19 byte-wide registers
 * (with three 16-bit big-endian fields), so a full regmap isn't warranted.
 * CONFIG_REGMAP_I2C is also a hidden Kconfig symbol with no prompt and can
 * only be pulled in via "select" from another driver, which an out-of-tree
 * module cannot do cleanly.
 */
static int envcombo_read_reg(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int envcombo_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int envcombo_update_bits(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	int ret;

	ret = envcombo_read_reg(client, reg);
	if (ret < 0)
		return ret;

	return envcombo_write_reg(client, reg, (ret & ~mask) | (val & mask));
}

/*
 * 16-bit big-endian register pair (MSB first), accessed as a single bus
 * transaction so the device cannot update the pair (e.g. latch a new ALS
 * sample in continuous mode) between the two byte accesses.
 */
static int envcombo_read_reg16(struct i2c_client *client, u8 reg, u16 *val)
{
	int ret;

	ret = i2c_smbus_read_word_swapped(client, reg);
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}

static int envcombo_write_reg16(struct i2c_client *client, u8 reg, u16 val)
{
	return i2c_smbus_write_word_swapped(client, reg, val);
}

struct envcombo_data {
	struct i2c_client *client;
	struct mutex lock;
	struct completion als_done;
	struct iio_trigger *trig;

	u8 als_gain_idx;
	u8 als_time_idx;
	u8 calib_again;
	u8 calib_atime;
	/* (integer, micro) pair advertised when CAL_ATIME fixes the time */
	int calib_time_avail[2];

	u16 thresh_low;
	u16 thresh_high;

	bool ev_en;
	bool buffer_en;

	struct {
		u16 light;
		s64 timestamp;
	} scan __aligned(8);
};

/*
 * The device signals a threshold crossing with a single direction-less
 * ALS_INT bit, so the low/high threshold values are exposed through the
 * falling/rising VALUE attributes while the event itself is enabled and
 * reported with IIO_EV_DIR_EITHER.
 */
static const struct iio_event_spec envcombo_als_event_specs[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec envcombo_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				       BIT(IIO_CHAN_INFO_SCALE) |
				       BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				       BIT(IIO_CHAN_INFO_INT_TIME),
		.info_mask_separate_available =
				       BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				       BIT(IIO_CHAN_INFO_INT_TIME),
		.event_spec = envcombo_als_event_specs,
		.num_event_specs = ARRAY_SIZE(envcombo_als_event_specs),
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

/* Caller must hold data->lock. */
static int envcombo_update_power_mode(struct envcombo_data *data)
{
	bool active = data->buffer_en || data->ev_en;

	return envcombo_update_bits(data->client, ENVCOMBO_REG_PWR_MODE,
				     ENVCOMBO_PWR_MODE_MASK,
				     active ? ENVCOMBO_PWR_CONTINUOUS :
					      ENVCOMBO_PWR_SLEEP);
}

static int envcombo_read_als_raw(struct envcombo_data *data, int *val)
{
	u16 light;
	int ret;

	mutex_lock(&data->lock);

	if (!data->buffer_en && !data->ev_en) {
		reinit_completion(&data->als_done);

		ret = envcombo_write_reg(data->client, ENVCOMBO_REG_PWR_MODE,
					  ENVCOMBO_PWR_ONE_SHOT);
		if (ret < 0)
			goto out_unlock;

		if (!wait_for_completion_timeout(&data->als_done,
				msecs_to_jiffies(ENVCOMBO_RAW_READ_TIMEOUT_MS))) {
			ret = -ETIMEDOUT;
			goto out_unlock;
		}
	}

	ret = envcombo_read_reg16(data->client, ENVCOMBO_REG_ALS_MSB, &light);
	if (ret)
		goto out_unlock;

	*val = light;
	ret = 0;

out_unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static int envcombo_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct envcombo_data *data = iio_priv(indio_dev);
	int gain;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return envcombo_read_als_raw(data, val) ?: IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		gain = envcombo_als_gain_table[data->als_gain_idx] *
		       (data->calib_again ?: 1);
		*val = 1;
		*val2 = gain;
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = envcombo_als_gain_table[data->als_gain_idx];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_INT_TIME:
		*val = 0;
		*val2 = data->calib_atime ? data->calib_atime * 1000 :
			envcombo_als_time_table_us[data->als_time_idx];
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static int envcombo_read_avail(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				const int **vals, int *type, int *length,
				long mask)
{
	struct envcombo_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*vals = envcombo_als_gain_table;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(envcombo_als_gain_table);
		return IIO_AVAIL_LIST;

	case IIO_CHAN_INFO_INT_TIME:
		if (data->calib_atime) {
			*vals = data->calib_time_avail;
			*type = IIO_VAL_INT_PLUS_MICRO;
			*length = ARRAY_SIZE(data->calib_time_avail);
			return IIO_AVAIL_LIST;
		}
		*vals = envcombo_als_time_avail;
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = ARRAY_SIZE(envcombo_als_time_avail);
		return IIO_AVAIL_LIST;

	default:
		return -EINVAL;
	}
}

static int envcombo_write_raw_get_fmt(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return IIO_VAL_INT;
	}
}

static int envcombo_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct envcombo_data *data = iio_priv(indio_dev);
	int ret, idx, i;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		idx = -1;
		for (i = 0; i < ARRAY_SIZE(envcombo_als_gain_table); i++) {
			if (envcombo_als_gain_table[i] == val) {
				idx = i;
				break;
			}
		}
		if (idx < 0)
			return -EINVAL;

		mutex_lock(&data->lock);
		ret = envcombo_update_bits(data->client, ENVCOMBO_REG_CFG,
					    ENVCOMBO_CFG_ALS_GAIN_MASK,
					    FIELD_PREP(ENVCOMBO_CFG_ALS_GAIN_MASK, idx));
		if (!ret)
			data->als_gain_idx = idx;
		mutex_unlock(&data->lock);
		return ret;

	case IIO_CHAN_INFO_INT_TIME:
		if (data->calib_atime)
			return -EOPNOTSUPP;

		idx = -1;
		if (val == 0) {
			for (i = 0; i < ARRAY_SIZE(envcombo_als_time_table_us); i++) {
				if (envcombo_als_time_table_us[i] == val2) {
					idx = i;
					break;
				}
			}
		}
		if (idx < 0)
			return -EINVAL;

		mutex_lock(&data->lock);
		ret = envcombo_update_bits(data->client, ENVCOMBO_REG_CFG,
					    ENVCOMBO_CFG_ALS_TIME_MASK,
					    FIELD_PREP(ENVCOMBO_CFG_ALS_TIME_MASK, idx));
		if (!ret)
			data->als_time_idx = idx;
		mutex_unlock(&data->lock);
		return ret;

	default:
		return -EINVAL;
	}
}

static int envcombo_read_event_value(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      enum iio_event_info info,
				      int *val, int *val2)
{
	struct envcombo_data *data = iio_priv(indio_dev);

	switch (dir) {
	case IIO_EV_DIR_RISING:
		*val = READ_ONCE(data->thresh_high);
		break;
	case IIO_EV_DIR_FALLING:
		*val = READ_ONCE(data->thresh_low);
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int envcombo_write_event_value(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       enum iio_event_info info,
				       int val, int val2)
{
	struct envcombo_data *data = iio_priv(indio_dev);
	int ret;

	if (val < 0 || val > 0xFFFF)
		return -EINVAL;

	mutex_lock(&data->lock);

	switch (dir) {
	case IIO_EV_DIR_RISING:
		if (val < data->thresh_low) {
			ret = -EINVAL;
			break;
		}
		ret = envcombo_write_reg16(data->client, ENVCOMBO_REG_ALS_TH_HIGH,
					    val);
		if (!ret)
			WRITE_ONCE(data->thresh_high, val);
		break;

	case IIO_EV_DIR_FALLING:
		if (val > data->thresh_high) {
			ret = -EINVAL;
			break;
		}
		ret = envcombo_write_reg16(data->client, ENVCOMBO_REG_ALS_TH_LOW,
					    val);
		if (!ret)
			WRITE_ONCE(data->thresh_low, val);
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&data->lock);
	return ret;
}

static int envcombo_read_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir)
{
	struct envcombo_data *data = iio_priv(indio_dev);

	return READ_ONCE(data->ev_en);
}

static int envcombo_write_event_config(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir,
					int state)
{
	struct envcombo_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	WRITE_ONCE(data->ev_en, !!state);
	ret = envcombo_update_power_mode(data);
	mutex_unlock(&data->lock);

	return ret;
}

static const struct iio_info envcombo_info = {
	.read_raw = envcombo_read_raw,
	.read_avail = envcombo_read_avail,
	.write_raw = envcombo_write_raw,
	.write_raw_get_fmt = envcombo_write_raw_get_fmt,
	.read_event_value = envcombo_read_event_value,
	.write_event_value = envcombo_write_event_value,
	.read_event_config = envcombo_read_event_config,
	.write_event_config = envcombo_write_event_config,
};

static int envcombo_set_trigger_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct envcombo_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	data->buffer_en = enable;
	ret = envcombo_update_power_mode(data);
	mutex_unlock(&data->lock);

	return ret;
}

static const struct iio_trigger_ops envcombo_trigger_ops = {
	.set_trigger_state = envcombo_set_trigger_state,
};

static irqreturn_t envcombo_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct envcombo_data *data = iio_priv(indio_dev);
	int ret;

	ret = envcombo_read_reg16(data->client, ENVCOMBO_REG_ALS_MSB,
				   &data->scan.light);
	/*
	 * Our own trigger fires via iio_trigger_poll_nested(), which skips
	 * the iio_pollfunc_store_time top half, so pf->timestamp is only
	 * populated when an external hard-irq trigger is in use.
	 */
	if (!ret)
		iio_push_to_buffers_with_timestamp(indio_dev, &data->scan,
						    pf->timestamp ?:
						    iio_get_time_ns(indio_dev));

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static irqreturn_t envcombo_irq_thread(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct envcombo_data *data = iio_priv(indio_dev);
	int status;

	status = envcombo_read_reg(data->client, ENVCOMBO_REG_STATUS);
	if (status < 0) {
		dev_warn_ratelimited(&data->client->dev,
				      "failed to read STATUS: %d\n", status);
		return IRQ_HANDLED;
	}

	if (status & ENVCOMBO_STATUS_ALS_RDY) {
		complete(&data->als_done);
		/*
		 * Threaded handler context: use the nested variant of
		 * iio_trigger_poll(), which is reserved for hard-irq context.
		 */
		if (READ_ONCE(data->buffer_en))
			iio_trigger_poll_nested(data->trig);
	}

	/*
	 * The hardware reports threshold crossings via a single ALS_INT
	 * bit with no direction information, so the event is reported as
	 * IIO_EV_DIR_EITHER.
	 *
	 * Avoid data->lock here: this is a threaded IRQF_ONESHOT handler,
	 * so blocking on a mutex held by the raw-read path (up to
	 * ENVCOMBO_RAW_READ_TIMEOUT_MS) would delay re-arming the
	 * interrupt. ev_en is snapshotted with READ_ONCE() instead,
	 * paired with WRITE_ONCE() on the sysfs write side.
	 */
	if ((status & ENVCOMBO_STATUS_ALS_INT) && READ_ONCE(data->ev_en))
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_LIGHT, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static int envcombo_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct envcombo_data *data;
	struct iio_dev *indio_dev;
	int ret;

	if (!client->irq)
		return -EINVAL;

	if (!i2c_check_functionality(client->adapter,
				      I2C_FUNC_SMBUS_BYTE_DATA |
				      I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);
	init_completion(&data->als_done);

	ret = envcombo_read_reg(client, ENVCOMBO_REG_WHO_AM_I);
	if (ret < 0)
		return ret;
	if (ret != ENVCOMBO_WHO_AM_I_VAL)
		return -ENODEV;

	indio_dev->name = "envcombo";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &envcombo_info;
	indio_dev->channels = envcombo_channels;
	indio_dev->num_channels = ARRAY_SIZE(envcombo_channels);

	ret = envcombo_read_reg(client, ENVCOMBO_REG_CAL_ALS_GAIN);
	if (ret < 0)
		return ret;
	data->calib_again = ret;

	ret = envcombo_read_reg(client, ENVCOMBO_REG_CAL_ALS_TIME);
	if (ret < 0)
		return ret;
	data->calib_atime = ret;
	if (data->calib_atime) {
		data->calib_time_avail[0] = 0;
		data->calib_time_avail[1] = data->calib_atime * 1000;
		dev_warn(dev,
			 "factory calibration overrides ALS integration time to %u ms; integration_time is read-only\n",
			 data->calib_atime);
	}

	data->als_gain_idx = ENVCOMBO_DEFAULT_GAIN_IDX;
	data->als_time_idx = ENVCOMBO_DEFAULT_TIME_IDX;

	ret = envcombo_write_reg(client, ENVCOMBO_REG_CFG,
				  ENVCOMBO_CFG_ALS_EN |
				  FIELD_PREP(ENVCOMBO_CFG_ALS_GAIN_MASK, data->als_gain_idx) |
				  FIELD_PREP(ENVCOMBO_CFG_ALS_TIME_MASK, data->als_time_idx));
	if (ret)
		return ret;

	/*
	 * INT_EN on, latched so a crossing is held until STATUS is read.
	 * The interrupt line pulses once per STATUS 0->1 transition, so
	 * INT_POL is left clear (active-low) and the IRQ is requested as
	 * falling-edge below.
	 */
	ret = envcombo_write_reg(client, ENVCOMBO_REG_INT_CFG,
				  ENVCOMBO_INT_CFG_EN | ENVCOMBO_INT_CFG_LATCH);
	if (ret)
		return ret;

	/* POR defaults: thresholds span the full range, i.e. disabled. */
	ret = envcombo_write_reg16(client, ENVCOMBO_REG_ALS_TH_LOW, 0x0000);
	if (ret)
		return ret;
	data->thresh_low = 0x0000;

	ret = envcombo_write_reg16(client, ENVCOMBO_REG_ALS_TH_HIGH, 0xFFFF);
	if (ret)
		return ret;
	data->thresh_high = 0xFFFF;

	/* Stay low-power until a raw read, buffer, or event is requested. */
	ret = envcombo_update_bits(client, ENVCOMBO_REG_PWR_MODE,
				    ENVCOMBO_PWR_MODE_MASK, ENVCOMBO_PWR_SLEEP);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					 envcombo_irq_thread,
					 IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					 "envcombo", indio_dev);
	if (ret)
		return ret;

	data->trig = devm_iio_trigger_alloc(dev, "%s-dev%d", indio_dev->name,
					     iio_device_id(indio_dev));
	if (!data->trig)
		return -ENOMEM;

	data->trig->ops = &envcombo_trigger_ops;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(dev, data->trig);
	if (ret)
		return ret;

	/*
	 * Default trigger: take a reference for indio_dev->trig; the IIO
	 * core drops it in iio_dev_release(), so no matching put here.
	 */
	indio_dev->trig = iio_trigger_get(data->trig);

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					       iio_pollfunc_store_time,
					       envcombo_trigger_handler, NULL);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id envcombo_id[] = {
	{ "envcombo" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, envcombo_id);

static struct i2c_driver envcombo_driver = {
	.driver = {
		.name = "envcombo",
	},
	.probe = envcombo_probe,
	.id_table = envcombo_id,
};
module_i2c_driver(envcombo_driver);

MODULE_AUTHOR("Gil Tabibian");
MODULE_DESCRIPTION("ENV-COMBO IIO ambient light sensor driver");
MODULE_LICENSE("GPL");
