#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define TEMP_RAW_SHIFT      5
#define TEMP_RAW_MASK       ((1 << 11) - 1)
#define TEMP_SCALE_MUL      200000
#define TEMP_SCALE_DIV      2048
#define TEMP_OFFSET         -50000

#define PRESS_RAW_SHIFT     0
#define PRESS_RAW_MASK      ((1 << 14) - 1)
#define PRESS_RAW_MIN       1638
#define PRESS_RAW_MAX       14745
#define PRESS_SCALE_DIV     (PRESS_RAW_MAX - PRESS_RAW_MIN)

// Scale mBar to kPascal as required by IIO ABI
#define PRESS_SCALE_DIV_KPA (PRESS_SCALE_DIV * 10)

typedef enum {
	AMS5915_SCAN_PRESS = 0,
	AMS5915_SCAN_TEMP,
	AMS5915_SCAN_TIMESTAMP,
} AMS5915_SCAN_T;

typedef struct {
	int pmin;
	int pmax;
} AMS5915_TYPEDEF_T;

typedef struct {
	struct i2c_client *client;
	int press_scale_mul;
	int press_offset;
} AMS5915_DATA_T;

typedef struct {
	uint32_t press;
	uint32_t temp;
	int64_t timestamp;
} AMS5915_RAW_T;

typedef enum {
	AMS5915_0005_D_TYPE = 0,
	AMS5915_0010_D_TYPE,
	AMS5915_0005_D_B_TYPE,
	AMS5915_0010_D_B_TYPE,
	AMS5915_0020_D_TYPE,
	AMS5915_0050_D_TYPE,
	AMS5915_0100_D_TYPE,
	AMS5915_0020_D_B_TYPE,
	AMS5915_0050_D_B_TYPE,
	AMS5915_0100_D_B_TYPE,
	AMS5915_0200_D_TYPE,
	AMS5915_0350_D_TYPE,
	AMS5915_1000_D_TYPE,
	AMS5915_2000_D_TYPE,
	AMS5915_4000_D_TYPE,
	AMS5915_7000_D_TYPE,
	AMS5915_10000_D_TYPE,
	AMS5915_0200_D_B_TYPE,
	AMS5915_0350_D_B_TYPE,
	AMS5915_1000_D_B_TYPE,
	AMS5915_1000_A_TYPE,
	AMS5915_1200_B_TYPE,
	AMS5915_TYPE_COUNT
} AMS5915_TYPE_T;

static const AMS5915_TYPEDEF_T ams5915_types[AMS5915_TYPE_COUNT] = {
	{ .pmin = 0, .pmax = 5 },
	{ .pmin = 0, .pmax = 10 },
	{ .pmin = -5, .pmax = 5 },
	{ .pmin = -10, .pmax = 10 },
	{ .pmin = 0, .pmax = 20 },
	{ .pmin = 0, .pmax = 50 },
	{ .pmin = 0, .pmax = 100 },
	{ .pmin = -20, .pmax = 20 },
	{ .pmin = -50, .pmax = 50 },
	{ .pmin = -100, .pmax = 100 },
	{ .pmin = 0, .pmax = 200 },
	{ .pmin = 0, .pmax = 350 },
	{ .pmin = 0, .pmax = 1000 },
	{ .pmin = 0, .pmax = 2000 },
	{ .pmin = 0, .pmax = 4000 },
	{ .pmin = 0, .pmax = 7000 },
	{ .pmin = 0, .pmax = 10000 },
	{ .pmin = -200, .pmax = 200 },
	{ .pmin = -350, .pmax = 350 },
	{ .pmin = -1000, .pmax = 1000 },
	{ .pmin = 0, .pmax = 1000 },
	{ .pmin = 700, .pmax = 1200 }
};

static int ams5915_read(struct iio_dev *indio_dev, AMS5915_RAW_T *raw)
{
	AMS5915_DATA_T *data = iio_priv(indio_dev);
	int ret;
	uint16_t buf[2];

	ret = i2c_master_recv(data->client, (char *)&buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}
	if (ret != sizeof(buf)) {
		return -EIO;
	}

	raw->press = (be16_to_cpu(buf[0]) >> PRESS_RAW_SHIFT) & PRESS_RAW_MASK;
	raw->temp = (be16_to_cpu(buf[1]) >> TEMP_RAW_SHIFT) & TEMP_RAW_MASK;
	raw->timestamp = iio_get_time_ns(indio_dev);

	return 0;
}

static irqreturn_t ams5915_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	AMS5915_RAW_T raw;
	int ret;

	ret = ams5915_read(indio_dev, &raw);
	if (ret < 0) {
		goto fail0;
	}

	iio_push_to_buffers(indio_dev, &raw);

fail0:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ams5915_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	AMS5915_DATA_T *data = iio_priv(indio_dev);
	int ret;
	AMS5915_RAW_T raw;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ams5915_read(indio_dev, &raw);
		if (ret) {
			return ret;
		}

		switch (chan->type) {
		case IIO_TEMP:
			*val = raw.temp;
			return IIO_VAL_INT;

		case IIO_PRESSURE:
			*val = raw.press;
			return IIO_VAL_INT;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_PROCESSED:
		ret = ams5915_read(indio_dev, &raw);
		if (ret) {
			return ret;
		}

		switch (chan->type) {
		case IIO_TEMP:
			*val = (raw.temp * TEMP_SCALE_MUL / TEMP_SCALE_DIV) + TEMP_OFFSET;
			return IIO_VAL_INT;

		case IIO_PRESSURE:
			*val = raw.press * data->press_scale_mul + data->press_offset;
			*val2 = PRESS_SCALE_DIV_KPA;
			return IIO_VAL_FRACTIONAL;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_TEMP:
			*val = TEMP_SCALE_MUL;
			*val2 = TEMP_SCALE_DIV;
			return IIO_VAL_FRACTIONAL;

		case IIO_PRESSURE:
			*val = data->press_scale_mul;
			*val2 = PRESS_SCALE_DIV_KPA;
			return IIO_VAL_FRACTIONAL;

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val = TEMP_OFFSET;
			return IIO_VAL_INT;

		case IIO_PRESSURE:
			*val = data->press_offset;
			*val2 = PRESS_SCALE_DIV_KPA;
			return IIO_VAL_FRACTIONAL;

		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static const struct iio_info ams5915_info = {
	.read_raw = ams5915_read_raw,
};

static const unsigned long ams5915_scan_masks[] =
	{ ((1 << AMS5915_SCAN_TIMESTAMP) - 1), 0};

static const struct iio_chan_spec ams5915_channels[] = {
	{
		.type = IIO_PRESSURE,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = AMS5915_SCAN_PRESS,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_CPU,
		},
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = AMS5915_SCAN_TEMP,
		.scan_type = {
			.sign = 'u',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_CPU,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(AMS5915_SCAN_TIMESTAMP)
};

static int ams5915_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int type_id;
	struct iio_dev *indio_dev;
	const AMS5915_TYPEDEF_T *type;
	AMS5915_DATA_T *data;
	int ret;
	AMS5915_RAW_T raw;

	type_id = (int) id->driver_data;
	if (type_id < 0 || type_id >= AMS5915_TYPE_COUNT) {
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		return -EOPNOTSUPP;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(AMS5915_DATA_T));
	if (indio_dev == NULL) {
		return -ENOMEM;
	}

	type = &ams5915_types[type_id];
	data = iio_priv(indio_dev);
	data->client = client;
	data->press_scale_mul = type->pmax - type->pmin;
	data->press_offset = type->pmin * PRESS_SCALE_DIV - PRESS_RAW_MIN * data->press_scale_mul;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = id->name;
	indio_dev->info = &ams5915_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ams5915_channels;
	indio_dev->num_channels = ARRAY_SIZE(ams5915_channels);
	indio_dev->available_scan_masks = ams5915_scan_masks;

	// try to read data
	ret = ams5915_read(indio_dev, &raw);
	if (ret < 0) {
		dev_err(&client->dev, "Error on initial read from sensor.\n");
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(
		&client->dev, indio_dev, NULL, ams5915_trigger_handler, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "IIO triggered buffer setup failed\n");
		return ret;
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register iio device\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ams5915_of_i2c_match[] = {
	{ .compatible = "amsys,ams5915-0005-d" },
	{ .compatible = "amsys,ams5915-0010-d" },
	{ .compatible = "amsys,ams5915-0005-d-b" },
	{ .compatible = "amsys,ams5915-0010-d-b" },
	{ .compatible = "amsys,ams5915-0020-d" },
	{ .compatible = "amsys,ams5915-0050-d" },
	{ .compatible = "amsys,ams5915-0100-d" },
	{ .compatible = "amsys,ams5915-0020-d-b" },
	{ .compatible = "amsys,ams5915-0050-d-b" },
	{ .compatible = "amsys,ams5915-0100-d-b" },
	{ .compatible = "amsys,ams5915-0200-d" },
	{ .compatible = "amsys,ams5915-0350-d" },
	{ .compatible = "amsys,ams5915-1000-d" },
	{ .compatible = "amsys,ams5915-2000-d" },
	{ .compatible = "amsys,ams5915-4000-d" },
	{ .compatible = "amsys,ams5915-7000-d" },
	{ .compatible = "amsys,ams5915-10000-d" },
	{ .compatible = "amsys,ams5915-0200-d-b" },
	{ .compatible = "amsys,ams5915-0350-d-b" },
	{ .compatible = "amsys,ams5915-1000-d-b" },
	{ .compatible = "amsys,ams5915-1000-a" },
	{ .compatible = "amsys,ams5915-1200-b" },
	{ },
};
MODULE_DEVICE_TABLE(of, ams5915_of_i2c_match);
#else
#define ams5915_of_i2c_match NULL
#endif

static const struct i2c_device_id ams5915_i2c_id[] = {
	{ "ams5915-0005-d", AMS5915_0005_D_TYPE },
	{ "ams5915-0010-d", AMS5915_0010_D_TYPE },
	{ "ams5915-0005-d-b", AMS5915_0005_D_B_TYPE },
	{ "ams5915-0010-d-b", AMS5915_0010_D_B_TYPE },
	{ "ams5915-0020-d", AMS5915_0020_D_TYPE },
	{ "ams5915-0050-d", AMS5915_0050_D_TYPE },
	{ "ams5915-0100-d", AMS5915_0100_D_TYPE },
	{ "ams5915-0020-d-b", AMS5915_0020_D_B_TYPE },
	{ "ams5915-0050-d-b", AMS5915_0050_D_B_TYPE },
	{ "ams5915-0100-d-b", AMS5915_0100_D_B_TYPE },
	{ "ams5915-0200-d", AMS5915_0200_D_TYPE },
	{ "ams5915-0350-d", AMS5915_0350_D_TYPE },
	{ "ams5915-1000-d", AMS5915_1000_D_TYPE },
	{ "ams5915-2000-d", AMS5915_2000_D_TYPE },
	{ "ams5915-4000-d", AMS5915_4000_D_TYPE },
	{ "ams5915-7000-d", AMS5915_7000_D_TYPE },
	{ "ams5915-10000-d", AMS5915_10000_D_TYPE },
	{ "ams5915-0200-d-b", AMS5915_0200_D_B_TYPE },
	{ "ams5915-0350-d-b", AMS5915_0350_D_B_TYPE },
	{ "ams5915-1000-d-b", AMS5915_1000_D_B_TYPE },
	{ "ams5915-1000-a", AMS5915_1000_A_TYPE },
	{ "ams5915-1200-b", AMS5915_1200_B_TYPE },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ams5915_i2c_id);

static struct i2c_driver ams5915_i2c_driver = {
	.driver = {
		.name	= "ams5915",
		.of_match_table = of_match_ptr(ams5915_of_i2c_match),
	},
	.probe		= ams5915_i2c_probe,
	.id_table	= ams5915_i2c_id,
};
module_i2c_driver(ams5915_i2c_driver);

MODULE_AUTHOR("Sascha Ittner <sascha.ittner@modusoft.de>");
MODULE_DESCRIPTION("Driver for AMSYS AMS5915 pressure sensors");
MODULE_LICENSE("GPL v2");

