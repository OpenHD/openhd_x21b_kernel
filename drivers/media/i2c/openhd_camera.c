/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-fwnode.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <linux/delay.h>

#define OPENHD_WIDTH 1280
#define OPENHD_HEIGHT 720

/* Link frequency for MIPI CSI (in Hz) */
#define OPENHD_LINK_FREQ_300MHZ    300000000
#define OPENHD_PIXEL_RATE         (OPENHD_LINK_FREQ_300MHZ * 2)

static const s64 link_freq_menu_items[] = {
	OPENHD_LINK_FREQ_300MHZ,
};

struct openhd_camera_dev {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;


	struct i2c_client *i2c_client;

	/* Device tree parsed values */
	u32 csi_lanes;
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;

	bool streaming;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
};

static int openhd_camera_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_format *fmt)
{
	fmt->format.width = OPENHD_WIDTH;
	fmt->format.height = OPENHD_HEIGHT;
	fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;  /* Fixed: use 2X8 format */
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_SRGB;

	return 0;
}

static int  openhd_camera_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
    fi->interval.numerator = 10000;
	fi->interval.denominator = 600000;

	return 0;
}

static int openhd_camera_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_format *fmt)
{
	struct openhd_camera_dev *sensor = container_of(sd, struct openhd_camera_dev, sd);

	/* Force our fixed format */
	fmt->format.width = OPENHD_WIDTH;
	fmt->format.height = OPENHD_HEIGHT;
	fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;  /* Fixed: use 2X8 format */
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_SRGB;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		mutex_lock(state->lock);
		sensor->fmt = fmt->format;
		mutex_unlock(state->lock);
	}

	return 0;
}

static int openhd_camera_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_2X8;
	return 0;
}

static int openhd_camera_get_mbus_config(struct v4l2_subdev *sd,
				unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	struct openhd_camera_dev *sensor = container_of(sd, struct openhd_camera_dev, sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = sensor->csi_lanes;
	cfg->bus.mipi_csi2.flags = 0; /* Continuous clock mode */

	return 0;
}

static int openhd_camera_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct openhd_camera_dev *sensor = container_of(sd, struct openhd_camera_dev, sd);
	int ret = 0;

	dev_info(sd->dev, "%s: %s streaming\n", __func__, enable ? "start" : "stop");

	if (enable) {
		/* Power on sequence */
		if (sensor->pwdn_gpio)
			gpiod_set_value_cansleep(sensor->pwdn_gpio, 0);

		if (sensor->xvclk) {
			ret = clk_prepare_enable(sensor->xvclk);
			if (ret) {
				dev_err(sd->dev, "Failed to enable clock\n");
				return ret;
			}
		}

		/* Reset sequence */
		if (sensor->reset_gpio) {
			gpiod_set_value_cansleep(sensor->reset_gpio, 1);
			usleep_range(1000, 2000);
			gpiod_set_value_cansleep(sensor->reset_gpio, 0);
			usleep_range(10000, 20000);
		}

		sensor->streaming = true;
		dev_info(sd->dev, "OpenHD camera streaming started\n");
	} else {
		sensor->streaming = false;

		/* Power down sequence */
		if (sensor->xvclk)
			clk_disable_unprepare(sensor->xvclk);

		if (sensor->pwdn_gpio)
			gpiod_set_value_cansleep(sensor->pwdn_gpio, 1);

		dev_info(sd->dev, "OpenHD camera streaming stopped\n");
	}

	return 0;
}

static int openhd_camera_s_power(struct v4l2_subdev *sd, int on)
{
	struct openhd_camera_dev *sensor = container_of(sd, struct openhd_camera_dev, sd);
	int ret = 0;

	dev_info(sd->dev, "%s: power %s\n", __func__, on ? "on" : "off");

	if (on) {
		if (sensor->xvclk) {
			ret = clk_prepare_enable(sensor->xvclk);
			if (ret) {
				dev_err(sd->dev, "Failed to enable xvclk\n");
				return ret;
			}
		}
	} else {
		if (sensor->xvclk)
			clk_disable_unprepare(sensor->xvclk);
	}

	return 0;
}

static int openhd_camera_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_state *state,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index != 0)
		return -EINVAL;

	if (fie->width != OPENHD_WIDTH || fie->height != OPENHD_HEIGHT)
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_UYVY8_2X8)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = 60;

	return 0;
}

static int openhd_camera_enum_frame_sizes(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_state *state,
	struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index != 0)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_UYVY8_2X8)
		return -EINVAL;

	fse->min_width = OPENHD_WIDTH;
	fse->max_width = OPENHD_WIDTH;
	fse->min_height = OPENHD_HEIGHT;
	fse->max_height = OPENHD_HEIGHT;

	return 0;
}

static const struct v4l2_subdev_video_ops openhd_camera_video_ops = {
	.s_stream = openhd_camera_s_stream,
	.g_frame_interval = openhd_camera_g_frame_interval,
};

static const struct v4l2_subdev_core_ops openhd_camera_core_ops = {
	.s_power = openhd_camera_s_power,
};

static const struct v4l2_subdev_pad_ops openhd_camera_pad_ops = {
	.get_fmt = openhd_camera_get_fmt,
	.set_fmt = openhd_camera_set_fmt,
	.enum_frame_size = openhd_camera_enum_frame_sizes,
	.enum_frame_interval = openhd_camera_enum_frame_interval,
	.enum_mbus_code = openhd_camera_enum_mbus_code,
	.get_mbus_config = openhd_camera_get_mbus_config,
};

static const struct v4l2_subdev_ops openhd_camera_subdev_ops = {
	.core = &openhd_camera_core_ops,
	.video = &openhd_camera_video_ops,
	.pad = &openhd_camera_pad_ops,
};

static int openhd_parse_dt(struct openhd_camera_dev *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_fwnode_endpoint endpoint = { .bus_type = 0 };
	struct device_node *ep;
	int ret;

	/* Parse module info */
	ret = of_property_read_u32(node, "rockchip,camera-module-index",
				   &sensor->module_index);
	ret |= of_property_read_string(node, "rockchip,camera-module-facing",
				       &sensor->module_facing);
	ret |= of_property_read_string(node, "rockchip,camera-module-name",
				       &sensor->module_name);
	ret |= of_property_read_string(node, "rockchip,camera-lens-name",
				       &sensor->len_name);
	if (ret) {
		dev_warn(dev, "Missing module information in DT\n");
		sensor->module_index = 0;
		sensor->module_facing = "back";
		sensor->module_name = "openhd-camera";
		sensor->len_name = "openhd-lens";
	}

	/* Get CSI endpoint */
	ep = of_graph_get_next_endpoint(node, NULL);
	if (!ep) {
		dev_err(dev, "Missing endpoint node\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep), &endpoint);
	of_node_put(ep);
	if (ret) {
		dev_err(dev, "Failed to parse endpoint\n");
		return ret;
	}

	if (endpoint.bus_type != V4L2_MBUS_CSI2_DPHY) {
		dev_err(dev, "Unsupported bus type %d\n", endpoint.bus_type);
		ret = -EINVAL;
		goto cleanup_endpoint;
	}

	sensor->csi_lanes = endpoint.bus.mipi_csi2.num_data_lanes;
	if (sensor->csi_lanes == 0 || sensor->csi_lanes > 4) {
		dev_err(dev, "Invalid number of CSI lanes: %d\n", sensor->csi_lanes);
		ret = -EINVAL;
		goto cleanup_endpoint;
	}

	dev_info(dev, "Using %d CSI lanes\n", sensor->csi_lanes);

cleanup_endpoint:
	v4l2_fwnode_endpoint_free(&endpoint);
	return ret;
}

static int openhd_get_regulators(struct openhd_camera_dev *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	struct device *dev = &client->dev;

	/* Get optional clock */
	sensor->xvclk = devm_clk_get_optional(dev, "xvclk");
	if (IS_ERR(sensor->xvclk))
		return PTR_ERR(sensor->xvclk);

	/* Get optional GPIOs */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return PTR_ERR(sensor->reset_gpio);

	sensor->pwdn_gpio = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->pwdn_gpio))
		return PTR_ERR(sensor->pwdn_gpio);

	return 0;
}

static int openhd_camera_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct openhd_camera_dev *sensor;
	struct v4l2_subdev *sd;
	struct fwnode_handle *fwnode = dev_fwnode(&client->dev);
	char facing[2];
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &openhd_camera_subdev_ops);

	/* Parse device tree */
	ret = openhd_parse_dt(sensor);
	if (ret)
		return ret;

	/* Get regulators and GPIOs */
	ret = openhd_get_regulators(sensor);
	if (ret)
		return ret;

	/* Initialize controls */
	v4l2_ctrl_handler_init(&sensor->ctrl_handler, 2);

	sensor->link_freq = v4l2_ctrl_new_int_menu(&sensor->ctrl_handler,
						   NULL,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq_menu_items) - 1,
						   0, link_freq_menu_items);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->pixel_rate = v4l2_ctrl_new_std(&sensor->ctrl_handler,
					       NULL,
					       V4L2_CID_PIXEL_RATE,
					       0, OPENHD_PIXEL_RATE, 1,
					       OPENHD_PIXEL_RATE);
	if (sensor->pixel_rate)
		sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sd->ctrl_handler = &sensor->ctrl_handler;
	if (sensor->ctrl_handler.error) {
		ret = sensor->ctrl_handler.error;
		goto err_free_handler;
	}

	/* Initialize format */
	sensor->fmt.width = OPENHD_WIDTH;
	sensor->fmt.height = OPENHD_HEIGHT;
	sensor->fmt.code = MEDIA_BUS_FMT_UYVY8_2X8;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->fmt.colorspace = V4L2_COLORSPACE_SRGB;
	sensor->fmt.ycbcr_enc = V4L2_YCBCR_ENC_601;
	sensor->fmt.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	sensor->fmt.xfer_func = V4L2_XFER_FUNC_SRGB;

	/* Initialize media entity */
	sd->owner = THIS_MODULE;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret)
		goto err_free_handler;

	sd->fwnode = fwnode;
	i2c_set_clientdata(client, sd);

	/* Create device name */
	memset(facing, 0, sizeof(facing));
	if (strcmp(sensor->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sensor->module_index, facing,
		 "openhd_camera", dev_name(sd->dev));

	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_clean_entity;

	dev_info(&client->dev, "OpenHD camera probe successful: %ux%u, format UYVY, %d CSI lanes\n",
		 OPENHD_WIDTH, OPENHD_HEIGHT, sensor->csi_lanes);

	return 0;

err_clean_entity:
	media_entity_cleanup(&sd->entity);
err_free_handler:
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
	return ret;
}

static void openhd_camera_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct openhd_camera_dev *sensor = container_of(sd, struct openhd_camera_dev, sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
	dev_info(&client->dev, "OpenHD camera removed\n");
}

static const struct i2c_device_id openhd_camera_id[] = {
	{"openhd_camera", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, openhd_camera_id);

static const struct of_device_id openhd_camera_of_match[] = {
	{ .compatible = "openhd,camera" },
	{ },
};
MODULE_DEVICE_TABLE(of, openhd_camera_of_match);

static struct i2c_driver openhd_camera_driver = {
	.driver = {
		.name = "openhd_camera",
		.of_match_table = openhd_camera_of_match,
	},
	.probe = openhd_camera_probe,
	.remove = openhd_camera_remove,
	.id_table = openhd_camera_id,
};

module_i2c_driver(openhd_camera_driver);

MODULE_DESCRIPTION("OpenHD UYVY Fake Sensor for Rockchip CSI");
MODULE_LICENSE("GPL v2");
