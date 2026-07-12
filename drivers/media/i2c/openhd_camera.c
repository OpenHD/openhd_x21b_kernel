/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/clk.h>
#include <asm/unaligned.h>
#include <linux/gpio/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-fwnode.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <linux/delay.h>

#include "openhd_camera.h"

#define OPENHD_WIDTH 1280
#define OPENHD_HEIGHT 720
#define RUNCAM_REG_ADDR_LEN	3
#define RUNCAM_REG_VAL_LEN	4

/* Link frequency for MIPI CSI (in Hz) */
#define OPENHD_LINK_FREQ_300MHZ    300000000
#define OPENHD_PIXEL_RATE         (OPENHD_LINK_FREQ_300MHZ * 2)

static const s64 link_freq_menu_items[] = {
	OPENHD_LINK_FREQ_300MHZ,
};

struct openhd_camera_dev {
	struct v4l2_subdev sd;
	struct media_pad pad;

	const struct runcam_model_info *model;
	const struct runcam_mode *cur_mode;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	s64 link_freq_menu;

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

static int openhd_camera_read32(struct i2c_client *client, u32 reg, u32 *val)
{
	u8 addr[RUNCAM_REG_ADDR_LEN];
	u8 data[RUNCAM_REG_VAL_LEN];
	struct i2c_msg msgs[2];
	int ret;

	addr[0] = reg >> 16;
	addr[1] = reg >> 8;
	addr[2] = reg;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr);
	msgs[0].buf = addr;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(data);
	msgs[1].buf = data;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

	*val = get_unaligned_be32(data);

	return 0;
}

static int openhd_camera_write32(struct i2c_client *client, u32 reg, u32 val)
{
	u8 buf[RUNCAM_REG_ADDR_LEN + RUNCAM_REG_VAL_LEN];
	int ret;

	buf[0] = reg >> 16;
	buf[1] = reg >> 8;
	buf[2] = reg;
	put_unaligned_be32(val, &buf[3]);

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret != sizeof(buf))
		return ret < 0 ? ret : -EIO;

	return 0;
}

static u64 openhd_pixel_rate(const struct runcam_mode *mode, u32 lanes)
{
	return (u64)mode->link_freq_hz * lanes / 8;
}

static void openhd_update_link_controls(struct openhd_camera_dev *sensor)
{
	u64 prate = openhd_pixel_rate(sensor->cur_mode, sensor->csi_lanes);

	sensor->link_freq_menu = (s64)sensor->cur_mode->link_freq_hz;

	if (sensor->link_freq)
		*sensor->link_freq->p_new.p_s64 = sensor->link_freq_menu;

	if (sensor->pixel_rate)
		sensor->pixel_rate->val = (s32)min_t(u64, prate, S32_MAX);
}

static int openhd_camera_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_format *fmt)
{
	struct openhd_camera_dev *sensor =
        container_of(sd, struct openhd_camera_dev, sd);
	fmt->format = sensor->fmt;

	return 0;
}

static int  openhd_camera_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct openhd_camera_dev *sensor =
        container_of(sd, struct openhd_camera_dev, sd);
    fi->interval.numerator = 1;
	fi->interval.denominator = sensor->cur_mode->fps;

	return 0;
}

static const struct runcam_mode *openhd_find_best_mode(const struct runcam_model_info *model,
                      u32 width, u32 height, u32 code)
{
	const struct runcam_mode *best = &model->modes[0];
	unsigned int i;

	for (i = 0; i < model->num_modes; i++) {
		const struct runcam_mode *m = &model->modes[i];
		if (m->width == width && m->height == height)
			best = m;
	}
	return best;
}

static int openhd_camera_set_fmt(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_state *state,
                                  struct v4l2_subdev_format *fmt)
{
	struct openhd_camera_dev *sensor =
		container_of(sd, struct openhd_camera_dev, sd);
	const struct runcam_mode *mode;

	mode = openhd_find_best_mode(sensor->model, fmt->format.width, fmt->format.height, fmt->format.code);
	if (!mode)
		mode = &sensor->model->modes[0];  /* fallback */

	fmt->format.width     = mode->width;
	fmt->format.height    = mode->height;
	fmt->format.code      = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->format.field     = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (sensor->streaming)
			return -EBUSY;

		sensor->cur_mode = mode;
		sensor->fmt      = fmt->format;

		openhd_update_link_controls(sensor);
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
	struct i2c_client *client = v4l2_get_subdevdata(sd);
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

		if (!sensor->model->skip_detect && sensor->cur_mode->mode_reg)
			openhd_camera_write32(client, 0x000008,
				sensor->cur_mode->mode_reg);

		if (sensor->cur_mode->timing_reg)
			openhd_camera_write32(client, 0x000034,
					sensor->cur_mode->timing_reg);

		if (sensor->cur_mode->needs_isp_reset)
			openhd_camera_write32(client,
					0x000694,
					0x00000130);

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

	fse->min_width = 100;
	fse->max_width = 1920;
	fse->min_height = 100;
	fse->max_height = 1080;

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

static bool openhd_runcam_valid_read(u32 val)
{
	return val != 0x00000000 && val != 0xffffffff;
}

static int openhd_runcam_detect(struct i2c_client *client,
				struct openhd_camera_dev *sensor)
{
	const struct runcam_model_info *model = sensor->model;
	u32 val;
	int ret;

	if (!model)
		return -EINVAL;

	if (client->addr != model->i2c_addr) {
		dev_err(&client->dev,
			"DT compatible %s expects i2c addr 0x%02x, got 0x%02x\n",
			model->name, model->i2c_addr, client->addr);
		return -ENODEV;
	}

	if (model->skip_detect) {
		dev_info(&client->dev,
			 "skipping register detect for %s\n",
			 model->name);
		return 0;
	}

	if (model->detect_by_write) {
		ret = openhd_camera_write32(client, model->detect_reg,
					    model->detect_value);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "failed to detect %s by write\n",
					     model->name);

		return 0;
	}

	ret = openhd_camera_read32(client, model->detect_reg, &val);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read detect reg for %s\n",
				     model->name);

	if (!openhd_runcam_valid_read(val)) {
		dev_err(&client->dev,
			"invalid detect value for %s: 0x%08x\n",
			model->name, val);
		return -ENODEV;
	}

	dev_dbg(&client->dev, "%s detect reg 0x%x = 0x%08x\n",
		model->name, model->detect_reg, val);

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

	sensor->model = device_get_match_data(&client->dev);
	if (!sensor->model)
		return -ENODEV;

	ret = openhd_runcam_detect(client, sensor);
	if (ret) {
		dev_err(&client->dev,
			"failed to detect supported RunCam at i2c addr 0x%02x\n",
			client->addr);
		return ret;
	}


	sensor->cur_mode = &sensor->model->modes[0];

	dev_info(&client->dev,
	 "detected RunCam %s at 0x%02x, link_freq=%llu Hz\n",
	 sensor->model->name,
	 client->addr,
	 (unsigned long long)sensor->cur_mode->link_freq_hz);

	/* Initialize controls */
	sensor->link_freq_menu = (s64)sensor->cur_mode->link_freq_hz;

	v4l2_ctrl_handler_init(&sensor->ctrl_handler, 2);

	sensor->link_freq = v4l2_ctrl_new_int_menu(&sensor->ctrl_handler, NULL, V4L2_CID_LINK_FREQ, 0, 0, &sensor->link_freq_menu);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sensor->pixel_rate = v4l2_ctrl_new_std(&sensor->ctrl_handler, NULL, V4L2_CID_PIXEL_RATE, 0, openhd_pixel_rate(sensor->cur_mode, sensor->csi_lanes), 1, openhd_pixel_rate(sensor->cur_mode, sensor->csi_lanes));
	if (sensor->pixel_rate)
		sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sd->ctrl_handler = &sensor->ctrl_handler;
	if (sensor->ctrl_handler.error) {
		ret = sensor->ctrl_handler.error;
		goto err_free_handler;
	}

	/* Initialize format */
	sensor->fmt.width = sensor->cur_mode->width;
	sensor->fmt.height = sensor->cur_mode->height;
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

	dev_info(&client->dev, "OpenHD RunCam %s camera probe successful: %ux%u, format UYVY, %d CSI lanes\n",
		 sensor->model->name, sensor->fmt.width, sensor->fmt.height, sensor->csi_lanes);

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
	{ .compatible = "runcam,micro-v1", .data = &runcam_micro_v1_info },
	{ .compatible = "runcam,micro-v2", .data = &runcam_micro_v2_info },
	{ .compatible = "runcam,nano-90",  .data = &runcam_nano90_info  },
	{ .compatible = "runcam,micro-v3", .data = &runcam_micro_v3_info },
	{ .compatible = "foxeer,digisight-v3", .data = &foxeer_digisight_v3_info },
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
