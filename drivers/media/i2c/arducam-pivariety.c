// SPDX-License-Identifier: GPL-2.0
/*
 * At this point more a form of art then a driver
 * Copyright (C) 2026 Luka Panio <lukapanio@gmail.com>
 *
 * A V4L2 driver for Arducam Pivariety Cameras
 * Copyright (C) 2022 Arducam Technology co., Ltd.
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 * I2C read and write method is taken from the OV9281 driver
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 */

#define RADXA
 #include <linux/crc32.h>
 #include <linux/clk.h>
 #include <linux/delay.h>
 #include <linux/gpio/consumer.h>
 #include <linux/i2c.h>
 #include <linux/module.h>
 #include <linux/pm_runtime.h>
 #include <linux/regulator/consumer.h>
 #include <linux/workqueue.h>
 #include <media/v4l2-ctrls.h>
 #include <media/v4l2-device.h>
 #include <media/v4l2-event.h>
 #include <media/v4l2-fwnode.h>
 #include <linux/of_graph.h>

 #ifdef RADXA 
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"
#endif

 #include "arducam-pivariety.h"
 #include "arducam-x21-fw.h"
 #if IS_ENABLED(CONFIG_ARDUCAM_OPENHD_EMBEDDED_FW)
 #include "arducam-x21-fw-data.h"
 #endif
 #include "arducam-x21-timing.h"
 #include "arducam-x21-hdr.h"
 #include "arducam-x21-isp.h"
 
 static int debug;
 module_param(debug, int, 0644);

 static bool x21_auto_firmware = true;
 module_param(x21_auto_firmware, bool, 0444);
 MODULE_PARM_DESC(x21_auto_firmware,
     "Update the MCU only on the explicitly selected X21 ARDUCAM_OPENHD board profile");

 static bool x21_linear_720p120 = true;
 module_param(x21_linear_720p120, bool, 0444);
 MODULE_PARM_DESC(x21_linear_720p120,
     "Apply the X21 four-lane 720p120 linear timing after preview settles");
 
 /* regulator supplies */
 static const char * const pivariety_supply_name[] = {
     /* Supplies can be enabled in any order */
     "VANA",  /* Analog (2.8V) supply */
     "VDIG",  /* Digital Core (1.8V) supply */
     "VDDL",  /* IF (1.2V) supply */
 };
 
 /* The supported raw formats. */
 static const u32 codes[] = {
     MEDIA_BUS_FMT_SBGGR8_1X8,
     MEDIA_BUS_FMT_SGBRG8_1X8,
     MEDIA_BUS_FMT_SGRBG8_1X8,
     MEDIA_BUS_FMT_SRGGB8_1X8,
     MEDIA_BUS_FMT_Y8_1X8,
 
     MEDIA_BUS_FMT_SBGGR10_1X10,
     MEDIA_BUS_FMT_SGBRG10_1X10,
     MEDIA_BUS_FMT_SGRBG10_1X10,
     MEDIA_BUS_FMT_SRGGB10_1X10,
     MEDIA_BUS_FMT_Y10_1X10,
 
     MEDIA_BUS_FMT_SBGGR12_1X12,
     MEDIA_BUS_FMT_SGBRG12_1X12,
     MEDIA_BUS_FMT_SGRBG12_1X12,
     MEDIA_BUS_FMT_SRGGB12_1X12,
     MEDIA_BUS_FMT_Y12_1X12,
 };
 
 #define ARDUCAM_NUM_SUPPLIES ARRAY_SIZE(pivariety_supply_name)
 
 #define ARDUCAM_XCLR_MIN_DELAY_US	10000
 #define ARDUCAM_XCLR_DELAY_RANGE_US	1000
 
 #define MAX_CTRLS 32
 
 #define ARDUCAM_LINK_FREQ		300000000

 static const s64 link_freq_menu_items[] = {
	ARDUCAM_LINK_FREQ
};

 struct pivariety {
     struct v4l2_subdev sd;
     struct media_pad pad;
 
     struct v4l2_mbus_config_mipi_csi2 bus;
     struct clk *xclk;
     u32 xclk_freq;
 
     struct gpio_desc *reset_gpio;
     struct regulator_bulk_data supplies[ARDUCAM_NUM_SUPPLIES];
 
     struct arducam_format *supported_formats;
     int num_supported_formats;
     int current_format_idx;
     int current_resolution_idx;
     int lanes;
     int bayer_order_volatile;
     bool wait_until_free;
     bool arducam_isp;
     bool x21_arducam_openhd;
     bool x21_fast_timing_active;
     struct v4l2_ctrl *x21_hdr;
     struct v4l2_ctrl *x21_iq[X21_ISP_COUNT];
     struct x21_ae_scene_state x21_ae_scene;
     struct delayed_work x21_timing_work;
 
     struct v4l2_ctrl_handler ctrl_handler;
     struct v4l2_ctrl *ctrls[MAX_CTRLS];
     /* V4L2 Controls */
     struct v4l2_ctrl *vflip;
     struct v4l2_ctrl *hflip;
 
     struct v4l2_rect crop;
     /*
      * Mutex for serialized access:
      * Protect sensor module set pad format and start/stop streaming safely.
      */
     struct mutex mutex;
 
     /* Streaming on/off */
     bool streaming;
 };
 
 #ifdef RADXA
static struct rkmodule_csi_dphy_param dcphy_param = {
	.vendor = PHY_VENDOR_SAMSUNG,
	.lp_vol_ref = 6,
	.lp_hys_sw = {3, 0, 0, 0},
	.lp_escclk_pol_sel = {1, 1, 1, 1},
	.skew_data_cal_clk = {0, 3, 3, 3},
	.clk_hs_term_sel = 2,
	.data_hs_term_sel = {2, 2, 2, 2},
	.reserved = {0},
};
#endif

 static inline struct pivariety *to_pivariety(struct v4l2_subdev *_sd)
 {
     return container_of(_sd, struct pivariety, sd);
 }
 
 /* Write registers up to 4 at a time */
 static int pivariety_write_reg(struct i2c_client *client, u16 reg, u32 val)
 {
     unsigned int len = sizeof(u32);
     u32 buf_i, val_i = 0;
     u8 buf[6];
     u8 *val_p;
     __be32 val_be;
 
     buf[0] = reg >> 8;
     buf[1] = reg & 0xff;
 
     val_be = cpu_to_be32(val);
     val_p = (u8 *)&val_be;
     buf_i = 2;
 
     while (val_i < 4)
         buf[buf_i++] = val_p[val_i++];
 
     if (i2c_master_send(client, buf, len + 2) != len + 2)
         return -EIO;
 
     return 0;
 }
 
 /* Read registers up to 4 at a time */
 static int pivariety_read_reg(struct i2c_client *client, u16 reg, u32 *val)
 {
     struct i2c_msg msgs[2];
     unsigned int len = sizeof(u32);
     u8 *data_be_p;
     __be32 data_be = 0;
     __be16 reg_addr_be = cpu_to_be16(reg);
     int ret;
 
     data_be_p = (u8 *)&data_be;
     /* Write register address */
     msgs[0].addr = client->addr;
     msgs[0].flags = 0;
     msgs[0].len = 2;
     msgs[0].buf = (u8 *)&reg_addr_be;
 
     /* Read data from register */
     msgs[1].addr = client->addr;
     msgs[1].flags = I2C_M_RD;
     msgs[1].len = len;
     msgs[1].buf = data_be_p;
 
     ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
     if (ret != ARRAY_SIZE(msgs))
         return -EIO;
 
     *val = be32_to_cpu(data_be);
 
     return 0;
 }
 
 static int
 pivariety_read(struct pivariety *pivariety, u16 addr, u32 *value)
 {
     struct v4l2_subdev *sd = &pivariety->sd;
     struct i2c_client *client = v4l2_get_subdevdata(sd);
     int ret, count = 0;
 
     while (count++ < I2C_READ_RETRY_COUNT) {
         ret = pivariety_read_reg(client, addr, value);
         if (!ret) {
             v4l2_dbg(2, debug, sd, "%s: 0x%02x 0x%04x\n",
                  __func__, addr, *value);
             return ret;
         }
     }
 
     v4l2_err(sd, "%s: Reading register 0x%02x failed\n",
          __func__, addr);
 
     return ret;
 }
 
 static int pivariety_write(struct pivariety *pivariety, u16 addr, u32 value)
 {
     struct v4l2_subdev *sd = &pivariety->sd;
     struct i2c_client *client = v4l2_get_subdevdata(sd);
     int ret, count = 0;
 
     while (count++ < I2C_WRITE_RETRY_COUNT) {
         ret = pivariety_write_reg(client, addr, value);
         if (!ret) {
             /* This MCU ACKs queued mode/control commands before its main
              * loop consumes them. Back-to-back index writes can overwrite
              * that slot, leaving a 1080p ISP behind a cached 720p format. */
             if (pivariety->x21_arducam_openhd && pivariety->arducam_isp && addr < 0x0500)
                 msleep(20);
             return ret;
         }
     }
 
     v4l2_err(sd, "%s: Write 0x%04x to register 0x%02x failed\n",
          __func__, value, addr);
 
     return ret;
 }
 
 static int wait_for_free_timeout(struct pivariety *pivariety, int interval,
                                  unsigned int timeout_ms)
 {
     u32 value;
     u32 count = 0;
 
     while (count++ < (timeout_ms / interval)) {
         int ret = pivariety_read(pivariety, SYSTEM_IDLE_REG, &value);
 
         if (ret)
             return ret;
         if (!value)
             return 0;
         msleep(interval);
     }
 
     v4l2_dbg(2, debug, &pivariety->sd, "%s: End wait, Count: %d.\n",
          __func__, count);
 
     return -ETIMEDOUT;
 }

 static int wait_for_free(struct pivariety *pivariety, int interval)
 {
     return wait_for_free_timeout(pivariety, interval, 1000);
 }

 static int pivariety_fw_read(void *context, unsigned short reg,
                              unsigned int *value)
 {
     struct pivariety *pivariety = context;
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);

     return pivariety_read_reg(client, reg, value);
 }

 static int pivariety_fw_write(void *context, unsigned short reg,
                               unsigned int value)
 {
     struct pivariety *pivariety = context;
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);

     return pivariety_write_reg(client, reg, value);
 }

 static int pivariety_fw_block(void *context, const unsigned char *data,
                               unsigned int size)
 {
     struct pivariety *pivariety = context;
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
     int ret = i2c_master_send(client, (const char *)data, size);

     return ret == size ? 0 : ret < 0 ? ret : -EIO;
 }

 static void pivariety_fw_delay(void *context, unsigned int milliseconds)
 {
     if (milliseconds <= 30)
         usleep_range(milliseconds * 1000, milliseconds * 1000 + 500);
     else
         msleep(milliseconds);
 }

 static void pivariety_fw_progress(void *context, const char *step)
 {
     struct pivariety *pivariety = context;
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);

     dev_info(&client->dev, "MCU firmware: %s\n", step);
 }

 static unsigned int pivariety_fw_crc(const unsigned char *data,
                                      unsigned int size)
 {
     return crc32_le(~0U, data, size) ^ ~0U;
 }

 static int pivariety_ensure_x21_firmware(struct pivariety *pivariety)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
     const struct x21_fw_ops ops = {
         .context = pivariety,
         .read = pivariety_fw_read,
         .write = pivariety_fw_write,
         .block = pivariety_fw_block,
         .delay = pivariety_fw_delay,
         .crc = pivariety_fw_crc,
         .progress = pivariety_fw_progress,
     };
     const struct i2c_adapter_quirks *quirks = client->adapter->quirks;
     int ret;

     if (!pivariety->x21_arducam_openhd || !x21_auto_firmware)
         return 0;
     if (!pivariety->arducam_isp || client->addr != 0x0c ||
         pivariety->bus.num_data_lanes != 4 ||
         !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ||
         (quirks && quirks->max_write_len && quirks->max_write_len < 66))
         return -ENODEV;

     dev_info(&client->dev, "Checking X21 ARDUCAM_OPENHD MCU firmware (manual AE, target 600)\n");
#if IS_ENABLED(CONFIG_ARDUCAM_OPENHD_EMBEDDED_FW)
     ret = x21_fw_ensure(&ops, x21_arducam_firmware, X21_FW_SIZE,
                         X21_FW_CRC32, 1);
#else
     /* Public builds can use a preprovisioned camera without a private payload. */
     ret = x21_fw_marker(&ops);
     ret = ret == 1 ? 0 : (ret ? ret : -ENODATA);
#endif
     if (ret < 0)
         dev_err(&client->dev,
                 "MCU firmware check/update failed (%d); camera not registered\n", ret);
     else
         dev_info(&client->dev, "X21 ARDUCAM_OPENHD firmware %s, revision 0x%04x\n",
                  ret ? "updated" : "already current", X21_FW_REV);
     return ret < 0 ? ret : 0;
 }

 static unsigned int pivariety_arducam_isp_max_fps(struct pivariety *pivariety)
 {
     struct arducam_resolution *resolution;

     resolution = &pivariety->supported_formats[pivariety->current_format_idx]
                      .resolution_set[pivariety->current_resolution_idx];
     if (pivariety->x21_hdr && pivariety->x21_hdr->val)
         return 30;
     return resolution->width == 1280 && resolution->height == 720 ? 120 : 60;
 }
 
 static int is_raw(int pixformat)
 {
     return pixformat >= 0x28 && pixformat <= 0x2D;
 }
 
 static u32 bayer_to_mbus_code(int data_type, int bayer_order)
 {
     const u32 depth8[] = {
         MEDIA_BUS_FMT_SBGGR8_1X8,
         MEDIA_BUS_FMT_SGBRG8_1X8,
         MEDIA_BUS_FMT_SGRBG8_1X8,
         MEDIA_BUS_FMT_SRGGB8_1X8,
         MEDIA_BUS_FMT_Y8_1X8,
     };
 
     const u32 depth10[] = {
         MEDIA_BUS_FMT_SBGGR10_1X10,
         MEDIA_BUS_FMT_SGBRG10_1X10,
         MEDIA_BUS_FMT_SGRBG10_1X10,
         MEDIA_BUS_FMT_SRGGB10_1X10,
         MEDIA_BUS_FMT_Y10_1X10,
     };
 
     const u32 depth12[] = {
         MEDIA_BUS_FMT_SBGGR12_1X12,
         MEDIA_BUS_FMT_SGBRG12_1X12,
         MEDIA_BUS_FMT_SGRBG12_1X12,
         MEDIA_BUS_FMT_SRGGB12_1X12,
         MEDIA_BUS_FMT_Y12_1X12,
     };
 
     if (bayer_order < 0 || bayer_order > 4)
         return 0;
 
     switch (data_type) {
     case IMAGE_DT_RAW8:
         return depth8[bayer_order];
     case IMAGE_DT_RAW10:
         return depth10[bayer_order];
     case IMAGE_DT_RAW12:
         return depth12[bayer_order];
     }
 
     return 0;
 }
 
 static u32 yuv422_to_mbus_code(int data_type, int order)
 {
     const u32 depth8[] = {
         MEDIA_BUS_FMT_YUYV8_1X16,
         MEDIA_BUS_FMT_YVYU8_1X16,
         MEDIA_BUS_FMT_UYVY8_1X16,
         MEDIA_BUS_FMT_VYUY8_1X16,
     };
 
     const u32 depth10[] = {
         MEDIA_BUS_FMT_YUYV10_1X20,
         MEDIA_BUS_FMT_YVYU10_1X20,
         MEDIA_BUS_FMT_UYVY10_1X20,
         MEDIA_BUS_FMT_VYUY10_1X20,
     };
 
     if (order < 0 || order > 3)
         return 0;
 
     switch (data_type) {
     case IMAGE_DT_YUV422_8:
         return depth8[order];
     case IMAGE_DT_YUV422_10:
         return depth10[order];
     }
 
     return 0;
 }
 
 static u32 data_type_to_mbus_code(int data_type, int bayer_order)
 {
     if (is_raw(data_type))
         return bayer_to_mbus_code(data_type, bayer_order);
 
     switch (data_type) {
     case IMAGE_DT_YUV422_8:
     case IMAGE_DT_YUV422_10:
         return yuv422_to_mbus_code(data_type, bayer_order);
     case IMAGE_DT_RGB565:
         return MEDIA_BUS_FMT_RGB565_2X8_LE;
     case IMAGE_DT_RGB888:
         return MEDIA_BUS_FMT_RGB888_1X24;
     }
 
     return 0;
 }
 
 /* Get bayer order based on flip setting. */
 static u32 pivariety_get_format_code(struct pivariety *pivariety,
                      struct arducam_format *format)
 {
     unsigned int order, origin_order;
 
     lockdep_assert_held(&pivariety->mutex);
 
     /*
      * Only the bayer format needs to transform the format.
      */
     if (!is_raw(format->data_type) ||
         !pivariety->bayer_order_volatile ||
         format->bayer_order == BAYER_ORDER_GRAY)
         return data_type_to_mbus_code(format->data_type,
                           format->bayer_order);
 
     order = format->bayer_order;
 
     origin_order = order;
 
     order = (pivariety->hflip && pivariety->hflip->val ? order ^ 1 : order);
     order = (pivariety->vflip && pivariety->vflip->val ? order ^ 2 : order);
 
     v4l2_dbg(1, debug, &pivariety->sd, "%s: before: %d, after: %d.\n",
          __func__, origin_order, order);
 
     return data_type_to_mbus_code(format->data_type, order);
 }
 
 /* Power/clock management functions */
 static int pivariety_power_on(struct device *dev)
 {
     struct i2c_client *client = to_i2c_client(dev);
     struct v4l2_subdev *sd = i2c_get_clientdata(client);
     struct pivariety *pivariety = to_pivariety(sd);
     int ret;
 
     ret = regulator_bulk_enable(ARDUCAM_NUM_SUPPLIES,
                     pivariety->supplies);
     if (ret) {
         dev_err(dev, "%s: failed to enable regulators\n",
             __func__);
         return ret;
     }
 
     ret = clk_prepare_enable(pivariety->xclk);
     if (ret) {
         dev_err(dev, "%s: failed to enable clock\n",
             __func__);
         goto reg_off;
     }
 
     gpiod_set_value_cansleep(pivariety->reset_gpio, 1);
     usleep_range(ARDUCAM_XCLR_MIN_DELAY_US,
              ARDUCAM_XCLR_MIN_DELAY_US + ARDUCAM_XCLR_DELAY_RANGE_US);
 
     return 0;
 
 reg_off:
     regulator_bulk_disable(ARDUCAM_NUM_SUPPLIES, pivariety->supplies);
 
     return ret;
 }
 
 static int pivariety_power_off(struct device *dev)
 {
     struct i2c_client *client = to_i2c_client(dev);
     struct v4l2_subdev *sd = i2c_get_clientdata(client);
     struct pivariety *pivariety = to_pivariety(sd);
 
     gpiod_set_value_cansleep(pivariety->reset_gpio, 0);
     regulator_bulk_disable(ARDUCAM_NUM_SUPPLIES, pivariety->supplies);
     clk_disable_unprepare(pivariety->xclk);
 
     return 0;
 }
 
 static int pivariety_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
 {
     struct pivariety *pivariety = to_pivariety(sd);
     struct v4l2_mbus_framefmt *try_fmt =
         v4l2_subdev_get_try_format(sd, fh->state, 0);
     struct arducam_format *def_fmt = &pivariety->supported_formats[0];
 
     /* Initialize try_fmt */
     try_fmt->width = def_fmt->resolution_set->width;
     try_fmt->height = def_fmt->resolution_set->height;
     try_fmt->code = def_fmt->mbus_code;
     try_fmt->field = V4L2_FIELD_NONE;
 
     return 0;
 }
 
 static int pivariety_isp_read(void *context, unsigned short reg,
                               unsigned int *value)
 {
     struct pivariety *pivariety = context;
     u32 response;
     unsigned int attempt;
     int ret = wait_for_free(pivariety, 5);

     if (ret)
         return ret;
     ret = pivariety_write(pivariety, 0x0501, (u32)reg << 16);
     if (ret)
         return ret;
     /* Revision 0102 includes queued requests in SYSTEM_IDLE_REG. */
     usleep_range(5000, 6000);
     ret = wait_for_free(pivariety, 5);
     if (ret)
         return ret;
     for (attempt = 0; attempt < 10; attempt++) {
         ret = pivariety_read(pivariety, 0x0501, &response);
         if (ret)
             return ret;
         if ((response >> 16) == reg) {
             *value = response & 0xffff;
             return 0;
         }
         usleep_range(5000, 6000);
     }
     return -EPROTO;
 }

 static int pivariety_isp_write(void *context, unsigned short reg,
                                unsigned int value)
 {
     struct pivariety *pivariety = context;
     int ret = wait_for_free(pivariety, 5);

     if (ret)
         return ret;
     ret = pivariety_write(pivariety, 0x0502, ((u32)reg << 16) | value);
     if (ret)
         return ret;
     usleep_range(5000, 6000);
     return wait_for_free(pivariety, 5);
 }

 /* Standard V4L2 controls belong to the ISP subdevice. No private IQ ABI. */
 static const u32 x21_iq_ids[X21_ISP_COUNT] = {
     V4L2_CID_EXPOSURE_ABSOLUTE, V4L2_CID_GAIN,
     V4L2_CID_AUTO_WHITE_BALANCE, V4L2_CID_WHITE_BALANCE_TEMPERATURE,
     V4L2_CID_BRIGHTNESS, V4L2_CID_CONTRAST, V4L2_CID_SATURATION,
     V4L2_CID_GAMMA, V4L2_CID_EXPOSURE_AUTO,
     V4L2_CID_SCENE_MODE, V4L2_CID_AUTO_EXPOSURE_BIAS,
 };
 static int pivariety_x21_iq_index(u32 id)
 {
     unsigned int i;
     for (i = 0; i < ARRAY_SIZE(x21_iq_ids); i++)
         if (x21_iq_ids[i] == id) return i;
     return -1;
 }
 static struct v4l2_ctrl *get_control(struct pivariety *pivariety, u32 id);
 static int pivariety_x21_auto_fps(struct pivariety *pivariety, unsigned int fps)
 {
     unsigned int flicker;
     int ret;
     if (fps > 60) {
         ret = pivariety_isp_read(pivariety, 0x5440, &flicker);
         if (ret) return ret;
         /* Flicker quantization/FRC must not lower a requested high rate. */
         ret = pivariety_isp_write(pivariety, 0x5440, flicker & ~7U);
         if (ret) return ret;
     }
     return pivariety_isp_write(pivariety, 0x2020, fps << 8);
 }
 static int pivariety_x21_apply_iq(struct pivariety *pivariety,
                                  unsigned int index, int value)
 {
     const struct x21_timing_ops ops = {
         .context = pivariety, .read = pivariety_isp_read,
         .write = pivariety_isp_write, .delay = pivariety_fw_delay,
     };
     if (!pivariety->streaming) return 0;
     if (index == X21_AE_SCENE) {
         struct v4l2_ctrl *rate = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);
         int ret;
         if (pivariety->x21_iq[X21_AE_AUTO]->val != V4L2_EXPOSURE_AUTO) return 0;
         ret = __v4l2_ctrl_s_ctrl(pivariety->x21_iq[X21_AE_BIAS], 8);
         if (ret) return ret;
         return rate ? x21_ae_scene_apply(&ops, &pivariety->x21_ae_scene,
                                         value, rate->val) : -EINVAL;
     }
     /* V4L2 exposure_absolute is in 100 us units. ISP takes us. */
     if (index == X21_EXPOSURE_US) value *= 100;
     if ((index == X21_EXPOSURE_US || index == X21_GAIN) &&
         pivariety->x21_iq[X21_AE_AUTO]->val == V4L2_EXPOSURE_AUTO)
         return 0;
     if (index == X21_EXPOSURE_US)
         return x21_sensor_manual_exposure(&ops, value,
             pivariety->x21_hdr && pivariety->x21_hdr->val &&
             pivariety->x21_fast_timing_active);
     if (index == X21_GAIN)
         return x21_sensor_manual_gain(&ops, value);
     if (index == X21_AE_AUTO) {
         int ret = x21_isp_apply(&ops, index, value == V4L2_EXPOSURE_AUTO);
         if (ret) return ret;
         if (value == V4L2_EXPOSURE_AUTO) {
             struct v4l2_ctrl *rate = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);
             /* Native AE manages sensor blanking at the requested rate. Do
              * not let the manual delayed table turn it back off. */
             cancel_delayed_work(&pivariety->x21_timing_work);
             pivariety->x21_fast_timing_active = false;
             ret = rate ? pivariety_x21_auto_fps(pivariety, rate->val) : -EINVAL;
             if (ret) return ret;
             return pivariety_x21_apply_iq(pivariety, X21_AE_SCENE,
                         pivariety->x21_iq[X21_AE_SCENE]->val);
         }
         if (value == V4L2_EXPOSURE_MANUAL) {
             ret = pivariety_x21_apply_iq(pivariety, X21_EXPOSURE_US,
                         pivariety->x21_iq[X21_EXPOSURE_US]->val);
             if (ret) return ret;
             ret = pivariety_x21_apply_iq(pivariety, X21_GAIN,
                         pivariety->x21_iq[X21_GAIN]->val);
             if (ret) return ret;
             if (x21_linear_720p120 && !pivariety->x21_fast_timing_active) {
                 struct v4l2_ctrl *rate = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);
                 if (rate && rate->val == 120)
                     mod_delayed_work(system_wq, &pivariety->x21_timing_work, 0);
             }
         }
         return 0;
     }
     /* Kelvin is retained while AWB is on, and applied when manual is selected. */
     if (index == X21_WB_K && pivariety->x21_iq[X21_WB_AUTO]->val) return 0;
     if (index == X21_WB_AUTO && !value) {
         int ret = x21_isp_apply(&ops, X21_WB_K, pivariety->x21_iq[X21_WB_K]->val);
         if (ret) return ret;
     }
     return x21_isp_apply(&ops, index, value);
 }
 static int pivariety_x21_restore_iq(struct pivariety *pivariety)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
     unsigned int i, attempt;
     int ret;
     if (!pivariety->x21_iq[0]) return 0;
     for (i = 0; i < X21_ISP_COUNT; i++) {
         for (attempt = 0; attempt < 10; attempt++) {
             ret = pivariety_x21_apply_iq(pivariety, i, pivariety->x21_iq[i]->val);
             if (ret != -EIO && ret != -EAGAIN) break;
             /* Sensor writes latch at frame boundaries. A just-completed
              * readout transition can still expose the previous value, and
              * MCU geometry readiness can precede sensor PLL readiness.
              * Retry only these transient states; never an unknown PLL. */
             if (attempt == 9) break;
             usleep_range(100000, 105000);
         }
         if (ret) {
             dev_err(&client->dev, "ISP control 0x%x restore failed: %d\n",
                     x21_iq_ids[i], ret);
             return ret;
         }
     }
     return 0;
 }

 static int pivariety_x21_720p120(struct pivariety *pivariety)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
     const struct x21_timing_ops timing = {
         .context = pivariety,
         .read = pivariety_isp_read,
         .write = pivariety_isp_write,
         .delay = pivariety_fw_delay,
     };
     const struct x21_fw_ops firmware = {
         .context = pivariety,
         .read = pivariety_fw_read,
         .write = pivariety_fw_write,
         .delay = pivariety_fw_delay,
     };
     int ret = x21_fw_marker(&firmware);

     if (ret != 1)
         return ret < 0 ? ret : -EOPNOTSUPP;
     /* Start from a settled preview, then acquire linear readout before the
      * final blanking adjustment. All commands hold the V4L2 driver mutex. */
     ret = pivariety_isp_write(pivariety, 0x2020, 60 << 8);
     if (ret)
         return ret;
     msleep(100);
     ret = x21_timing_720p120(&timing);
     if (ret) {
         dev_err(&client->dev, "720p120 sensor timing failed: %d\n", ret);
         return ret;
     }
     /* The vendor AE immediate request precedes our custom timing. Give its
      * asynchronous completion time to settle before restoring manual IQ. */
     msleep(1000);
     pivariety->x21_fast_timing_active = true;
     ret = pivariety_x21_restore_iq(pivariety);
     if (ret) {
         pivariety->x21_fast_timing_active = false;
         return ret;
     }
     dev_info(&client->dev,
              "720p120 linear timing: frame=1164 line=1180 exposure=4000us\n");
     return 0;
 }

 static int pivariety_x21_hdr30(struct pivariety *pivariety)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
     const struct x21_timing_ops timing = {
         .context = pivariety, .read = pivariety_isp_read,
         .write = pivariety_isp_write, .delay = pivariety_fw_delay,
     };
     const struct x21_fw_ops firmware = {
         .context = pivariety, .read = pivariety_fw_read,
         .write = pivariety_fw_write, .delay = pivariety_fw_delay,
     };
     int ret = x21_fw_marker(&firmware);

     if (ret != 1)
         return ret < 0 ? ret : -EOPNOTSUPP;
     ret = x21_timing_hdr30(&timing);
     if (ret)
         dev_err(&client->dev, "3-exposure HDR timing failed: %d\n", ret);
     else {
         pivariety->x21_fast_timing_active = true;
         ret = pivariety_x21_restore_iq(pivariety);
         if (ret) {
             pivariety->x21_fast_timing_active = false;
             return ret;
         }
         dev_info(&client->dev,
                  "3-exposure HDR: full-field binned input, RAW20 to RAW12, 30 fps\n");
     }
     return ret;
 }

 static int pivariety_x21_restore_geometry(struct pivariety *pivariety)
 {
     struct arducam_resolution *resolution =
         &pivariety->supported_formats[pivariety->current_format_idx]
              .resolution_set[pivariety->current_resolution_idx];
     unsigned int width, height, attempt;
     int ret;

     if (!pivariety->x21_arducam_openhd || !pivariety->arducam_isp)
         return 0;
     /* MCU completion precedes the ISP's asynchronous mode application.
      * Wait for the actual selected geometry instead of changing a booting ISP. */
     for (attempt = 0; attempt < 40; attempt++) {
         ret = pivariety_isp_read(pivariety, 0x2000, &width);
         if (ret) return ret;
         ret = pivariety_isp_read(pivariety, 0x2002, &height);
         if (ret) return ret;
         if (width == resolution->width && height == resolution->height)
             return 0;
         msleep(250);
     }
     v4l2_err(&pivariety->sd, "ISP remained %ux%u, expected %ux%u\n",
               width, height, resolution->width, resolution->height);
     return -ETIMEDOUT;
 }
 static void pivariety_x21_timing_work(struct work_struct *work)
 {
     struct pivariety *pivariety = container_of(to_delayed_work(work),
                                         struct pivariety, x21_timing_work);
     struct v4l2_ctrl *rate;
     struct arducam_resolution *resolution;
     int ret;

     mutex_lock(&pivariety->mutex);
     /* The handler shares this mutex; v4l2_ctrl_find() would relock it. */
     rate = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);
     resolution = &pivariety->supported_formats[pivariety->current_format_idx]
                      .resolution_set[pivariety->current_resolution_idx];
     if (pivariety->streaming && pivariety->x21_hdr &&
         pivariety->x21_hdr->val && rate && rate->val == 30 &&
         !pivariety->x21_fast_timing_active) {
         ret = pivariety_x21_hdr30(pivariety);
         if (ret)
             v4l2_err(&pivariety->sd, "Deferred HDR setup failed: %d\n", ret);
     } else if (x21_linear_720p120 && pivariety->streaming && rate && rate->val == 120 &&
         pivariety->x21_arducam_openhd && pivariety->lanes == 4 &&
         pivariety->x21_iq[X21_AE_AUTO]->val != V4L2_EXPOSURE_AUTO &&
         resolution->width == 1280 && resolution->height == 720 &&
         !pivariety->x21_fast_timing_active) {
         ret = pivariety_x21_720p120(pivariety);
         if (ret)
             v4l2_err(&pivariety->sd, "720p120 deferred timing failed: %d\n", ret);
     }
     mutex_unlock(&pivariety->mutex);
 }

 static int pivariety_arducam_isp_set_fps(struct pivariety *pivariety, u32 fps)
 {
     struct arducam_resolution *resolution =
         &pivariety->supported_formats[pivariety->current_format_idx]
              .resolution_set[pivariety->current_resolution_idx];

     if (pivariety->x21_hdr && pivariety->x21_hdr->val) {
         if (fps != 30) return -ERANGE;
         if (pivariety->streaming && !pivariety->x21_fast_timing_active)
             mod_delayed_work(system_wq, &pivariety->x21_timing_work,
                              msecs_to_jiffies(15000));
         return pivariety_isp_write(pivariety, 0x2020, 30 << 8);
     }
     if (pivariety->x21_arducam_openhd &&
         pivariety->x21_iq[X21_AE_AUTO]->val == V4L2_EXPOSURE_AUTO) {
         cancel_delayed_work(&pivariety->x21_timing_work);
         return pivariety_x21_auto_fps(pivariety, fps);
     }
     if (x21_linear_720p120 && pivariety->x21_arducam_openhd &&
         pivariety->x21_iq[X21_AE_AUTO]->val != V4L2_EXPOSURE_AUTO &&
         pivariety->lanes == 4 && fps == 120 &&
         resolution->width == 1280 && resolution->height == 720 &&
         !pivariety->x21_fast_timing_active) {
         /* Let capture return its initial buffers before stop/reacquire. */
         if (pivariety->streaming)
             mod_delayed_work(system_wq, &pivariety->x21_timing_work,
                              msecs_to_jiffies(15000));
         return pivariety_isp_write(pivariety, 0x2020, 60 << 8);
     }
     cancel_delayed_work(&pivariety->x21_timing_work);

     /* The supplied ISP MCU firmware's frame-rate setter is empty.
      * Use its existing volatile 16-bit ISP-register bridge instead.
      * PREVIEW_MAX_FPS is Q8.8; this does not change PLLs or sensor timing. */
     return pivariety_isp_write(pivariety, 0x2020, fps << 8);
 }

 static int pivariety_s_ctrl(struct v4l2_ctrl *ctrl)
 {
     int ret, i;
     struct pivariety *pivariety =
         container_of(ctrl->handler, struct pivariety,
                  ctrl_handler);
     struct arducam_format *supported_fmts = pivariety->supported_formats;
     int num_supported_formats = pivariety->num_supported_formats;

     i = pivariety_x21_iq_index(ctrl->id);
     if (pivariety->x21_arducam_openhd && pivariety->arducam_isp &&
         pivariety->lanes == 4 && i >= 0) {
         /* A queued Fly correction must not affect a newer Manual/Normal
          * selection. Neutral is always allowed for cleanup. */
         if (i == X21_AE_BIAS && ctrl->val != 8 &&
             (pivariety->x21_iq[X21_AE_AUTO]->val != V4L2_EXPOSURE_AUTO ||
              pivariety->x21_iq[X21_AE_SCENE]->val != V4L2_SCENE_MODE_BACKLIGHT))
             return -EBUSY;
         /* Three-exposure HDR still uses an explicit sensor table. Ordinary
          * 720p120 can instead use native AE with the preview FPS cap. */
         if (i == X21_AE_AUTO && ctrl->val == V4L2_EXPOSURE_AUTO) {
             if (pivariety->x21_hdr && pivariety->x21_hdr->val)
                 return -EOPNOTSUPP;
         }
         return pivariety_x21_apply_iq(pivariety, i, ctrl->val);
     }

     if (pivariety->x21_arducam_openhd && ctrl->id == V4L2_CID_WIDE_DYNAMIC_RANGE) {
         struct v4l2_ctrl *rate;
         if (pivariety->streaming) return -EBUSY;
         if (ctrl->val && pivariety->x21_iq[X21_AE_AUTO] &&
             pivariety->x21_iq[X21_AE_AUTO]->val == V4L2_EXPOSURE_AUTO)
             return -EOPNOTSUPP;
         rate = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);
         if (!rate) return -EINVAL;
         return __v4l2_ctrl_modify_range(rate, 30,
             pivariety_arducam_isp_max_fps(pivariety), 1, 30);
     }

     /* Do not call the empty vendor FPS setter. The ISP is reloaded at
      * stream-on; apply cached FPS there, or directly while streaming. */
     if (pivariety->arducam_isp && ctrl->id == V4L2_CID_ARDUCAM_FRAME_RATE) {
         if (pivariety->x21_arducam_openhd &&
             ctrl->val > pivariety_arducam_isp_max_fps(pivariety))
             return -ERANGE;
         return pivariety->streaming ?
             pivariety_arducam_isp_set_fps(pivariety, ctrl->val) : 0;
     }
 
     v4l2_dbg(3, debug, &pivariety->sd, "%s: cid = (0x%X), value = (%d).\n",
          __func__, ctrl->id, ctrl->val);
 
     ret = wait_for_free(pivariety, 1);
     if (ret)
         return ret;
     ret = pivariety_write(pivariety, CTRL_ID_REG, ctrl->id);
     if (ret)
         return ret;
     ret = wait_for_free(pivariety, 1);
     if (ret)
         return ret;
     ret = pivariety_write(pivariety, CTRL_VALUE_REG, ctrl->val);
     if (ret)
         return ret;
     /* The ISP must finish each command before the next control/stream-on. */
     ret = wait_for_free(pivariety, 1);
     if (ret)
         return ret;

     /* When flip is set, modify all bayer formats */
     if (ctrl->id == V4L2_CID_VFLIP || ctrl->id == V4L2_CID_HFLIP) {
         for (i = 0; i < num_supported_formats; i++) {
             supported_fmts[i].mbus_code =
                 pivariety_get_format_code(pivariety,
                               &supported_fmts[i]);
         }
     }
 
     return 0;
 }
 
 static const struct v4l2_ctrl_ops pivariety_ctrl_ops = {
     .s_ctrl = pivariety_s_ctrl,
 };
 
 static int pivariety_enum_mbus_code(struct v4l2_subdev *sd,
                     struct v4l2_subdev_state *sd_state,
                     struct v4l2_subdev_mbus_code_enum *code)
 {
     struct pivariety *pivariety = to_pivariety(sd);
     struct arducam_format *supported_formats = pivariety->supported_formats;
     int num_supported_formats = pivariety->num_supported_formats;
 
     v4l2_dbg(1, debug, sd, "%s: index = (%d)\n", __func__, code->index);
 
     if (code->index >= num_supported_formats)
         return -EINVAL;
 
     code->code = supported_formats[code->index].mbus_code;
 
     return 0;
 }
 
 static int pivariety_enum_framesizes(struct v4l2_subdev *sd,
                      struct v4l2_subdev_state *sd_state,
                      struct v4l2_subdev_frame_size_enum *fse)
 {
     int i;
     struct pivariety *pivariety = to_pivariety(sd);
     struct arducam_format *supported_formats = pivariety->supported_formats;
     int num_supported_formats = pivariety->num_supported_formats;
     struct arducam_format *format;
     struct arducam_resolution *resolution;
 
     v4l2_dbg(1, debug, sd, "%s: code = (0x%X), index = (%d)\n",
          __func__, fse->code, fse->index);
 
     for (i = 0; i < num_supported_formats; i++) {
         format = &supported_formats[i];
         if (fse->code == format->mbus_code) {
             if (fse->index >= format->num_resolution_set)
                 return -EINVAL;
 
             resolution = &format->resolution_set[fse->index];
             fse->min_width = resolution->width;
             fse->max_width = resolution->width;
             fse->min_height = resolution->height;
             fse->max_height = resolution->height;
 
             return 0;
         }
     }
 
     return -EINVAL;
 }
 
 static int pivariety_enum_frame_interval(struct v4l2_subdev *sd,
                      struct v4l2_subdev_state *state,
                      struct v4l2_subdev_frame_interval_enum *fie)
 {
     static const unsigned int fps[] = {30, 60, 120};
     struct pivariety *pivariety = to_pivariety(sd);
     unsigned int count;

     if (!pivariety->x21_arducam_openhd || fie->pad ||
         fie->code != MEDIA_BUS_FMT_UYVY8_1X16)
         return -EINVAL;
     if (fie->width == 1280 && fie->height == 720)
         count = 3;
     else if (fie->width == 1920 && fie->height == 1080)
         count = 2;
     else
         return -EINVAL;
     if (pivariety->x21_hdr && pivariety->x21_hdr->val)
         count = 1;
     if (fie->index >= count)
         return -EINVAL;
     /* Interval enumeration describes selectable linear output modes. */
     fie->interval.numerator = 1;
     fie->interval.denominator = fps[fie->index];
     return 0;
 }

 static int pivariety_get_fmt(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *sd_state,
                  struct v4l2_subdev_format *format)
 {
     struct pivariety *pivariety = to_pivariety(sd);
     struct arducam_format *current_format;
     struct v4l2_mbus_framefmt *fmt = &format->format;
     int cur_res_idx;
 
     if (format->pad != 0)
         return -EINVAL;
 
     mutex_lock(&pivariety->mutex);

     if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
         format->format = *v4l2_subdev_get_try_format(sd, sd_state, 0);
         mutex_unlock(&pivariety->mutex);
         return 0;
     }

     current_format =
         &pivariety->supported_formats[pivariety->current_format_idx];
     cur_res_idx = pivariety->current_resolution_idx;
     format->format.width =
         current_format->resolution_set[cur_res_idx].width;
     format->format.height =
         current_format->resolution_set[cur_res_idx].height;
     format->format.code = current_format->mbus_code;
     format->format.field = V4L2_FIELD_NONE;
     fmt->colorspace = V4L2_COLORSPACE_RAW;
     fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
     fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
                               fmt->colorspace,
                               fmt->ycbcr_enc);
     fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
 
     v4l2_dbg(1, debug, sd, "%s: width: (%d) height: (%d) code: (0x%X)\n",
          __func__, format->format.width, format->format.height,
          format->format.code);
 
     mutex_unlock(&pivariety->mutex);
     return 0;
 }
 
 static int pivariety_get_fmt_idx_by_code(struct pivariety *pivariety,
                      u32 mbus_code)
 {
     int i;
     u32 data_type;
     struct arducam_format *formats = pivariety->supported_formats;
 
     for (i = 0; i < pivariety->num_supported_formats; i++) {
         if (formats[i].mbus_code == mbus_code)
             return i;
     }
 
     /*
      * If the specified format is not found in the list of supported
      * formats, try to find a format of the same data type.
      */
     for (i = 0; i < ARRAY_SIZE(codes); i++)
         if (codes[i] == mbus_code)
             break;
 
     if (i >= ARRAY_SIZE(codes))
         return -EINVAL;
 
     data_type = i / 5 + IMAGE_DT_RAW8;
 
     for (i = 0; i < pivariety->num_supported_formats; i++) {
         if (formats[i].data_type == data_type)
             return i;
     }
 
     return -EINVAL;
 }
 
 static struct v4l2_ctrl *get_control(struct pivariety *pivariety,
                      u32 id)
 {
     int index = 0;
 
     while (index < MAX_CTRLS && pivariety->ctrls[index]) {
         if (pivariety->ctrls[index]->id == id)
             return pivariety->ctrls[index];
         index++;
     }
 
     return NULL;
 }
 
 static int update_control(struct pivariety *pivariety, u32 id)
 {
     struct v4l2_subdev *sd = &pivariety->sd;
     struct v4l2_ctrl *ctrl;
     u32 min, max, step, def, id2;
     int ret = 0;
 
     pivariety_write(pivariety, CTRL_ID_REG, id);
     pivariety_read(pivariety, CTRL_ID_REG, &id2);
 
     v4l2_dbg(1, debug, sd, "%s: Write ID: 0x%08X Read ID: 0x%08X\n",
          __func__, id, id2);
 
     pivariety_write(pivariety, CTRL_VALUE_REG, 0);
     wait_for_free(pivariety, 1);
 
     ret += pivariety_read(pivariety, CTRL_MAX_REG, &max);
     ret += pivariety_read(pivariety, CTRL_MIN_REG, &min);
     ret += pivariety_read(pivariety, CTRL_DEF_REG, &def);
     ret += pivariety_read(pivariety, CTRL_STEP_REG, &step);
 
     if (ret < 0)
         goto err;
 
     if (id == NO_DATA_AVAILABLE || max == NO_DATA_AVAILABLE ||
         min == NO_DATA_AVAILABLE || def == NO_DATA_AVAILABLE ||
         step == NO_DATA_AVAILABLE)
         goto err;
 
     v4l2_dbg(1, debug, sd, "%s: min: %d, max: %d, step: %d, def: %d\n",
          __func__, min, max, step, def);
 
     ctrl = get_control(pivariety, id);
     if (!ctrl)
         return -EINVAL;
     if (pivariety->x21_arducam_openhd && id == V4L2_CID_ARDUCAM_FRAME_RATE) {
         max = min(max, pivariety_arducam_isp_max_fps(pivariety));
         def = min(def, max);
     }
     return __v4l2_ctrl_modify_range(ctrl, min, max, step, def);
 
 err:
     return -EINVAL;
 }
 
 static int update_controls(struct pivariety *pivariety)
 {
     int ret = 0;
     int index = 0;
 
     wait_for_free(pivariety, 5);
 
     while (index < MAX_CTRLS && pivariety->ctrls[index]) {
         ret += update_control(pivariety, pivariety->ctrls[index]->id);
         index++;
     }
 
     return ret;
 }
 
 static int pivariety_set_fmt(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *sd_state,
                  struct v4l2_subdev_format *format)
 {
     int i, j, ret = 0;
     struct pivariety *pivariety = to_pivariety(sd);
     struct arducam_format *supported_formats = pivariety->supported_formats;
 
     if (format->pad != 0)
         return -EINVAL;
 
     mutex_lock(&pivariety->mutex);

     if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && pivariety->streaming) {
         mutex_unlock(&pivariety->mutex);
         return -EBUSY;
     }
 
     format->format.colorspace = V4L2_COLORSPACE_RAW;
     format->format.field = V4L2_FIELD_NONE;
 
     v4l2_dbg(1, debug, sd, "%s: code: 0x%X, width: %d, height: %d\n",
          __func__, format->format.code, format->format.width,
          format->format.height);
 
     i = pivariety_get_fmt_idx_by_code(pivariety, format->format.code);
     if (i < 0)
         i = 0;
 
     format->format.code = supported_formats[i].mbus_code;
      
     for (j = 0; j < supported_formats[i].num_resolution_set; j++) {
         if (supported_formats[i].resolution_set[j].width ==
                         format->format.width &&
             supported_formats[i].resolution_set[j].height ==
                         format->format.height) {
             v4l2_dbg(1, debug, sd,
                  "%s: format match.\n", __func__);
             v4l2_dbg(1, debug, sd,
                  "%s: set format to device: %d %d.\n",
                  __func__, supported_formats[i].index, j);

             if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
                 *v4l2_subdev_get_try_format(sd, sd_state, 0) = format->format;
                 goto unlock;
             }
 
             ret = pivariety_write(pivariety, PIXFORMAT_INDEX_REG,
                     supported_formats[i].index);
             if (ret)
                 goto unlock;
             ret = pivariety_write(pivariety, RESOLUTION_INDEX_REG, j);
             if (ret)
                 goto unlock;
 
             pivariety->current_format_idx = i;
             pivariety->current_resolution_idx = j;
 
             ret = update_controls(pivariety);
 
             goto unlock;
         }
     }
 
     format->format.width = supported_formats[i].resolution_set[0].width;
     format->format.height = supported_formats[i].resolution_set[0].height;

     if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
         *v4l2_subdev_get_try_format(sd, sd_state, 0) = format->format;
         goto unlock;
     }
 
     ret = pivariety_write(pivariety, PIXFORMAT_INDEX_REG,
             supported_formats[i].index);
     if (ret)
         goto unlock;
     ret = pivariety_write(pivariety, RESOLUTION_INDEX_REG, 0);
     if (ret)
         goto unlock;
 
     pivariety->current_format_idx = i;
     pivariety->current_resolution_idx = 0;
     ret = update_controls(pivariety);
 
 unlock:
 
     mutex_unlock(&pivariety->mutex);
 
     return ret;
 }
 
 /* Start streaming */
 static int pivariety_start_streaming(struct pivariety *pivariety)
 {
     int ret;
 printk("pivariety_start_streaming!!!!!!\n");
     /* set stream on register */
     ret = pivariety_write(pivariety, MODE_SELECT_REG,
                   ARDUCAM_MODE_STREAMING);
     if (!ret && pivariety->arducam_isp) {
         /* Command ACK precedes the MCU's ISP reload (about six seconds).
          * Give its command queue time to dispatch before checking busy. */
         msleep(20);
         ret = wait_for_free_timeout(pivariety, 5, 10000);
     }
     return ret;
 }
 
 static int pivariety_get_selection(struct v4l2_subdev *sd,
                    struct v4l2_subdev_state *sd_state,
                    struct v4l2_subdev_selection *sel)
 {
     struct pivariety *pivariety = to_pivariety(sd);
     struct arducam_resolution *resolution;

     if (sel->pad != 0)
         return -EINVAL;
     switch (sel->target) {
     case V4L2_SEL_TGT_CROP:
     case V4L2_SEL_TGT_NATIVE_SIZE:
     case V4L2_SEL_TGT_CROP_DEFAULT:
     case V4L2_SEL_TGT_CROP_BOUNDS:
         break;
     default:
         return -EINVAL;
     }

     /* No subdevice crop setter is exposed. The selected mode is the full
      * output rectangle. ISP IPC geometry still describes the previous mode
      * until stream-on; using it prevents CIF from enlarging 720p to 1080p. */
     mutex_lock(&pivariety->mutex);
     sel->r.left = 0;
     sel->r.top = 0;
     if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
         struct v4l2_mbus_framefmt *format =
             v4l2_subdev_get_try_format(sd, sd_state, sel->pad);
         sel->r.width = format->width;
         sel->r.height = format->height;
     } else {
         resolution = &pivariety->supported_formats[pivariety->current_format_idx]
             .resolution_set[pivariety->current_resolution_idx];
         sel->r.width = resolution->width;
         sel->r.height = resolution->height;
     }
     mutex_unlock(&pivariety->mutex);
     return 0;
 }
 
 /* Stop streaming */
 static int pivariety_stop_streaming(struct pivariety *pivariety)
 {
     int ret;
 printk("pivariety_stop_streaming!!!!!!!!\n");
     /* set stream off register */
     ret = pivariety_write(pivariety, MODE_SELECT_REG, ARDUCAM_MODE_STANDBY);
     if (ret)
         v4l2_err(&pivariety->sd, "%s failed to set stream\n", __func__);
 
     /*
      * Return success even if it was an error, as there is nothing the
      * caller can do about it.
      */
     return 0;
 }
 
 static int pivariety_set_stream(struct v4l2_subdev *sd, int enable)
 {
     struct pivariety *pivariety = to_pivariety(sd);
     struct i2c_client *client = v4l2_get_subdevdata(sd);
     struct v4l2_ctrl *frame_rate = NULL;
     int ret = 0;
 
     mutex_lock(&pivariety->mutex);
     if (pivariety->streaming == enable) {
         mutex_unlock(&pivariety->mutex);
         return 0;
     }
 
     if (enable) {
         pivariety->x21_fast_timing_active = false;
         pivariety->x21_ae_scene.valid = 0;
         ret = pm_runtime_get_sync(&client->dev);
         if (ret < 0) {
             pm_runtime_put_noidle(&client->dev);
             goto err_unlock;
         }
 
         /*
          * Pivariety has no guaranteed default mode. Runtime resume may
          * restart its ISP, so program the cached selection while powered,
          * then apply controls before enabling the UYVY stream.
          */
         ret = pivariety_write(pivariety, PIXFORMAT_INDEX_REG,
             pivariety->supported_formats[pivariety->current_format_idx].index);
         if (ret)
             goto err_rpm_put;
         ret = pivariety_write(pivariety, RESOLUTION_INDEX_REG,
                              pivariety->current_resolution_idx);
         if (ret)
             goto err_rpm_put;
         ret = wait_for_free(pivariety, 5);
         if (ret)
             goto err_rpm_put;
         ret = __v4l2_ctrl_handler_setup(pivariety->sd.ctrl_handler);
         if (ret)
             goto err_rpm_put;
         ret = wait_for_free(pivariety, 5);
         if (ret)
             goto err_rpm_put;
         ret = pivariety_start_streaming(pivariety);
         if (ret)
             goto err_rpm_put;
         ret = pivariety_x21_restore_geometry(pivariety);
         if (ret) {
             dev_err(&client->dev, "ISP output geometry setup failed: %d\n", ret);
             goto err_stop_stream;
         }
         /* ARDUCAM_OPENHD/ISP reloads the ISP and mode defaults on stream-on.
          * Reapply FPS afterwards, while keeping flip controls before start.
          * Call the setter directly: the cached value has not changed, so
          * __v4l2_ctrl_s_ctrl() may otherwise skip the hardware transaction. */
         ret = wait_for_free(pivariety, 5);
         if (ret)
             goto err_stop_stream;
         frame_rate = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);
         if (frame_rate) {
             ret = pivariety->arducam_isp ?
                   pivariety_arducam_isp_set_fps(pivariety, frame_rate->val) :
                   pivariety_s_ctrl(frame_rate);
             if (ret)
                 goto err_stop_stream;
         }
     } else {
         cancel_delayed_work(&pivariety->x21_timing_work);
         pivariety_stop_streaming(pivariety);
         pivariety->x21_fast_timing_active = false;
         pm_runtime_put(&client->dev);
     }
 
     pivariety->streaming = enable;
     if (enable && pivariety->x21_arducam_openhd) {
         ret = pivariety_x21_restore_iq(pivariety);
         if (ret) goto err_stop_stream;
     }
     if (enable && pivariety->x21_arducam_openhd && frame_rate &&
         ((x21_linear_720p120 && frame_rate->val == 120 &&
           pivariety->x21_iq[X21_AE_AUTO]->val != V4L2_EXPOSURE_AUTO) ||
          (pivariety->x21_hdr && pivariety->x21_hdr->val)))
         mod_delayed_work(system_wq, &pivariety->x21_timing_work,
                          msecs_to_jiffies(15000));
 
     if (pivariety->x21_hdr)
         __v4l2_ctrl_grab(pivariety->x21_hdr, enable);

     /*
      * vflip and hflip cannot change during streaming
      * Pivariety may not implement flip control.
      */
     if (pivariety->vflip)
         __v4l2_ctrl_grab(pivariety->vflip, enable);
 
     if (pivariety->hflip)
         __v4l2_ctrl_grab(pivariety->hflip, enable);
 
     mutex_unlock(&pivariety->mutex);
 
     return ret;
 
 err_stop_stream:
     pivariety->streaming = false;
     pivariety->x21_fast_timing_active = false;
     pivariety_stop_streaming(pivariety);
 err_rpm_put:
     pm_runtime_put(&client->dev);
 err_unlock:
     mutex_unlock(&pivariety->mutex);
 
     return ret;
 }
 
 static int __maybe_unused pivariety_suspend(struct device *dev)
 {
     struct i2c_client *client = to_i2c_client(dev);
     struct v4l2_subdev *sd = i2c_get_clientdata(client);
     struct pivariety *pivariety = to_pivariety(sd);
 
     cancel_delayed_work_sync(&pivariety->x21_timing_work);
     mutex_lock(&pivariety->mutex);
     pivariety->x21_fast_timing_active = false;
     if (pivariety->streaming)
         pivariety_stop_streaming(pivariety);
     mutex_unlock(&pivariety->mutex);
 
     return 0;
 }
 
 static int __maybe_unused pivariety_resume(struct device *dev)
 {
     struct i2c_client *client = to_i2c_client(dev);
     struct v4l2_subdev *sd = i2c_get_clientdata(client);
     struct pivariety *pivariety = to_pivariety(sd);
     struct v4l2_ctrl *rate;
     int ret;
 
     mutex_lock(&pivariety->mutex);
     if (pivariety->streaming) {
         ret = pivariety_start_streaming(pivariety);
         if (ret)
             goto error;
         rate = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);
         if (rate && pivariety->arducam_isp) {
             ret = pivariety_arducam_isp_set_fps(pivariety, rate->val);
             if (ret)
                 goto error;
         }
     }
 
     mutex_unlock(&pivariety->mutex);
     return 0;
 
 error:
     pivariety_stop_streaming(pivariety);
     pivariety->streaming = 0;
     mutex_unlock(&pivariety->mutex);
     return ret;
 }
 
static int ardu_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
    struct pivariety *pivariety = to_pivariety(sd);
    struct v4l2_ctrl *ctrl = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);

    if (fi->pad || !ctrl)
        return -EINVAL;
    mutex_lock(&pivariety->mutex);
    fi->interval.numerator = 1;
    fi->interval.denominator = ctrl->val;
    mutex_unlock(&pivariety->mutex);
    return 0;
}

static int ardu_s_frame_interval(struct v4l2_subdev *sd,
                                struct v4l2_subdev_frame_interval *fi)
{
    struct pivariety *pivariety = to_pivariety(sd);
    struct v4l2_ctrl *ctrl = get_control(pivariety, V4L2_CID_ARDUCAM_FRAME_RATE);
    u32 fps;
    int ret;

    if (fi->pad || !ctrl || !fi->interval.numerator || !fi->interval.denominator)
        return -EINVAL;
    fps = DIV_ROUND_CLOSEST(fi->interval.denominator, fi->interval.numerator);
    if (pivariety->x21_arducam_openhd)
        fps = min(fps, pivariety_arducam_isp_max_fps(pivariety));
    ret = v4l2_ctrl_s_ctrl(ctrl, fps);
    if (ret)
        return ret;
    fi->interval.numerator = 1;
    fi->interval.denominator = fps;
    return 0;
}

 static int pivariety_get_regulators(struct pivariety *pivariety)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
     int i;
 
     for (i = 0; i < ARDUCAM_NUM_SUPPLIES; i++)
         pivariety->supplies[i].supply = pivariety_supply_name[i];
 
     return devm_regulator_bulk_get(&client->dev,
                        ARDUCAM_NUM_SUPPLIES,
                        pivariety->supplies);
 }
 
 static int pivariety_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
                      struct v4l2_mbus_config *cfg)
 {
     struct pivariety *pivariety = to_pivariety(sd);
 
     if (pivariety->lanes > pivariety->bus.num_data_lanes)
         return -EINVAL;
 
     cfg->type = V4L2_MBUS_CSI2_DPHY;
     cfg->bus.mipi_csi2.flags = pivariety->bus.flags;
     cfg->bus.mipi_csi2.num_data_lanes = pivariety->lanes;
 
     return 0;
 }
 
 #ifdef RADXA
 static void pivariety_get_module_inf(struct pivariety *pivariety,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, "arducam-pivariety", sizeof("arducam-pivariety"));
	strlcpy(inf->base.module, "arducam",
		sizeof("arducam"));
	strlcpy(inf->base.lens, "default", sizeof("default"));
}

static int pivariety_get_channel_info(struct pivariety *pivariety, struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
    ch_info->width = pivariety->supported_formats[pivariety->current_format_idx]
        .resolution_set[pivariety->current_resolution_idx].width;
	ch_info->height = pivariety->supported_formats[pivariety->current_format_idx]
        .resolution_set[pivariety->current_resolution_idx].height;
	ch_info->vc = 0;
	ch_info->bus_fmt = pivariety->supported_formats[pivariety->current_format_idx].mbus_code; // Sketchy
	return 0;
}

 static long pivariety_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct pivariety *pivariety = to_pivariety(sd);
	struct rkmodule_channel_info *ch_info;
	long ret = 0;
	struct rkmodule_csi_dphy_param *dphy_param;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    
    #if 0
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w, stream;
	u8 lanes = pivariety->bus.num_data_lanes;
	struct rkmodule_exp_delay *exp_delay;
	struct rkmodule_exp_info *exp_info;
	int idx_max = 0;
	u64 pixel_rate = 0;
    #endif

	switch (cmd) {
#if 0
	case PREISP_CMD_SET_HDRAE_EXP:
		if (imx415->cur_mode->hdr_mode == HDR_X2)
			ret = imx415_set_hdrae(imx415, arg);
		else if (imx415->cur_mode->hdr_mode == HDR_X3)
			ret = imx415_set_hdrae_3frame(imx415, arg);
		break;
#endif
	case RKMODULE_GET_MODULE_INFO:
		pivariety_get_module_inf(pivariety, (struct rkmodule_inf *)arg);
		break;
#if 0
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = imx415->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = imx415->cur_mode->width;
		h = imx415->cur_mode->height;
		for (i = 0; i < imx415->cfg_num; i++) {
			if (w == imx415->supported_modes[i].width &&
			    h == imx415->supported_modes[i].height &&
			    imx415->supported_modes[i].hdr_mode == hdr->hdr_mode) {
				dev_info(&imx415->client->dev, "set hdr cfg, set mode to %d\n", i);
				imx415_change_mode(imx415, &imx415->supported_modes[i]);
				break;
			}
		}
		if (i == imx415->cfg_num) {
			dev_err(&imx415->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			mode = imx415->cur_mode;
			if (imx415->streaming) {
				ret = imx415_write_reg(imx415->client, IMX415_GROUP_HOLD_REG,
					IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_START);

				ret |= imx415_write_array(imx415->client, imx415->cur_mode->reg_list);

				ret |= imx415_write_reg(imx415->client, IMX415_GROUP_HOLD_REG,
					IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_END);
				if (ret)
					return ret;
			}
			w = mode->hts_def - imx415->cur_mode->width;
			h = mode->vts_def - mode->height;
			mutex_lock(&imx415->mutex);
			__v4l2_ctrl_modify_range(imx415->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(imx415->vblank, h,
				IMX415_VTS_MAX - mode->height,
				1, h);
			__v4l2_ctrl_s_ctrl(imx415->link_freq, mode->mipi_freq_idx);
			pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
				mode->bpp * 2 * lanes;
			__v4l2_ctrl_s_ctrl_int64(imx415->pixel_rate,
						 pixel_rate);
			mutex_unlock(&imx415->mutex);
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = imx415_write_reg(imx415->client, IMX415_REG_CTRL_MODE,
				IMX415_REG_VALUE_08BIT, IMX415_MODE_STREAMING);
		else
			ret = imx415_write_reg(imx415->client, IMX415_REG_CTRL_MODE,
				IMX415_REG_VALUE_08BIT, IMX415_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_SONY_BRL:
		if (imx415->cur_mode->width == 3864 && imx415->cur_mode->height == 2192)
			*((u32 *)arg) = BRL_ALL;
		else
			*((u32 *)arg) = BRL_BINNING;
		break;
#endif
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = pivariety_get_channel_info(pivariety, ch_info);
		break;
	case RKMODULE_GET_CSI_DPHY_PARAM:
        dphy_param = (struct rkmodule_csi_dphy_param *)arg;
        *dphy_param = dcphy_param;
        dev_info(&client->dev,
             "get sensor dphy param\n");
		break;
#if 0
	case RKMODULE_GET_EXP_DELAY:
		exp_delay = (struct rkmodule_exp_delay *)arg;
		exp_delay->exp_delay = 2;
		exp_delay->gain_delay = 2;
		exp_delay->vts_delay = 1;
		break;
	case RKMODULE_GET_EXP_INFO:
		exp_info = (struct rkmodule_exp_info *)arg;
		if (imx415->cur_mode->hdr_mode == NO_HDR)
			idx_max = 1;
		else if (imx415->cur_mode->hdr_mode == HDR_X2)
			idx_max = 2;
		else
			idx_max = 3;
		for (i = 0; i < idx_max; i++) {
			exp_info->exp[i] = imx415->cur_exposure[i];
			exp_info->gain[i] = imx415->cur_gain[i];
		}
		exp_info->hts = imx415->cur_mode->hts_def;
		exp_info->vts = imx415->cur_vts;
		exp_info->pclk = imx415->pclk;
		exp_info->gain_mode.gain_mode = RKMODULE_GAIN_MODE_DB;
		exp_info->gain_mode.factor = 1000;
		break;
    #endif
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

 static const struct v4l2_subdev_core_ops pivariety_core_ops = {
     .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
     .unsubscribe_event = v4l2_event_subdev_unsubscribe,
#ifdef RADXA
     .ioctl = pivariety_ioctl,
#endif
 };
 
 static const struct v4l2_subdev_video_ops pivariety_video_ops = {
     .s_stream = pivariety_set_stream,
     .g_frame_interval =ardu_g_frame_interval,
     .s_frame_interval =ardu_s_frame_interval,
 };
 
 static const struct v4l2_subdev_pad_ops pivariety_pad_ops = {
     .enum_mbus_code = pivariety_enum_mbus_code,
     .get_fmt = pivariety_get_fmt,
     .set_fmt = pivariety_set_fmt,
     .enum_frame_size = pivariety_enum_framesizes,
     .enum_frame_interval = pivariety_enum_frame_interval,
     .get_selection = pivariety_get_selection,
     .get_mbus_config = pivariety_get_mbus_config,
 };
 
 static const struct v4l2_subdev_ops pivariety_subdev_ops = {
     .core = &pivariety_core_ops,
     .video = &pivariety_video_ops,
     .pad = &pivariety_pad_ops,
 };
 
 static const struct v4l2_subdev_internal_ops pivariety_internal_ops = {
     .open = pivariety_open,
 };
 
 static void pivariety_free_controls(struct pivariety *pivariety)
 {
     v4l2_ctrl_handler_free(pivariety->sd.ctrl_handler);
     mutex_destroy(&pivariety->mutex);
 }
 
 static int pivariety_get_length_of_set(struct pivariety *pivariety,
                        u16 idx_reg, u16 val_reg)
 {
     int ret;
     int index = 0;
     u32 val;
 
     while (1) {
         ret = pivariety_write(pivariety, idx_reg, index);
         ret += pivariety_read(pivariety, val_reg, &val);
 
         if (ret < 0)
             return -1;
 
         if (val == NO_DATA_AVAILABLE)
             break;
         index++;
     }
     pivariety_write(pivariety, idx_reg, 0);
     return index;
 }
 
 static int pivariety_enum_resolution(struct pivariety *pivariety,
                      struct arducam_format *format)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
     int index = 0;
     u32 width, height;
     int num_resolution = 0;
     int ret;
 
     num_resolution = pivariety_get_length_of_set(pivariety,
                              RESOLUTION_INDEX_REG,
                              FORMAT_WIDTH_REG);
     if (num_resolution < 0)
         goto err;
 
     format->resolution_set = devm_kzalloc(&client->dev,
                           sizeof(*format->resolution_set) *
                                 num_resolution,
                           GFP_KERNEL);
     while (1) {
         ret = pivariety_write(pivariety, RESOLUTION_INDEX_REG, index);
         ret += pivariety_read(pivariety, FORMAT_WIDTH_REG, &width);
         ret += pivariety_read(pivariety, FORMAT_HEIGHT_REG, &height);
 
         if (ret < 0)
             goto err;
 
         if (width == NO_DATA_AVAILABLE || height == NO_DATA_AVAILABLE)
             break;
 
         format->resolution_set[index].width = width;
         format->resolution_set[index].height = height;
   printk(KERN_INFO "pivariety: format[%d] resolution[%d]: %ux%u\n",
                format->index, index, width, height);
         index++;
     }
 
     format->num_resolution_set = index;
     pivariety_write(pivariety, RESOLUTION_INDEX_REG, 0);
     return 0;
 err:
     return -ENODEV;
 }
 
 static int pivariety_enum_pixformat(struct pivariety *pivariety)
 {
     int ret = 0;
     u32 mbus_code = 0;
     int pixfmt_type;
     int bayer_order;
     int bayer_order_not_volatile;
     int lanes;
     int index = 0;
     int num_pixformat = 0;
     struct arducam_format *arducam_fmt;
     struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
 
     num_pixformat = pivariety_get_length_of_set(pivariety,
                             PIXFORMAT_INDEX_REG,
                             PIXFORMAT_TYPE_REG);
 
     if (num_pixformat < 0)
         goto err;
 
     ret = pivariety_read(pivariety, FLIPS_DONT_CHANGE_ORDER_REG,
                  &bayer_order_not_volatile);
     if (bayer_order_not_volatile == NO_DATA_AVAILABLE)
         pivariety->bayer_order_volatile = 1;
     else
         pivariety->bayer_order_volatile = !bayer_order_not_volatile;
 
     if (ret < 0)
         goto err;
 
     pivariety->supported_formats =
         devm_kzalloc(&client->dev,
                  sizeof(*pivariety->supported_formats) *
                                 num_pixformat,
                  GFP_KERNEL);
 
     while (1) {
         ret = pivariety_write(pivariety, PIXFORMAT_INDEX_REG, index);
         ret += pivariety_read(pivariety, PIXFORMAT_TYPE_REG,
                       &pixfmt_type);
 
         if (pixfmt_type == NO_DATA_AVAILABLE)
             break;
 
         ret += pivariety_read(pivariety, MIPI_LANES_REG, &lanes);
         if (lanes == NO_DATA_AVAILABLE)
             break;
 
         ret += pivariety_read(pivariety, PIXFORMAT_ORDER_REG,
                       &bayer_order);
         if (ret < 0)
             goto err;
 
         mbus_code = data_type_to_mbus_code(pixfmt_type, bayer_order);
         arducam_fmt = &pivariety->supported_formats[index];
         arducam_fmt->index = index;
         arducam_fmt->mbus_code = mbus_code;
         arducam_fmt->bayer_order = bayer_order;
         arducam_fmt->data_type = pixfmt_type;
         if (pivariety_enum_resolution(pivariety, arducam_fmt))
             goto err;
 
           printk(KERN_INFO "pivariety: format[%d]: mbus_code=0x%x, "
                          "pixfmt_type=0x%x, bayer_order=%d, lanes=%d but kinda 4??? THX arducam :)\n",
                 index, mbus_code, pixfmt_type, bayer_order, lanes);

         index++;
     }
 
     pivariety_write(pivariety, PIXFORMAT_INDEX_REG, 0);
     pivariety->num_supported_formats = index;
     pivariety->current_format_idx = 0;
     pivariety->current_resolution_idx = 0;
     pivariety->lanes = 4;
 
     return 0;
 
 err:
     return -ENODEV;
 }
 
 static const char *pivariety_ctrl_get_name(u32 id)
 {
     switch (id) {
     case V4L2_CID_ARDUCAM_FRAME_RATE:
         return "frame_rate";
     case V4L2_CID_ARDUCAM_EXT_TRI:
         return "trigger_mode";
     case V4L2_CID_ARDUCAM_IRCUT:
         return "ircut";
     case V4L2_CID_ARDUCAM_STROBE_SHIFT:
         return "strobe_shift";
     case V4L2_CID_ARDUCAM_STROBE_WIDTH:
         return "strobe_width";
     case V4L2_CID_ARDUCAM_MODE:
         return "mode";
     default:
         return NULL;
     }
 }
 
 enum v4l2_ctrl_type pivariety_get_v4l2_ctrl_type(u32 id)
 {
     switch (id) {
     case V4L2_CID_ARDUCAM_EXT_TRI:
         return V4L2_CTRL_TYPE_BOOLEAN;
     case V4L2_CID_ARDUCAM_IRCUT:
         return V4L2_CTRL_TYPE_BOOLEAN;
     default:
         return V4L2_CTRL_TYPE_INTEGER;
     }
 }
 
 static struct v4l2_ctrl *v4l2_ctrl_new_arducam(struct v4l2_ctrl_handler *hdl,
                            const struct v4l2_ctrl_ops *ops,
                            u32 id, s64 min, s64 max,
                            u64 step, s64 def)
 {
     struct v4l2_ctrl_config ctrl_cfg = {
         .ops = ops,
         .id = id,
         .name = NULL,
         .type = V4L2_CTRL_TYPE_INTEGER,
         .flags = 0,
         .min = min,
         .max = max,
         .def = def,
         .step = step,
     };
 
     ctrl_cfg.name = pivariety_ctrl_get_name(id);
     ctrl_cfg.type = pivariety_get_v4l2_ctrl_type(id);
 
     return v4l2_ctrl_new_custom(hdl, &ctrl_cfg, NULL);
 }
 
 static int pivariety_enum_controls(struct pivariety *pivariety)
 {
     struct v4l2_subdev *sd = &pivariety->sd;
     struct i2c_client *client = v4l2_get_subdevdata(sd);
     struct v4l2_ctrl_handler *ctrl_hdlr = &pivariety->ctrl_handler;
     struct v4l2_fwnode_device_properties props;
     struct v4l2_ctrl **ctrl = pivariety->ctrls;
     int ret, index, num_ctrls;
     u32 id, min, max, def, step;
     struct v4l2_ctrl *ctrl1;

     num_ctrls = pivariety_get_length_of_set(pivariety, CTRL_INDEX_REG,
                         CTRL_ID_REG);
     if (num_ctrls < 0)
         goto err;
 
     v4l2_dbg(1, debug, sd, "%s: num_ctrls = %d\n",
          __func__, num_ctrls);
 
     ret = v4l2_ctrl_handler_init(ctrl_hdlr, num_ctrls);
     if (ret)
         return ret;
     ctrl_hdlr->lock = &pivariety->mutex;
 
          ctrl1 = v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl1)
		ctrl1->flags |= V4L2_CTRL_FLAG_READ_ONLY;

     index = 0;
     while (1) {
         ret = pivariety_write(pivariety, CTRL_INDEX_REG, index);
         pivariety_write(pivariety, CTRL_VALUE_REG, 0);
         wait_for_free(pivariety, 1);
 
         ret += pivariety_read(pivariety, CTRL_ID_REG, &id);
         ret += pivariety_read(pivariety, CTRL_MAX_REG, &max);
         ret += pivariety_read(pivariety, CTRL_MIN_REG, &min);
         ret += pivariety_read(pivariety, CTRL_DEF_REG, &def);
         ret += pivariety_read(pivariety, CTRL_STEP_REG, &step);
         if (ret < 0)
             goto err;
 
         if (id == NO_DATA_AVAILABLE || max == NO_DATA_AVAILABLE ||
             min == NO_DATA_AVAILABLE || def == NO_DATA_AVAILABLE ||
             step == NO_DATA_AVAILABLE)
             break;

         /* Replace MCU-advertised IQ entries with standard ISP-backed
          * controls below, rather than registering duplicate IDs. */
         if (pivariety->x21_arducam_openhd && pivariety->arducam_isp &&
             pivariety->lanes == 4 && pivariety_x21_iq_index(id) >= 0) {
             index++;
             continue;
         }
 
         v4l2_dbg(1, debug, sd,
              "%s: index = %d, id = 0x%x, max = %d, min = %d, def = %d, step = %d\n",
              __func__, index, id, max, min, def, step);
 
         if (pivariety->x21_arducam_openhd && id == V4L2_CID_ARDUCAM_FRAME_RATE) {
             max = min(max, pivariety_arducam_isp_max_fps(pivariety));
             def = clamp_t(u32, 30, min, max);
         }

         if (v4l2_ctrl_get_name(id)) {
             *ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
                           &pivariety_ctrl_ops,
                           id, min,
                           max, step,
                           def);
             v4l2_dbg(1, debug, sd, "%s: ctrl: 0x%p\n",
                  __func__, *ctrl);
         } else if (pivariety_ctrl_get_name(id)) {
             *ctrl = v4l2_ctrl_new_arducam(ctrl_hdlr,
                               &pivariety_ctrl_ops,
                               id, min, max, step, def);
 
             v4l2_dbg(1, debug, sd,
                  "%s: new custom ctrl, ctrl: 0x%p.\n",
                  __func__, *ctrl);
         } else {
             index++;
             continue;
         }
 
         if (!*ctrl)
             goto err;
 
         switch (id) {
         case V4L2_CID_HFLIP:
             pivariety->hflip = *ctrl;
             if (pivariety->bayer_order_volatile)
                 pivariety->hflip->flags |=
                         V4L2_CTRL_FLAG_MODIFY_LAYOUT;
             break;
 
         case V4L2_CID_VFLIP:
             pivariety->vflip = *ctrl;
             if (pivariety->bayer_order_volatile)
                 pivariety->vflip->flags |=
                         V4L2_CTRL_FLAG_MODIFY_LAYOUT;
             break;
 
         case V4L2_CID_HBLANK:
             (*ctrl)->flags |= V4L2_CTRL_FLAG_READ_ONLY;
             break;
         }
 
         ctrl++;
         index++;
     }
 
     pivariety_write(pivariety, CTRL_INDEX_REG, 0);
 
     if (pivariety->x21_arducam_openhd && pivariety->arducam_isp && pivariety->lanes == 4) {
         pivariety->x21_hdr = v4l2_ctrl_new_std(ctrl_hdlr,
             &pivariety_ctrl_ops, V4L2_CID_WIDE_DYNAMIC_RANGE, 0, 1, 1, 0);
         if (!pivariety->x21_hdr) goto err;
         {
             static const struct { int min, max, step, def; } iq[] = {
                 {1,40,1,10}, /* 100 us, not sensor lines */
                 {256,2048,16,256}, /* U8.8 */
                 {0,1,1,1},
                 {2000,10000,100,6500},
                 {-4096,4096,1,0}, /* S3.12 */
                 {0,8192,1,4096},
                 {0,8192,1,4096},
                 {0,8192,1,0}, /* 0 selects ISP sRGB */
             };
             unsigned int i;
             for (i = 0; i < ARRAY_SIZE(iq); i++) {
                 pivariety->x21_iq[i] = v4l2_ctrl_new_std(ctrl_hdlr,
                     &pivariety_ctrl_ops, x21_iq_ids[i], iq[i].min,
                     iq[i].max, iq[i].step, iq[i].def);
                 if (!pivariety->x21_iq[i]) goto err;
             }
             pivariety->x21_iq[X21_AE_AUTO] = v4l2_ctrl_new_std_menu(ctrl_hdlr,
                 &pivariety_ctrl_ops, V4L2_CID_EXPOSURE_AUTO,
                 V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
             if (!pivariety->x21_iq[X21_AE_AUTO]) goto err;
             pivariety->x21_iq[X21_AE_SCENE] = v4l2_ctrl_new_std_menu(ctrl_hdlr,
                 &pivariety_ctrl_ops, V4L2_CID_SCENE_MODE, V4L2_SCENE_MODE_SPORTS,
                 ((1ULL << (V4L2_SCENE_MODE_SPORTS + 1)) - 1) &
                 ~((1ULL << V4L2_SCENE_MODE_NONE) | (1ULL << V4L2_SCENE_MODE_BACKLIGHT) |
                   (1ULL << V4L2_SCENE_MODE_SPORTS)), V4L2_SCENE_MODE_NONE);
             if (!pivariety->x21_iq[X21_AE_SCENE]) goto err;
             {
                 static const s64 ev[] = {-2000,-1750,-1500,-1250,-1000,-750,-500,-250,
                                           0,250,500,750,1000,1250,1500,1750,2000};
                 pivariety->x21_iq[X21_AE_BIAS] = v4l2_ctrl_new_int_menu(ctrl_hdlr,
                     &pivariety_ctrl_ops, V4L2_CID_AUTO_EXPOSURE_BIAS, 16, 8, ev);
                 if (!pivariety->x21_iq[X21_AE_BIAS]) goto err;
             }
         }
     }

     ret = v4l2_fwnode_device_parse(&client->dev, &props);
     if (ret)
         goto err;
 
     ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr,
                           &pivariety_ctrl_ops,
                           &props);
     if (ret)
         goto err;
 
     pivariety->sd.ctrl_handler = ctrl_hdlr;
     v4l2_ctrl_handler_setup(ctrl_hdlr);
     return 0;
 err:
     return -ENODEV;
 }
 
 static int pivariety_parse_dt(struct pivariety *pivariety, struct device *dev)
 {
     struct fwnode_handle *endpoint;
     struct v4l2_fwnode_endpoint ep_cfg = {
         .bus_type = V4L2_MBUS_CSI2_DPHY
     };
     int ret = -EINVAL;
 
     /* Get CSI2 bus config */
     endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
     if (!endpoint) {
         dev_err(dev, "endpoint node not found\n");
         return -EINVAL;
     }
 
     if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
         dev_err(dev, "could not parse endpoint\n");
         goto error_out;
     }
 
     pivariety->bus = ep_cfg.bus.mipi_csi2;
 
     ret = 0;
 
 error_out:
     v4l2_fwnode_endpoint_free(&ep_cfg);
     fwnode_handle_put(endpoint);
 
     return ret;
 }
 
static int pivariety_probe(struct i2c_client *client,
    const struct i2c_device_id *id) {
    
        struct device *dev = &client->dev;
        struct pivariety *pivariety;
        u32 device_id, firmware_version, sensor_id;
        int ret;

        pivariety = devm_kzalloc(dev, sizeof(*pivariety), GFP_KERNEL);
        if (!pivariety)
        return -ENOMEM;
        mutex_init(&pivariety->mutex);
        INIT_DELAYED_WORK(&pivariety->x21_timing_work,
                          pivariety_x21_timing_work);
        pivariety->x21_arducam_openhd = of_device_is_compatible(dev->of_node,
                                      "openhd,x21-arducam_openhd");

        /* Initialize subdev */
        v4l2_i2c_subdev_init(&pivariety->sd, client, &pivariety_subdev_ops);

        /* Basic subdev setup before registration */
        pivariety->sd.internal_ops = &pivariety_internal_ops;
        pivariety->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
        pivariety->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
        pivariety->pad.flags = MEDIA_PAD_FL_SOURCE;
        pivariety->sd.fwnode = dev_fwnode(dev);

        /* Initialize media entity + pad */
        ret = media_entity_pads_init(&pivariety->sd.entity, 1, &pivariety->pad);
        if (ret) {
        dev_err(dev, "media_entity_pads_init failed: %d\n", ret);
        return ret;
        }

        /* Parse device tree */
        if (pivariety_parse_dt(pivariety, dev)) {
        ret = -EINVAL;
        goto error_media_entity;
        }

        /* Get system clock (xclk) */
        pivariety->xclk = devm_clk_get(dev, "xclk");
        if (IS_ERR(pivariety->xclk)) {
        dev_err(dev, "failed to get xclk\n");
        ret = PTR_ERR(pivariety->xclk);
        goto error_media_entity;
        }

        pivariety->xclk_freq = clk_get_rate(pivariety->xclk);
        if (pivariety->xclk_freq != 24000000) {
        dev_err(dev, "xclk frequency not supported: %d Hz\n",
        pivariety->xclk_freq);
        ret = -EINVAL;
        goto error_media_entity;
        }

        /* Get optional power regulators */
        ret = pivariety_get_regulators(pivariety);
        if (ret)
        goto error_media_entity;

        /* Optional reset GPIO */
        pivariety->reset_gpio = devm_gpiod_get_optional(dev, "reset",
                                    GPIOD_OUT_HIGH);

        /* Power on the device */
        ret = pivariety_power_on(dev);
        if (ret)
        goto error_media_entity;

        /* Read ID and firmware version */
        ret = pivariety_read(pivariety, DEVICE_ID_REG, &device_id);
        if (ret || device_id != DEVICE_ID) {
        dev_err(dev, "probe failed: invalid device id\n");
        ret = -ENODEV;
        goto error_power_off;
        }

        ret = pivariety_read(pivariety, DEVICE_VERSION_REG, &firmware_version);
        if (ret)
            goto error_power_off;

        dev_info(dev, "firmware version: 0x%04X\n", firmware_version);

        ret = pivariety_read(pivariety, SENSOR_ID_REG, &sensor_id);
        if (ret)
            goto error_power_off;
        pivariety->arducam_isp = sensor_id == 0x1302;

        /* Explicit board/module profile only; ordinary Pivariety cameras
         * retain their firmware. Runs before any capture client can open it. */
        ret = pivariety_ensure_x21_firmware(pivariety);
        if (ret)
            goto error_power_off;

        /* Enumerate formats and controls */
        if (pivariety_enum_pixformat(pivariety)) {
        dev_err(dev, "enum pixformat failed.\n");
        ret = -ENODEV;
        goto error_power_off;
        }

        if (pivariety_enum_controls(pivariety)) {
        dev_err(dev, "enum controls failed.\n");
        ret = -ENODEV;
        goto error_power_off;
        }

        /* Publish only after formats, controls and their lock are ready. */
        ret = v4l2_async_register_subdev_sensor(&pivariety->sd);
        if (ret < 0)
            goto error_power_off;

        pm_runtime_set_active(dev);
        pm_runtime_enable(dev);
        pm_runtime_idle(dev);

        return 0;

        error_power_off:
        pivariety_power_off(dev);

        error_media_entity:
        media_entity_cleanup(&pivariety->sd.entity);
        pivariety_free_controls(pivariety);

        return ret;
    }

static void pivariety_remove(struct i2c_client *client)
 {
     struct v4l2_subdev *sd = i2c_get_clientdata(client);
     struct pivariety *pivariety = to_pivariety(sd);
 
     v4l2_async_unregister_subdev(sd);
     cancel_delayed_work_sync(&pivariety->x21_timing_work);
     media_entity_cleanup(&sd->entity);
     pivariety_free_controls(pivariety);
 
     pm_runtime_disable(&client->dev);
     pm_runtime_set_suspended(&client->dev);
 }
 
 static const struct dev_pm_ops pivariety_pm_ops = {
     SET_SYSTEM_SLEEP_PM_OPS(pivariety_suspend, pivariety_resume)
     SET_RUNTIME_PM_OPS(pivariety_power_off, pivariety_power_on, NULL)
 };
 
 static const struct of_device_id arducam_pivariety_dt_ids[] = {
     { .compatible = "openhd,x21-arducam_openhd" },
     { .compatible = "arducam,arducam-pivariety" },
     { /* sentinel */ }
 };
 MODULE_DEVICE_TABLE(of, arducam_pivariety_dt_ids);
 
 static struct i2c_driver arducam_pivariety_i2c_driver = {
     .driver = {
         .name = "arducam-pivariety",
         .of_match_table	= arducam_pivariety_dt_ids,
         .pm = &pivariety_pm_ops,
     },
     .probe = pivariety_probe,
     .remove = pivariety_remove,
 };
 
 module_i2c_driver(arducam_pivariety_i2c_driver);
 
 MODULE_AUTHOR("Lee Jackson <info@arducam.com>");
 MODULE_DESCRIPTION("Arducam Pivariety v4l2 driver");
 MODULE_LICENSE("GPL v2");
