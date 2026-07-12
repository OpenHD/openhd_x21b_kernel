/* SPDX-License-Identifier: GPL-2.0 */
#ifndef OPENHD_CAMERA_H
#define OPENHD_CAMERA_H

enum runcam_attr_id {
	RUNCAM_ATTR_BRIGHTNESS,
	RUNCAM_ATTR_SHARPNESS,
	RUNCAM_ATTR_CONTRAST,
	RUNCAM_ATTR_SATURATION,
	RUNCAM_ATTR_SHUTTER,
	RUNCAM_ATTR_WB_MODE,
	RUNCAM_ATTR_WB_RED,
	RUNCAM_ATTR_WB_BLUE,
	RUNCAM_ATTR_HV_FLIP,
	RUNCAM_ATTR_NIGHT_MODE,
	RUNCAM_ATTR_LED_MODE,
	RUNCAM_ATTR_VIDEO_FMT,

	RUNCAM_ATTR_COUNT,
};

struct runcam_attribute {
	const char *name;

	bool enabled;

	u32 min;
	u32 max;
	u32 def;
};

struct runcam_mode {
	const char *name;

	u32 width;
	u32 height;
	u32 fps;

	u32 mode_reg;
	u32 timing_reg;

	bool needs_isp_reset;
	bool bw_17m;
    u64 link_freq_hz;
};

struct runcam_model_info {
	const char *name;

	u8 i2c_addr;

	bool skip_detect;
	u32 detect_reg;
	bool detect_by_write;
	u32 detect_value;

	const struct runcam_attribute attrs[RUNCAM_ATTR_COUNT];

	const struct runcam_mode *modes;
	unsigned int num_modes;
};

/* Micro V1 */

static const struct runcam_mode runcam_micro_v1_modes[] = {
	{
		.name = "720p60",
		.width = 1280,
		.height = 720,
		.fps = 60,
		.link_freq_hz  = 331776000ULL,
	},
};

static const struct runcam_model_info runcam_micro_v1_info = {
	.name = "micro-v1",
	.i2c_addr = 0x21,

	.skip_detect = false,
	.detect_reg = 0x50,
	.detect_by_write = true,
	.detect_value = 0x0452484E,

	.attrs = {
		[RUNCAM_ATTR_BRIGHTNESS] = {
			.name = "brightness",
			.enabled = true,
			.min = 0x40,
			.max = 0xC0,
			.def = 0x80,
		},
		[RUNCAM_ATTR_SHARPNESS] = {
			.name = "sharpness",
			.enabled = true,
			.min = 0,
			.max = 2,
			.def = 1,
		},
		[RUNCAM_ATTR_CONTRAST] = {
			.name = "contrast",
			.enabled = true,
			.min = 0,
			.max = 2,
			.def = 1,
		},
		[RUNCAM_ATTR_SATURATION] = {
			.name = "saturation",
			.enabled = true,
			.min = 0,
			.max = 6,
			.def = 3,
		},
		[RUNCAM_ATTR_SHUTTER] = {
			.name = "shutter",
			.enabled = false,
			.min = 0,
			.max = 0x20,
			.def = 0,
		},
		[RUNCAM_ATTR_WB_MODE] = {
			.name = "wb-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 0,
		},
		[RUNCAM_ATTR_WB_RED] = {
			.name = "wb-red",
			.enabled = true,
			.min = 0,
			.max = 0xff,
			.def = 0xc7,
		},
		[RUNCAM_ATTR_WB_BLUE] = {
			.name = "wb-blue",
			.enabled = true,
			.min = 0,
			.max = 0xff,
			.def = 0xca,
		},
		[RUNCAM_ATTR_HV_FLIP] = {
			.name = "hv-flip",
			.enabled = false,
			.min = 0,
			.max = 3,
			.def = 0,
		},
		[RUNCAM_ATTR_NIGHT_MODE] = {
			.name = "night-mode",
			.enabled = false,
			.min = 0,
			.max = 1,
			.def = 1,
		},
		[RUNCAM_ATTR_LED_MODE] = {
			.name = "led-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 0,
		},
		[RUNCAM_ATTR_VIDEO_FMT] = {
			.name = "video-format",
			.enabled = false,
			.min = 0,
			.max = 0,
			.def = 0,
		},
	},

	.modes = runcam_micro_v1_modes,
	.num_modes = ARRAY_SIZE(runcam_micro_v1_modes),
};

/* Micro V2 */
static const struct runcam_mode runcam_micro_v2_modes[] = {
	{
		.name = "720p60-4:3",
		.width = 1280,
		.height = 720,
		.fps = 60,
		.mode_reg = 0x0008910B,
		.timing_reg = 0x00012941,
		.needs_isp_reset = true,
		.link_freq_hz  = 331776000ULL,
	},
	{
		.name = "720p60-16:9-crop",
		.width = 1280,
		.height = 720,
		.fps = 60,
		.mode_reg = 0x00089102,
		.timing_reg = 0x00012941,
		.needs_isp_reset = true,
		.link_freq_hz  = 331776000ULL,
	},
	{
		.name = "720p60-16:9-full",
		.width = 1280,
		.height = 720,
		.fps = 60,
		.mode_reg = 0x00089110,
		.timing_reg = 0x00012941,
		.needs_isp_reset = true,
		.link_freq_hz  = 331776000ULL,
	},
	{
		.name = "1080p30",
		.width = 1920,
		.height = 1080,
		.fps = 30,
		.mode_reg = 0x81089106,
		.timing_reg = 0x00014441,
		.needs_isp_reset = true,
		.link_freq_hz  = 373248000ULL,
	},
};

static const struct runcam_model_info runcam_micro_v2_info = {
	.name = "micro-v2",
	.i2c_addr = 0x22,

	.skip_detect = false,
	.detect_reg = 0x50,

	.attrs = {
		[RUNCAM_ATTR_BRIGHTNESS] = {
			.name = "brightness",
			.enabled = true,
			.min = 0x40,
			.max = 0xC0,
			.def = 0x80,
		},
		[RUNCAM_ATTR_SHARPNESS] = {
			.name = "sharpness",
			.enabled = true,
			.min = 0,
			.max = 2,
			.def = 1,
		},
		[RUNCAM_ATTR_CONTRAST] = {
			.name = "contrast",
			.enabled = true,
			.min = 0,
			.max = 2,
			.def = 1,
		},
		[RUNCAM_ATTR_SATURATION] = {
			.name = "saturation",
			.enabled = true,
			.min = 0,
			.max = 6,
			.def = 5,
		},
		[RUNCAM_ATTR_SHUTTER] = {
			.name = "shutter",
			.enabled = true,
			.min = 0,
			.max = 0x20,
			.def = 0,
		},
		[RUNCAM_ATTR_WB_MODE] = {
			.name = "wb-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 0,
		},
		[RUNCAM_ATTR_WB_RED] = {
			.name = "wb-red",
			.enabled = true,
			.min = 0,
			.max = 0xff,
			.def = 0xc7,
		},
		[RUNCAM_ATTR_WB_BLUE] = {
			.name = "wb-blue",
			.enabled = true,
			.min = 0,
			.max = 0xff,
			.def = 0xca,
		},
		[RUNCAM_ATTR_HV_FLIP] = {
			.name = "hv-flip",
			.enabled = true,
			.min = 0,
			.max = 3,
			.def = 0,
		},
		[RUNCAM_ATTR_NIGHT_MODE] = {
			.name = "night-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 1,
		},
		[RUNCAM_ATTR_LED_MODE] = {
			.name = "led-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 0,
		},
		[RUNCAM_ATTR_VIDEO_FMT] = {
			.name = "video-format",
			.enabled = true,
			.min = 0,
			.max = 3,
			.def = 0,
		},
	},

	.modes = runcam_micro_v2_modes,
	.num_modes = ARRAY_SIZE(runcam_micro_v2_modes),
};

/* Nano 90 */

static const struct runcam_mode runcam_nano90_modes[] = {
	{
		.name = "540p90",
		.width = 720,
		.height = 540,
		.fps = 90,
		.mode_reg = 0x8008811d,
		.needs_isp_reset = true,
        .link_freq_hz  = 84000000ULL,
	},
	{
		.name = "540p90-crop",
		.width = 720,
		.height = 540,
		.fps = 90,
		.mode_reg = 0x83088120,
		.needs_isp_reset = true,
        .link_freq_hz  = 84000000ULL,
	},
	{
		.name = "540p60",
		.width = 720,
		.height = 540,
		.fps = 60,
		.mode_reg = 0x8108811e,
		.needs_isp_reset = true,
		.bw_17m = true,
        .link_freq_hz  = 56000000ULL,
	},
	{
		.name = "960x720p60",
		.width = 960,
		.height = 720,
		.fps = 60,
		.mode_reg = 0x8208811f,
		.needs_isp_reset = true,
        .link_freq_hz  = 100000000ULL,
	},
};

static const struct runcam_model_info runcam_nano90_info = {
	.name = "nano-90",
	.i2c_addr = 0x23,

	.skip_detect = false,
	.detect_reg = 0x50,

	.attrs = {
		[RUNCAM_ATTR_BRIGHTNESS] = {
			.name = "brightness",
			.enabled = true,
			.min = 0x40,
			.max = 0xC0,
			.def = 0x80,
		},
		[RUNCAM_ATTR_SHARPNESS] = {
			.name = "sharpness",
			.enabled = true,
			.min = 0,
			.max = 2,
			.def = 1,
		},
		[RUNCAM_ATTR_CONTRAST] = {
			.name = "contrast",
			.enabled = true,
			.min = 0,
			.max = 2,
			.def = 1,
		},
		[RUNCAM_ATTR_SATURATION] = {
			.name = "saturation",
			.enabled = true,
			.min = 0,
			.max = 6,
			.def = 5,
		},
		[RUNCAM_ATTR_SHUTTER] = {
			.name = "shutter",
			.enabled = true,
			.min = 0,
			.max = 0x20,
			.def = 0,
		},
		[RUNCAM_ATTR_WB_MODE] = {
			.name = "wb-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 0,
		},
		[RUNCAM_ATTR_WB_RED] = {
			.name = "wb-red",
			.enabled = true,
			.min = 0,
			.max = 0xff,
			.def = 0xc7,
		},
		[RUNCAM_ATTR_WB_BLUE] = {
			.name = "wb-blue",
			.enabled = true,
			.min = 0,
			.max = 0xff,
			.def = 0xca,
		},
		[RUNCAM_ATTR_HV_FLIP] = {
			.name = "hv-flip",
			.enabled = true,
			.min = 0,
			.max = 3,
			.def = 0,
		},
		[RUNCAM_ATTR_NIGHT_MODE] = {
			.name = "night-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 1,
		},
		[RUNCAM_ATTR_LED_MODE] = {
			.name = "led-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 0,
		},
		[RUNCAM_ATTR_VIDEO_FMT] = {
			.name = "video-format",
			.enabled = true,
			.min = 0,
			.max = 3,
			.def = 0,
		},
	},

	.modes = runcam_nano90_modes,
	.num_modes = ARRAY_SIZE(runcam_nano90_modes),
};

/* Micro V3 */

static const struct runcam_mode runcam_micro_v3_modes[] = {
	{
		.name = "720p60-4:3",
		.width = 1280,
		.height = 720,
		.fps = 60,
		.mode_reg = 0x8208910B,
		.timing_reg = 0x00012941,
		.needs_isp_reset = true,
		.link_freq_hz  = 331776000ULL,
	},
	{
		.name = "720p60-16:9-crop",
		.width = 1280,
		.height = 720,
		.fps = 60,
		.mode_reg = 0x82089102,
		.timing_reg = 0x00012941,
		.needs_isp_reset = true,
		.link_freq_hz  = 331776000ULL,
	},
	{
		.name = "720p60-16:9-full",
		.width = 1280,
		.height = 720,
		.fps = 60,
		.mode_reg = 0x82089110,
		.timing_reg = 0x00012941,
		.needs_isp_reset = true,
		.link_freq_hz  = 331776000ULL,
	},
	{
		.name = "1080p30",
		.width = 1920,
		.height = 1080,
		.fps = 30,
		.mode_reg = 0x81089106,
		.timing_reg = 0x00014441,
		.needs_isp_reset = true,
		.link_freq_hz  = 373248000ULL,
	},
};

static const struct runcam_model_info runcam_micro_v3_info = {
	.name = "micro-v3",
	.i2c_addr = 0x24,

	.skip_detect = false,
	.detect_reg = 0x50,

	.attrs = {
		[RUNCAM_ATTR_BRIGHTNESS] = {
			.name = "brightness",
			.enabled = true,
			.min = 0x40,
			.max = 0xC0,
			.def = 0x80,
		},
		[RUNCAM_ATTR_SHARPNESS] = {
			.name = "sharpness",
			.enabled = true,
			.min = 0,
			.max = 2,
			.def = 1,
		},
		[RUNCAM_ATTR_CONTRAST] = {
			.name = "contrast",
			.enabled = true,
			.min = 0,
			.max = 2,
			.def = 1,
		},
		[RUNCAM_ATTR_SATURATION] = {
			.name = "saturation",
			.enabled = true,
			.min = 0,
			.max = 6,
			.def = 5,
		},
		[RUNCAM_ATTR_SHUTTER] = {
			.name = "shutter",
			.enabled = true,
			.min = 0,
			.max = 0x20,
			.def = 0,
		},
		[RUNCAM_ATTR_WB_MODE] = {
			.name = "wb-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 0,
		},
		[RUNCAM_ATTR_WB_RED] = {
			.name = "wb-red",
			.enabled = true,
			.min = 0,
			.max = 0xff,
			.def = 0xc7,
		},
		[RUNCAM_ATTR_WB_BLUE] = {
			.name = "wb-blue",
			.enabled = true,
			.min = 0,
			.max = 0xff,
			.def = 0xca,
		},
		[RUNCAM_ATTR_HV_FLIP] = {
			.name = "hv-flip",
			.enabled = true,
			.min = 0,
			.max = 3,
			.def = 0,
		},
		[RUNCAM_ATTR_NIGHT_MODE] = {
			.name = "night-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 1,
		},
		[RUNCAM_ATTR_LED_MODE] = {
			.name = "led-mode",
			.enabled = true,
			.min = 0,
			.max = 1,
			.def = 0,
		},
		[RUNCAM_ATTR_VIDEO_FMT] = {
			.name = "video-format",
			.enabled = true,
			.min = 0,
			.max = 3,
			.def = 2,
		},
	},

	.modes = runcam_micro_v3_modes,
	.num_modes = ARRAY_SIZE(runcam_micro_v3_modes),
};

static const struct runcam_mode foxeer_digisight_v3_modes[] = {
	{
		.name = "720p60",
		.width = 1280,
		.height = 720,
		.fps = 60,
		.link_freq_hz  = 331776000ULL,
	},
};

static const struct runcam_model_info foxeer_digisight_v3_info = {
	.name = "foxeer-digisight-v3",
	.i2c_addr = 0x64,
	.skip_detect = true,

	.modes = foxeer_digisight_v3_modes,
	.num_modes = ARRAY_SIZE(foxeer_digisight_v3_modes),
};

#endif