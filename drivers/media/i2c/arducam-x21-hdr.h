/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef ARDUCAM_X21_HDR_H
#define ARDUCAM_X21_HDR_H
#include "arducam-x21-timing.h"
/* Three-exposure table from Kurokesu/arducam_openhd-rpi-driver, arducam_openhd.c.
 * Copyright (C) 2025-2026 UAB Kurokesu.
 * Keep the complete sensor field with 2x2 binning into the ISP's 1920x1080
 * input. Native 4K input requires a different ISP geometry configuration.
 */
static const unsigned short x21_hdr_rows[][2] = {
 {0x50a2,0x3f2a},
 {0x50a4,0x875a},
 {0x50a6,0x030f},
 {0x50a6,0x0f0f},
 {0x50a8,0x030f},
 {0x50a8,0x0f0f},
 {0x50aa,0x030f},
 {0x50aa,0x050f},
 {0x50ac,0x0301},
 {0x50ac,0x0101},
 {0x50ae,0x0301},
 {0x50ae,0x0101},
 {0x50b0,0x0301},
 {0x50b0,0x0101},
 {0x50b2,0x03ff},
 {0x50b4,0x030f},
 {0x50b4,0x0f0f},
 {0x50b6,0x030f},
 {0x50b6,0x0f0f},
 {0x50b8,0x030f},
 {0x50b8,0x050f},
 {0x3d2a,0x0fff},
 {0x3d34,0x9c40},
 {0x3d36,0xffff},
 {0x3d02,0x5033},
 {0x3086,0x1a28},
 {0x33e4,0x0040},
 {0x3c70,0x6222},
 {0x3280,0x0ed8},
 {0x3282,0x0ed8},
 {0x3284,0x0ed8},
 {0x3286,0x0ed8},
 {0x3288,0x0ed8},
 {0x328a,0x0ed8},
 {0x328c,0x0ed8},
 {0x328e,0x0ed8},
 {0x3290,0x0ed8},
 {0x3292,0x0ed8},
 {0x3294,0x0ed8},
 {0x3296,0x0ed8},
 {0x3100,0xc001},
 {0x3102,0xbed8},
 {0x3104,0xbed8},
 {0x3106,0xbed8},
 {0x3108,0x07d0},
 {0x3124,0x006d},
 {0x3126,0x003c},
 {0x3d00,0x300f},
 {0x3d28,0x0acd},
 {0x3d2a,0x0ed8},
 {0x3d34,0xacd0},
 {0x3d36,0xed80},
 {0x3d40,0xfffe},
 {0x3d42,0xffff},
 {0x3d64,0x0ed8},
 {0x3d66,0x0ed8},
 {0x3d68,0x0ed8},
 {0x3d6a,0x0ed8},
 {0x3d6c,0x0ed8},
 {0x3d6e,0x0ed8},
 {0x3d70,0x0ed8},
 {0x3d72,0x0ed8},
 {0x3d74,0x0ed8},
 {0x3d76,0x0ed8},
 {0x3d78,0x0ed8},
 {0x3d7a,0x0ed8},
 {0x33c0,0x2000},
 {0x33c2,0x3450},
 {0x33c4,0x48a0},
 {0x33c6,0x5cf0},
 {0x33c8,0x7140},
 {0x33ca,0x8590},
 {0x33cc,0x99e0},
 {0x33ce,0xae30},
 {0x33d0,0xc280},
 {0x33d2,0xd6d0},
 {0x33d4,0xeb20},
 {0x33d6,0xff70},
 {0x5202,0x37c3},
 {0x51ea,0x0149},
 {0x51fc,0x044d},
 {0x51ec,0x0700},
 {0x51ee,0x0001},
 {0x51f0,0x0002},
 {0x51f2,0x0003},
 {0x51f4,0x0004},
 {0x5402,0x37c3},
 {0x5560,0x0149},
 {0x556c,0x044d},
 {0x5562,0x0700},
 {0x5564,0x0001},
 {0x5566,0x0002},
 {0x5568,0x0003},
 {0x556a,0x0004},
 {0x5000,0x11c1},
 {0x5200,0x0180},
 {0x5400,0x0180},
 {0x3110,0x0011},
 {0x30b0,0x0820},
 {0x3116,0x2001},
 {0x3d60,0x0f0f},
 {0x31d0,0x0001},
 {0x3082,0x0008},
 {0x30ba,0x013e},
 {0x3238,0x0044},
 {0x5914,0x4006},
};

static int x21_hdr_stall(const struct x21_timing_ops *ops, int enable)
{
 unsigned int value, attempt;
 int ret = ops->read(ops->context, 0x601a, &value);
 if (ret || !!(value & 0x200) == enable) return ret;
 ret = ops->write(ops->context, 0x601a, 0x0140);
 if (ret) return ret;
 for (attempt = 0; attempt < 100; attempt++) {
  ops->delay(ops->context, 20);
  ret = ops->read(ops->context, 0x601a, &value);
  if (ret || !!(value & 0x200) == enable) return ret;
 }
 return -ETIMEDOUT;
}

static int x21_hdr_sensor_write(const struct x21_timing_ops *ops,
                               unsigned short reg, unsigned int value)
{
 unsigned int readback;
 int ret = x21_timing_sensor(ops, reg, &value, 1, 0);
 /* Gain-table load-complete is a read-only status bit. */
 if (ret == -EIO && reg == 0x5914) {
  ret = x21_timing_sensor(ops, reg, &readback, 0, 0);
  if (!ret && (readback & 0x7fff) != (value & 0x7fff)) ret = -EIO;
 }
 return ret;
}

static int x21_timing_hdr30(const struct x21_timing_ops *ops)
{
 static const unsigned short geometry[][2] = {
  {0x3002,8}, {0x3004,8}, {0x3006,2167}, {0x3008,3847},
  {0x30a2,3}, {0x30a6,3}, {0x3402,1920}, {0x3404,1080},
  {0x3040,0x3000}, {0x31ac,0x140c}, {0x3036,6},
  {0x300a,2250}, {0x300c,2440},
  {0x3012,512}, {0x3212,32}, {0x3216,2},
 };
 unsigned int value, width, height, i;
 int ret, resume_ret;
 ret = x21_timing_expect(ops, 0x0000, 0x0265);
 if (ret) return ret;
 ret = x21_timing_expect(ops, 0x0006, 0);
 if (ret) return ret;
 ret = ops->read(ops->context, 0x2000, &width);
 if (ret) return ret;
 ret = ops->read(ops->context, 0x2002, &height);
 if (ret) return ret;
 if (!((width == 1280 && height == 720) ||
       (width == 1920 && height == 1080))) return -ENODEV;
 ret = x21_timing_expect(ops, 0x1010, 0x0100);
 if (ret) return ret;
 ret = x21_timing_expect(ops, 0x6034, 600);
 if (ret) return ret;
 /* Never feed full-resolution data into an ISP configured for binned input. */
 ret = x21_timing_expect(ops, 0x2014, 2);
 if (ret) return ret;
 ret = x21_timing_expect(ops, 0x202e, 1);
 if (ret) return ret;
 ret = x21_timing_sensor(ops, 0x3000, &value, 0, 0);
 if (ret || value != 0x0f56) return ret ? ret : -ENODEV;
 ret = x21_timing_sensor(ops, 0x300e, &value, 0, 0);
 if (ret || value != 0x2303) return ret ? ret : -ENODEV;
 ret = ops->write(ops->context, 0x2020, 30 << 8);
 if (ret) return ret;
 ret = ops->read(ops->context, 0x5440, &value);
 if (ret) return ret;
 ret = ops->write(ops->context, 0x5440, value & 0xfff8);
 if (ret) return ret;
 ret = ops->write(ops->context, 0x500c, 0);
 if (ret) return ret;
 ret = ops->write(ops->context, 0x500e, 10000);
 if (ret) return ret;
 ret = ops->write(ops->context, 0x5002, 0x0290);
 if (ret) return ret;
 ret = ops->write(ops->context, 0x5002, 0x0690);
 if (ret) return ret;
 /* Let the vendor AE-mode update finish before installing a custom readout. */
 ops->delay(ops->context, 500);
 ret = x21_timing_expect(ops, 0x0006, 0);
 if (ret) return ret;
 ret = x21_hdr_stall(ops, 1);
 if (ret) return ret;
 value = 0;
 ret = x21_timing_sensor(ops, 0x301c, &value, 1, 1);
 if (ret) goto resume;
 for (i = 0; i < sizeof(x21_hdr_rows) / sizeof(x21_hdr_rows[0]); i++) {
  ret = x21_hdr_sensor_write(ops, x21_hdr_rows[i][0], x21_hdr_rows[i][1]);
  if (ret) goto resume;
 }
 for (i = 0; i < sizeof(geometry) / sizeof(geometry[0]); i++) {
  ret = x21_hdr_sensor_write(ops, geometry[i][0], geometry[i][1]);
  if (ret) goto resume;
 }
resume:
 value = 1;
 resume_ret = x21_timing_sensor(ops, 0x301c, &value, 1, 1);
 if (!ret) ret = resume_ret;
 resume_ret = x21_hdr_stall(ops, 0);
 if (!ret) ret = resume_ret;
 if (ret) return ret;
 ops->delay(ops->context, 1000);
 return x21_timing_expect(ops, 0x0006, 0);
}
#endif
