/* SPDX-License-Identifier: GPL-2.0 */
#ifndef ARDUCAM_X21_TIMING_H
#define ARDUCAM_X21_TIMING_H
#ifdef __KERNEL__
#include <linux/errno.h>
#else
#include <errno.h>
#endif

/* Volatile ISP/SIP-DMA operations. This interface cannot write MCU flash. */
struct x21_timing_ops {
 void *context;
 int (*read)(void *context, unsigned short reg, unsigned int *value);
 int (*write)(void *context, unsigned short reg, unsigned int value);
 void (*delay)(void *context, unsigned int milliseconds);
};

static int x21_timing_write32(const struct x21_timing_ops *ops,
                             unsigned short reg, unsigned int value)
{
 int ret = ops->write(ops->context, reg, value >> 16);
 return ret ? ret : ops->write(ops->context, reg + 2, value & 0xffff);
}

static int x21_timing_dma_idle(const struct x21_timing_ops *ops)
{
 unsigned int value, attempt;
 int ret;
 for (attempt = 0; attempt < 30; attempt++) {
  ret = ops->read(ops->context, 0x60ac, &value);
  if (ret) return ret;
  if (!(value & 7)) return 0;
  ops->delay(ops->context, 2);
 }
 return -ETIMEDOUT;
}

static int x21_timing_sensor(const struct x21_timing_ops *ops,
                             unsigned short reg, unsigned int *value,
                             int write, int byte)
{
 unsigned int readback;
 int ret = x21_timing_dma_idle(ops);
 if (ret) return ret;
 ret = x21_timing_write32(ops, 0x60a8, byte ? 1 : 2);
 if (ret) return ret;
 if (write) {
  ret = x21_timing_write32(ops, 0x60a0, (*value << 16) | 0x60a0);
  if (ret) return ret;
  ret = x21_timing_write32(ops, 0x60a4,
                          (byte ? 0x01200000 : 0x03200000) | reg);
 } else {
  ret = x21_timing_write32(ops, 0x60a0,
                          (byte ? 0x01200000 : 0x03200000) | reg);
  if (ret) return ret;
  ret = x21_timing_write32(ops, 0x60a4, 0x60a4);
 }
 if (ret) return ret;
 ret = ops->write(ops->context, 0x60ac, write ? 0x0302 : 0x0032);
 if (ret) return ret;
 ret = x21_timing_dma_idle(ops);
 if (ret) return ret;
 if (write) {
  ret = x21_timing_sensor(ops, reg, &readback, 0, byte);
  return ret ? ret : readback == *value ? 0 : -EIO;
 }
 ret = ops->read(ops->context, 0x60a4, &readback);
 if (!ret) *value = byte ? readback >> 8 : readback;
 return ret;
}

static int x21_timing_expect(const struct x21_timing_ops *ops,
                             unsigned short reg, unsigned int expected)
{
 unsigned int value;
 int ret = ops->read(ops->context, reg, &value);
 return ret ? ret : value == expected ? 0 : -ENODEV;
}

/* Apply only after the dedicated board/MCU identity was checked by the caller.
 * Keep the complete binned field of view; no zoom, PLL or packet format changes.
 * A small margin below 120 avoids overrunning ISP's 120-fps output limit.
 * Exposure is manual, 4 ms. Lower rates retain the ordinary firmware defaults.
 */
static int x21_timing_720p120(const struct x21_timing_ops *ops)
{
 static const unsigned short sensor[][2] = {
  {0x3012, 400}, {0x3082, 1}, {0x30ba, 0x0024}, {0x31d0, 0},
  {0x31ac, 0x0c0c}, {0x3036, 6}, {0x300a, 1152}, {0x300c, 1216},
 };
 static const unsigned short preview[][2] = {
  {0x2020, 0x7800}, {0x500e, 4000}, {0x500c, 0},
  {0x5002, 0x0290}, {0x5002, 0x0690},
 };
 unsigned int i, value, width, height;
 int ret, resume_ret;
 ret = x21_timing_expect(ops, 0x0000, 0x0265);
 if (ret) return ret;
 ret = x21_timing_expect(ops, 0x0006, 0);
 if (ret) return ret;
 ret = ops->read(ops->context, 0x2000, &width);
 if (ret) return ret;
 ret = ops->read(ops->context, 0x2002, &height);
 if (ret) return ret;
 if (width != 1280 || height != 720) return -ENODEV;
 ret = x21_timing_expect(ops, 0x1010, 0x0100);
 if (ret) return ret;
 ret = x21_timing_expect(ops, 0x6034, 600);
 if (ret) return ret;
 ret = x21_timing_sensor(ops, 0x3000, &value, 0, 0);
 if (ret) return ret;
 if (value != 0x0f56) return -ENODEV;
 ret = ops->read(ops->context, 0x5440, &value);
 if (ret) return ret;
 ret = ops->write(ops->context, 0x5440, value & 0xfff8);
 if (ret) return ret;
 for (i = 0; i < sizeof(preview) / sizeof(preview[0]); i++) {
  ret = ops->write(ops->context, preview[i][0], preview[i][1]);
  if (ret) return ret;
 }
 ops->delay(ops->context, 500);
 ret = x21_timing_expect(ops, 0x0006, 0);
 if (ret) return ret;
 value = 0;
 ret = x21_timing_sensor(ops, 0x301c, &value, 1, 1);
 if (ret) goto resume;
 for (i = 0; i < sizeof(sensor) / sizeof(sensor[0]); i++) {
  value = sensor[i][1];
  ret = x21_timing_sensor(ops, sensor[i][0], &value, 1, 0);
  if (ret) break;
 }
resume:
 /* Always attempt to restart the sensor if programming fails while stopped. */
 value = 1;
 resume_ret = x21_timing_sensor(ops, 0x301c, &value, 1, 1);
 if (ret) return ret;
 if (resume_ret) return resume_ret;
 /* Reacquire the linear stream before shortening its line time. */
 ops->delay(ops->context, 500);
 ret = x21_timing_expect(ops, 0x0006, 0);
 if (ret) return ret;
 value = 1164;
 ret = x21_timing_sensor(ops, 0x300a, &value, 1, 0);
 if (ret) return ret;
 value = 1180;
 ret = x21_timing_sensor(ops, 0x300c, &value, 1, 0);
 if (ret) return ret;
 ret = x21_timing_expect(ops, 0x2000, 1280);
 if (ret) return ret;
 ret = x21_timing_expect(ops, 0x2002, 720);
 return ret ? ret : x21_timing_expect(ops, 0x0006, 0);
}
#endif
