/* SPDX-License-Identifier: GPL-2.0 */
#ifndef ARDUCAM_X21_FW_H
#define ARDUCAM_X21_FW_H

/* Transport-independent protocol, also executed by the host fault tests. */
#ifdef __KERNEL__
#include <linux/errno.h>
#else
#include <errno.h>
#endif

struct x21_fw_ops {
	void *context;
	int (*read)(void *context, unsigned short reg, unsigned int *value);
	int (*write)(void *context, unsigned short reg, unsigned int value);
	int (*block)(void *context, const unsigned char *data, unsigned int size);
	void (*delay)(void *context, unsigned int milliseconds);
	unsigned int (*crc)(const unsigned char *data, unsigned int size);
	void (*progress)(void *context, const char *step);
};

#define X21_FW_MAGIC_REG 0xfff0
#define X21_FW_REV_REG 0xfff2
#define X21_FW_MAGIC 0x4f48
#define X21_FW_REV 0x0103

static unsigned int x21_fw_le32(const unsigned char *p)
{
	return (unsigned int)p[0] | ((unsigned int)p[1] << 8) |
	       ((unsigned int)p[2] << 16) | ((unsigned int)p[3] << 24);
}

static int x21_fw_validate(const struct x21_fw_ops *ops,
			   const unsigned char *data, unsigned int size,
			   unsigned int expected_crc)
{
	unsigned int stack, reset;

	if (size < 8 || size > 0x7800 || (size & 3))
		return -EINVAL;
	if (ops->crc(data, size) != expected_crc)
		return -EBADMSG;
	stack = x21_fw_le32(data);
	reset = x21_fw_le32(data + 4);
	if (stack < 0x20000000 || stack > 0x20002000 || (stack & 7) ||
	    !(reset & 1) || (reset & ~1u) < 0x08003800 ||
	    (reset & ~1u) >= 0x08003800 + size)
		return -EINVAL;
	return 0;
}

static int x21_fw_idle(const struct x21_fw_ops *ops)
{
	unsigned int value, attempt;
	int ret;

	/* A cold camera boot can still be loading the ISP when Linux probes.
	 * Keep this bounded and never treat a busy MCU as an unmarked image. */
	for (attempt = 0; attempt < 500; attempt++) {
		ret = ops->read(ops->context, 0x0107, &value);
		if (ret)
			return ret;
		if (!value)
			return 0;
		ops->delay(ops->context, 10);
	}
	return -ETIMEDOUT;
}

static int x21_fw_bridge_read(const struct x21_fw_ops *ops,
			      unsigned short reg, unsigned int *value)
{
	unsigned int response;
	int ret = x21_fw_idle(ops);

	if (ret)
		return ret;
	ret = ops->write(ops->context, 0x0501, (unsigned int)reg << 16);
	if (ret)
		return ret;
	ops->delay(ops->context, 30);
	ret = x21_fw_idle(ops);
	if (ret)
		return ret;
	ret = ops->read(ops->context, 0x0501, &response);
	if (ret)
		return ret;
	/* Stock MCU firmware cannot perform 16-bit ISP reads. */
	if ((response >> 16) != reg)
		return -EOPNOTSUPP;
	*value = response & 0xffff;
	return 0;
}

static int x21_fw_marker(const struct x21_fw_ops *ops)
{
	unsigned int magic, revision;
	int ret = x21_fw_bridge_read(ops, X21_FW_MAGIC_REG, &magic);

	if (ret == -EOPNOTSUPP)
		return 0;
	if (ret)
		return ret;
	if (magic != X21_FW_MAGIC)
		return 0;
	ret = x21_fw_bridge_read(ops, X21_FW_REV_REG, &revision);
	if (ret)
		return ret;
	/* Known OpenHD revisions and the successful context-disabled diagnostic. */
	if (revision == 0x0101 || revision == 0x0102 || revision == 0x01e1)
		return 0;
	/* Never downgrade an unrecognized OpenHD revision. */
	return revision == X21_FW_REV ? 1 : -EPROTONOSUPPORT;
}

static int x21_fw_identity(const struct x21_fw_ops *ops, unsigned int *version)
{
	unsigned int value;
	int ret = ops->read(ops->context, 0x0103, &value);

	if (ret)
		return ret;
	if (value != 0x30)
		return -ENODEV;
	ret = ops->read(ops->context, 0x0105, &value);
	if (ret)
		return ret;
	if (value != 0x1302)
		return -ENODEV;
	return ops->read(ops->context, 0x0101, version);
}

/* 0: already installed/profile not selected; 1: updated; negative: stop probing.
 * All validation/identity checks precede the bootloader/erase commands.
 * Call only at probe, before registering the camera with V4L2.
 */
static int x21_fw_ensure(const struct x21_fw_ops *ops,
			 const unsigned char *image, unsigned int size,
			 unsigned int expected_crc, int profile_selected)
{
	unsigned int version, checksum = 0, offset, count, value, attempt;
	unsigned char block[66];
	int ret;

	if (!profile_selected)
		return 0;
	ret = x21_fw_validate(ops, image, size, expected_crc);
	if (ret)
		return ret;
	ret = x21_fw_identity(ops, &version);
	if (ret)
		return ret;
	ret = x21_fw_marker(ops);
	if (ret)
		return ret < 0 ? ret : 0;
	/* This update was derived/tested only against vendor MCU version 1.0.4. */
	if (version != 0x00010004)
		return -EPROTONOSUPPORT;
	ret = x21_fw_idle(ops);
	if (ret)
		return ret;
	for (offset = 0; offset < size; offset++)
		checksum += image[offset];
	if (ops->progress)
		ops->progress(ops->context, "entering bootloader");
	ret = ops->write(ops->context, 0x5757, 1);
	if (ret)
		return ret;
	ops->delay(ops->context, 1000);
	ret = ops->write(ops->context, 0x5656, ~checksum);
	if (ret)
		return ret;
	ret = ops->write(ops->context, 0x5454, 1);
	if (ret)
		return ret;
	ops->delay(ops->context, 100);
	block[0] = 0x55;
	block[1] = 0x55;
	if (ops->progress)
		ops->progress(ops->context, "transferring application");
	for (offset = 0; offset < size; offset += count) {
		count = size - offset;
		if (count > 64)
			count = 64;
		for (attempt = 0; attempt < count; attempt++)
			block[attempt + 2] = image[offset + attempt];
		ret = ops->block(ops->context, block, count + 2);
		if (ret)
			return ret;
		ops->delay(ops->context, 10);
	}
	if (ops->progress)
		ops->progress(ops->context, "waiting for bootloader completion");
	/* Match the vendor updater: the first completion read is after 5 seconds.
	 * Earlier reads can see zero while the MCU is still restarting. */
	for (attempt = 0; attempt < 2; attempt++) {
		ops->delay(ops->context, 5000);
		ret = ops->read(ops->context, 0x5454, &value);
		if (ret)
			return ret;
		if (value == 2)
			return -EBADMSG;
		if (!value)
			break;
	}
	if (attempt == 2)
		return -ETIMEDOUT;
	if (ops->progress)
		ops->progress(ops->context, "verifying restarted application");
	for (attempt = 0; attempt < 20; attempt++) {
		ops->delay(ops->context, 100);
		ret = x21_fw_identity(ops, &version);
		if (!ret) {
			ret = x21_fw_marker(ops);
			if (ret == 1)
				return 1;
			if (!ret)
				ret = -EBADMSG;
		}
	}
	return ret;
}

#endif
