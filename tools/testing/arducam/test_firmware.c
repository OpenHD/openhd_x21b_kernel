/* SPDX-License-Identifier: GPL-2.0 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../../drivers/media/i2c/arducam-x21-fw.h"
/* Synthetic protocol fixture, not executable camera firmware. */
#define X21_FW_SIZE 512u
#define X21_FW_CRC32 0x40367ab5u
static const unsigned char x21_arducam_firmware[512] = {0x58,0x12,0,0x20,9,0x38,0,8};

struct mock {
	unsigned int device, sensor, version, request, marker, new_marker;
	unsigned int reads, writes, entered, erased, bytes, blocks, polls, checksum;
	int failed_read, failed_write, failed_block, busy, reject, timeout, complete;
	int readiness_failures, startup_busy_reads;
};

static unsigned int crc(const unsigned char *p, unsigned int size)
{
	unsigned int c = ~0u, i, bit;
	for (i = 0; i < size; i++) {
		c ^= p[i];
		for (bit = 0; bit < 8; bit++)
			c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1)));
	}
	return c ^ ~0u;
}

static int read_reg(void *context, unsigned short reg, unsigned int *value)
{
	struct mock *m = context;
	unsigned int marker = m->complete ? m->new_marker : m->marker;
	if (++m->reads == (unsigned int)m->failed_read)
		return -EIO;
	if (m->complete && m->readiness_failures > 0 && reg == 0x0103) {
		m->readiness_failures--;
		return -EIO;
	}
	switch (reg) {
	case 0x0103: *value = m->device; break;
	case 0x0105: *value = m->sensor; break;
	case 0x0101: *value = m->version; break;
	case 0x0107:
		*value = m->busy || m->startup_busy_reads > 0;
		if (m->startup_busy_reads > 0) m->startup_busy_reads--;
		break;
	case 0x0501:
		if (marker == 3) { *value = 0xfffe; break; }
		*value = m->request << 16;
		if (marker && m->request == X21_FW_MAGIC_REG)
			*value |= X21_FW_MAGIC;
		if (marker && m->request == X21_FW_REV_REG)
			*value |= marker == 1 ? X21_FW_REV : marker == 4 ? 0x0101 :
				marker == 5 ? 0x0102 : marker == 6 ? 0x01e1 : X21_FW_REV + 1;
		break;
	case 0x5454:
		m->polls++;
		*value = m->reject ? 2 : m->timeout ? 1 : 0;
		if (!*value) m->complete = 1;
		break;
	default: assert(!"unexpected read");
	}
	return 0;
}

static int write_reg(void *context, unsigned short reg, unsigned int value)
{
	struct mock *m = context;
	if (++m->writes == (unsigned int)m->failed_write)
		return -EIO;
	switch (reg) {
	case 0x0501: m->request = value >> 16; break;
	case 0x5757: assert(value == 1); m->entered++; break;
	case 0x5656: m->checksum = value; break;
	case 0x5454: assert(value == 1); m->erased++; break;
	default: assert(!"unexpected write");
	}
	return 0;
}

static int block(void *context, const unsigned char *data, unsigned int size)
{
	struct mock *m = context;
	assert(size > 2 && size <= 66 && !((size - 2) & 3));
	assert(data[0] == 0x55 && data[1] == 0x55);
	if (++m->blocks == (unsigned int)m->failed_block)
		return -EIO;
	assert(m->bytes + size - 2 <= X21_FW_SIZE);
	assert(!memcmp(data + 2, x21_arducam_firmware + m->bytes, size - 2));
	m->bytes += size - 2;
	return 0;
}

static void delay(void *context, unsigned int milliseconds)
{
	(void)context;
	assert(milliseconds <= 5000);
}

static struct mock fresh(void)
{
	struct mock m = {0};
	m.device = 0x30;
	m.sensor = 0x1302;
	m.version = 0x10004;
	m.new_marker = 1;
	return m;
}

static int ensure(struct mock *m, const unsigned char *image,
		   unsigned int size, unsigned int checksum, int selected)
{
	struct x21_fw_ops ops = {.context=m, .read=read_reg, .write=write_reg,
		.block=block, .delay=delay, .crc=crc};
	return x21_fw_ensure(&ops, image, size, checksum, selected);
}

#define RUN(m) ensure(&(m), x21_arducam_firmware, X21_FW_SIZE, X21_FW_CRC32, 1)
#define NO_ERASE(m) assert(!(m).entered && !(m).erased && !(m).blocks)

int main(void)
{
	struct mock m;
	unsigned char bad[X21_FW_SIZE];
	unsigned int sum = 0, i;
	assert(crc(x21_arducam_firmware, X21_FW_SIZE) == X21_FW_CRC32);
	m = fresh(); assert(ensure(&m, NULL, 0, 0, 0) == 0);
	assert(!m.reads && !m.writes);
	m = fresh(); assert(ensure(&m, x21_arducam_firmware, X21_FW_SIZE, 0, 1) == -EBADMSG);
	assert(!m.reads && !m.writes);
	for (i = 0; i < 8; i++) {
		m = fresh(); assert(ensure(&m, x21_arducam_firmware, i, 0, 1) == -EINVAL);
		assert(!m.reads && !m.writes);
	}
	m = fresh(); assert(ensure(&m, x21_arducam_firmware, X21_FW_SIZE - 1, 0, 1) == -EINVAL);
	assert(!m.reads && !m.writes);
	memcpy(bad, x21_arducam_firmware, sizeof(bad)); bad[0] = 1;
	m = fresh(); assert(ensure(&m, bad, sizeof(bad), crc(bad,sizeof(bad)), 1) == -EINVAL); NO_ERASE(m);
	memcpy(bad, x21_arducam_firmware, sizeof(bad)); bad[4] &= ~1;
	m = fresh(); assert(ensure(&m, bad, sizeof(bad), crc(bad,sizeof(bad)), 1) == -EINVAL); NO_ERASE(m);
	m = fresh(); m.device = 0x31; assert(RUN(m) == -ENODEV); NO_ERASE(m);
	m = fresh(); m.sensor = 0x1335; assert(RUN(m) == -ENODEV); NO_ERASE(m);
	m = fresh(); m.version++; assert(RUN(m) == -EPROTONOSUPPORT); NO_ERASE(m);
	m = fresh(); m.marker = 1; assert(RUN(m) == 0); NO_ERASE(m);
	m = fresh(); m.marker = 1; m.startup_busy_reads = 150;
	assert(RUN(m) == 0); assert(!m.startup_busy_reads); NO_ERASE(m);
	m = fresh(); m.marker = 2; assert(RUN(m) == -EPROTONOSUPPORT); NO_ERASE(m);
	m = fresh(); m.busy = 1; assert(RUN(m) == -ETIMEDOUT); NO_ERASE(m);
	for (i = 1; i <= 6; i++) {
		m = fresh(); m.failed_read = i; assert(RUN(m) == -EIO); NO_ERASE(m);
	}
	for (i = 1; i <= 4; i++) {
		m = fresh(); m.failed_write = i; assert(RUN(m) == -EIO); assert(!m.blocks);
	}
	m = fresh(); m.failed_block = 3; assert(RUN(m) == -EIO);
	assert(m.blocks == 3 && m.bytes == 128 && !m.polls);
	m = fresh(); m.reject = 1; assert(RUN(m) == -EBADMSG);
	m = fresh(); m.timeout = 1; assert(RUN(m) == -ETIMEDOUT); assert(m.polls == 2);
	m = fresh(); m.readiness_failures = 3; assert(RUN(m) == 1);
	assert(m.entered == 1 && m.readiness_failures == 0);
	m = fresh(); m.readiness_failures = 30; assert(RUN(m) == -EIO);
	assert(m.entered == 1 && m.readiness_failures == 10);
	m = fresh(); m.new_marker = 0; assert(RUN(m) == -EBADMSG);
	m = fresh(); m.marker = 4; assert(RUN(m) == 1);
	assert(m.entered == 1 && m.erased == 1);
	m = fresh(); m.marker = 5; assert(RUN(m) == 1);
	assert(m.entered == 1 && m.erased == 1);
	m = fresh(); m.marker = 6; assert(RUN(m) == 1);
	assert(m.entered == 1 && m.erased == 1);
	m = fresh(); m.marker = 3; assert(RUN(m) == 1);
	assert(m.entered == 1 && m.erased == 1 && m.bytes == X21_FW_SIZE);
	for (i = 0; i < X21_FW_SIZE; i++) sum += x21_arducam_firmware[i];
	assert(m.checksum == ~sum);
	m.marker = 1; m.entered = m.erased = m.blocks = 0;
	assert(RUN(m) == 0); NO_ERASE(m);
	puts("PASS: image validation, profile/identity gates, one-time update, transport faults, timeout and post-flash marker");
	return 0;
}
