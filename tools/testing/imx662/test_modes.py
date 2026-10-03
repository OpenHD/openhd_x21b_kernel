#!/usr/bin/env python3
"""Exercise the driver's actual mode tables and timing functions on a host.

Only kernel plumbing/control I/O is stubbed. This is not a hardware stream test.
Run with Python 3 and a C compiler (for example in WSL).
"""
from pathlib import Path
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[3]
    source = (root / "drivers/media/i2c/imx662.c").read_text()

    def section(start, end):
        return source[source.index(start):source.index(end, source.index(start))]

    mode_types = section("struct imx662_reg {", "/* IMX662 Register List */")
    tables = section("/* ADC/output depth", "/* Bayer orders:")
    lookup = section("static inline void get_mode_table", "/* Read registers")
    timing = section("static int imx662_g_frame_interval", "static int imx662_get_mbus_config")
    hdr_calc = section("static struct imx662_hdr_regs imx662_calc_hdrae", "static int imx662_apply_hdrae")
    hdr_types = section("struct imx662_hdr_regs {", "static struct imx662_hdr_regs imx662_calc_hdrae")
    prelude = r'''
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define ALIGN(x, a) (((x) + (a) - 1) & ~((u64)(a) - 1))
#define clamp_t(type, x, lo, hi) ((type)(x) < (lo) ? (lo) : ((type)(x) > (hi) ? (hi) : (type)(x)))
#define IMX662_PIXEL_RATE 74250000
#define IMX662_VMAX_MAX 0xffffe
#define IMX662_LINK_FREQ_297MHZ 0
#define IMX662_LINK_FREQ_360MHZ 1
#define IMAGE_PAD 0
#define NO_HDR 0
#define HDR_X2 5
#define IMX662_DOL2_RHS1 231U
#define IMX662_DOL2_SHR0_MIN 250U
#define IMX662_ANA_GAIN_MAX_HDR 80U
#define min_t(type, x, y) ((type)(x) < (type)(y) ? (type)(x) : (type)(y))
#define MEDIA_BUS_FMT_SRGGB10_1X10 0x300f
#define MEDIA_BUS_FMT_SGRBG10_1X10 0x300a
#define MEDIA_BUS_FMT_SGBRG10_1X10 0x300e
#define MEDIA_BUS_FMT_SBGGR10_1X10 0x3007
#define MEDIA_BUS_FMT_SRGGB12_1X12 0x3012
#define MEDIA_BUS_FMT_SGRBG12_1X12 0x3011
#define MEDIA_BUS_FMT_SGBRG12_1X12 0x3010
#define MEDIA_BUS_FMT_SBGGR12_1X12 0x3008
struct preisp_hdrae_exp_s {u32 middle_exp_reg, short_exp_reg, middle_gain_reg, short_gain_reg;};
struct v4l2_rect { int left, top, width, height; };
struct v4l2_fract { u32 numerator, denominator; };
struct v4l2_subdev_frame_interval { struct v4l2_fract interval; };
struct v4l2_subdev_frame_interval_enum {
    u32 index, pad, code, width, height, reserved[8];
    struct v4l2_fract interval;
};
struct v4l2_subdev_state {};
'''
    stubs = r'''
struct imx662;
struct v4l2_ctrl { struct imx662 *sensor; };
struct imx662 {
    const struct imx662_mode *mode;
    u32 HMAX, VMAX;
    int mutex;
    int hdr_enabled;
    struct v4l2_ctrl *vblank;
};
struct v4l2_subdev { struct imx662 *sensor; };
static struct imx662 *to_imx662(struct v4l2_subdev *sd) { return sd->sensor; }
static void mutex_lock(int *unused) { (void)unused; }
static void mutex_unlock(int *unused) { (void)unused; }
static u64 div64_u64(u64 a, u64 b) { return a / b; }
static int __v4l2_ctrl_s_ctrl(struct v4l2_ctrl *ctrl, u32 blank) {
    assert(!(blank & 1));
    ctrl->sensor->VMAX = ctrl->sensor->mode->height + blank;
    return 0;
}
'''
    checks = r'''
int main(void) {
    const u32 codes[] = { MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SRGGB12_1X12 };
    for (unsigned depth = 0; depth < ARRAY_SIZE(codes); depth++) {
        struct imx662 sensor = {0};
        struct v4l2_subdev sd = { .sensor = &sensor };
        struct v4l2_ctrl ctrl = { .sensor = &sensor };
        const struct imx662_mode *modes;
        unsigned count;
        sensor.vblank = &ctrl;
        get_mode_table(&sensor, codes[depth], &modes, &count);
        assert(count == 3);
        for (unsigned i = 0; i < count; i++) {
            const struct imx662_mode *mode = &modes[i];
            assert(mode->crop.width == mode->width && mode->crop.height == mode->height);
            assert(mode->height >= 752 && !(mode->height % 4));
            assert(!(mode->width % 16) && !(mode->crop.left % 2) && !(mode->crop.top % 4));
            assert(mode->crop.left + mode->width <= 1936);
            assert(mode->crop.top + mode->height <= 1100);
            assert(mode->min_VMAX >= mode->height + 70 && mode->min_VMAX >= 820);
            assert(!(mode->min_VMAX & 1));
            assert(mode->default_HMAX == (depth ? 990 : 660));
            assert(mode->link_freq_idx == (depth ? 0 : 1));
            assert(mode->reg_list.regs[1].address == 0x3022);
            assert(mode->reg_list.regs[1].val == depth);
            assert(mode->reg_list.regs[2].address == 0x3023);
            assert(mode->reg_list.regs[2].val == depth);
            sensor.mode = mode;
            sensor.HMAX = mode->default_HMAX;
            sensor.VMAX = mode->default_VMAX;
            struct v4l2_subdev_frame_interval_enum fie = {
                .index = depth * 3 + i,
            };
            assert(!imx662_enum_frame_interval(&sd, NULL, &fie));
            assert(fie.interval.numerator == sensor.HMAX * sensor.VMAX);
            assert(fie.interval.denominator == IMX662_PIXEL_RATE);
            for (unsigned fps = 1; fps <= 200; fps++) {
                struct v4l2_subdev_frame_interval fi = { .interval = {1, fps} };
                assert(!imx662_s_frame_interval(&sd, &fi));
                assert(!(sensor.VMAX & 1) && sensor.VMAX >= mode->min_VMAX);
                assert(sensor.VMAX <= IMX662_VMAX_MAX);
                assert((u64)fi.interval.numerator * fps >= fi.interval.denominator);
                struct v4l2_subdev_frame_interval actual;
                assert(!imx662_g_frame_interval(&sd, &actual));
                assert(actual.interval.numerator == fi.interval.numerator);
            }
            const u32 limits[][2] = { {UINT32_MAX, 1}, {1, UINT32_MAX}, {UINT32_MAX, UINT32_MAX} };
            for (unsigned k = 0; k < ARRAY_SIZE(limits); k++) {
                struct v4l2_subdev_frame_interval fi = { .interval = {limits[k][0], limits[k][1]} };
                assert(!imx662_s_frame_interval(&sd, &fi));
                assert(sensor.VMAX >= mode->min_VMAX && sensor.VMAX <= IMX662_VMAX_MAX);
            }
            struct v4l2_subdev_frame_interval invalid = { .interval = {0, 90} };
            assert(imx662_s_frame_interval(&sd, &invalid) == -EINVAL);
            printf("RAW%u %ux%u ceiling %.3f fps\n", depth ? 12 : 10,
                   mode->width, mode->height,
                   (double)IMX662_PIXEL_RATE / (mode->default_HMAX * mode->min_VMAX));
        }
        struct v4l2_subdev_frame_interval_enum illegal = { .index = 7 };
        assert(imx662_enum_frame_interval(&sd, NULL, &illegal) == -EINVAL);
    }
    assert(imx662_channel_vc(NO_HDR, 0) == 0);
    assert(imx662_channel_vc(NO_HDR, 1) == -EINVAL);
    assert(imx662_channel_vc(HDR_X2, 0) == 0);
    assert(imx662_channel_vc(HDR_X2, 1) == 1);
    assert(imx662_channel_vc(HDR_X2, 2) == -EINVAL);
    assert(imx662_channel_vc(HDR_X2, UINT32_MAX) == -EINVAL);
    struct imx662 sensor = {.mode = &modes_hdr[0], .hdr_enabled = 1, .HMAX = 660, .VMAX = 2500};
    struct v4l2_subdev sd = {.sensor = &sensor};
    struct v4l2_ctrl ctrl = {.sensor = &sensor};
    sensor.vblank = &ctrl;
    struct v4l2_subdev_frame_interval_enum hdr = {.index = 6};
    assert(!imx662_enum_frame_interval(&sd, NULL, &hdr));
    assert(hdr.reserved[0] == HDR_X2 && hdr.width == 1936 && hdr.height == 1100);
    assert(hdr.interval.denominator == hdr.interval.numerator * 45);
    for (u32 fps = 1; fps <= 200; fps++) {
        struct v4l2_subdev_frame_interval fi = {.interval = {1, fps}};
        assert(!imx662_s_frame_interval(&sd, &fi));
        assert(!(sensor.VMAX % 4) && sensor.VMAX >= 2500);
        for (u32 exp = 0; exp < 4000; exp++) {
            struct preisp_hdrae_exp_s ae = {.middle_exp_reg = exp, .short_exp_reg = exp,
                                           .middle_gain_reg = exp, .short_gain_reg = exp};
            struct imx662_hdr_regs regs = imx662_calc_hdrae(sensor.VMAX, &ae);
            assert(!(regs.shr0 % 2) && regs.shr0 >= 250 && regs.shr0 <= sensor.VMAX - 8);
            assert(regs.shr1 % 2 && regs.shr1 >= 5 && regs.shr1 <= 229);
            assert(regs.long_gain <= 80 && regs.short_gain <= 80);
        }
        struct preisp_hdrae_exp_s overflow = {.middle_exp_reg = UINT32_MAX, .short_exp_reg = UINT32_MAX,
                                             .middle_gain_reg = UINT32_MAX, .short_gain_reg = UINT32_MAX};
        struct imx662_hdr_regs regs = imx662_calc_hdrae(sensor.VMAX, &overflow);
        assert(regs.shr0 == 250 && regs.shr1 == 5 && regs.long_gain == 80 && regs.short_gain == 80);
    }
    puts("Full-frame RAW10 90 fps and DOL2 HDR 45 fps; HDR exposure boundaries passed");
    return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix="imx662-modes-") as temp:
        path = Path(temp)
        (path / "test.c").write_text(prelude + mode_types + tables + stubs + lookup + timing + hdr_types + hdr_calc + checks)
        subprocess.run(["cc", "-std=gnu11", "-Wall", "-Werror", str(path / "test.c"), "-o", str(path / "test")], check=True)
        subprocess.run([str(path / "test")], check=True)


if __name__ == "__main__":
    main()
