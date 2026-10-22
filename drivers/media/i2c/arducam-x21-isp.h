/* SPDX-License-Identifier: GPL-2.0 */
#ifndef ARDUCAM_X21_ISP_H
#define ARDUCAM_X21_ISP_H
#include "arducam-x21-timing.h"
/* ISP image controls and ARDUCAM_OPENHD manual exposure/gain through ISP SIP-DMA.
 * No flash, geometry, PLL, stream-enable or sensor-mode writes occur here.
 */
enum x21_isp_control { X21_EXPOSURE_US, X21_GAIN, X21_WB_AUTO, X21_WB_K,
 X21_BRIGHTNESS, X21_CONTRAST, X21_SATURATION, X21_GAMMA, X21_AE_AUTO,
 X21_AE_SCENE, X21_AE_BIAS,
 X21_ISP_COUNT };
static int x21_isp_apply(const struct x21_timing_ops *ops, unsigned int id, int v)
{
 unsigned int value; int ret;
 switch (id) {
 case X21_AE_BIAS:
  /* Standard V4L2 bias menu: -2..+2 EV in quarter-stop steps, neutral 8.
   * ISP AE_BV_OFF is signed Q8.8. This changes the native AE target,
   * without rewriting AE_CTRL or reloading the sensor timing. */
  if (v < 0 || v > 16) return -ERANGE;
  return ops->write(ops->context, 0x5014, ((v - 8) * 64) & 0xffff);
 case X21_AE_AUTO:
  if (v != 0 && v != 1) return -ERANGE;
  ret = ops->read(ops->context, 0x5002, &value);
  if (ret) return ret;
  /* An unchanged-mode write can still reload this bootdata's sensor mode. */
  if ((value & 15) == (v ? 12 : 0)) return 0;
  /* IMM and IMM1 can reload the vendor sensor timing when switching modes.
   * The native automatic loop runs with both clear; manual shutter/gain are
   * written directly through SIP-DMA and need neither immediate flag. */
  {
   unsigned int desired = (value & ~0x041fU) | (v ? 12 : 0);
   return ops->write(ops->context, 0x5002, desired);
  }
 case X21_EXPOSURE_US:
 case X21_GAIN:
  /* AE IMM causes this bootdata to restore its vendor sensor readout.
   * Use the sensor helpers below for manual control, even at lower FPS. */
  return -EOPNOTSUPP;
 case X21_WB_AUTO:
 case X21_WB_K:
  if (id == X21_WB_K) {
   if (v < 2000 || v > 10000) return -ERANGE;
   ret = ops->write(ops->context, 0x510a, v);
   if (ret) return ret;
  } else if (v != 0 && v != 1) return -ERANGE;
  ret = ops->read(ops->context, 0x5100, &value);
  if (ret) return ret;
  /* Mode 7 = manual; mode 15 = auto. Retain all unrelated AWB options. */
  value = (value & 0xfff0) | ((id == X21_WB_AUTO && v) ? 15 : 7) | 0x20;
  return ops->write(ops->context, 0x5100, value);
 case X21_BRIGHTNESS:
  if (v < -4096 || v > 4096) return -ERANGE;
  return ops->write(ops->context, 0x7000, v & 0xffff);
 case X21_CONTRAST:
 case X21_SATURATION:
  if (v < 0 || v > 8192) return -ERANGE;
  return ops->write(ops->context, id == X21_CONTRAST ? 0x7002 : 0x7006, v);
 case X21_GAMMA:
  if (v < 0 || v > 8192) return -ERANGE;
  return ops->write(ops->context, 0x700a, v);
 default: return -EINVAL;
 }
}

/* A scene is a small, reversible live transaction. Preserve the bootdata's
 * ordinary AE limits and all unrelated flicker flags. No AE_CTRL, timing,
 * geometry, PLL or nonvolatile operations belong in a profile change. */
struct x21_ae_scene_state {
 unsigned int baseline[6];
 int valid;
};
static int x21_ae_scene_apply(const struct x21_timing_ops *ops,
                            struct x21_ae_scene_state *state,
                            unsigned int scene, unsigned int fps)
{
 static const unsigned short regs[] = {0x5454,0x2024,0x2026,0x2028,0x202a,0x5440};
 unsigned int old[6], desired[6], i, j, cap, limit;
 int ret;
 /* Standard V4L2 scenes NONE=0, BACKLIGHT=1, SPORTS=11. OpenHD's Fly
  * profile uses BACKLIGHT plus image-based lower-region AE compensation. */
 if ((scene != 0 && scene != 1 && scene != 11) || !fps || fps > 120)
  return -ERANGE;
 for (i=0;i<6;i++) {
  ret=ops->read(ops->context,regs[i],&old[i]);
  if(ret)return ret;
  if(!state->valid)state->baseline[i]=old[i];
  desired[i]=state->baseline[i];
 }
 state->valid=1;
 desired[0]=(old[0]&~255U)|(scene==11?3:scene==1?7:state->baseline[0]&255U);
 desired[5]=(old[5]&~0x18U)|(state->baseline[5]&0x18U);
 if(scene) {
  limit=scene==11?2000:4000;
  cap=1000000U/fps;
  if(cap>limit)cap=limit;
  desired[1]=desired[3]=0;
  desired[2]=desired[4]=cap;
  desired[5]|=0x18;
 }
 if(fps>60)desired[5]&=~7U;
 for(i=0;i<6;i++) {
  if(old[i]==desired[i])continue;
  ret=ops->write(ops->context,regs[i],desired[i]);
  if(ret) {
   /* Include the failed register: a transport error need not mean the ISP
    * rejected its data. Best-effort rollback, retain the original error. */
   for(j=0;j<=i;j++)
    if(old[j]!=desired[j])ops->write(ops->context,regs[j],old[j]);
   return ret;
  }
 }
 return 0;
}

/* This module's measured PLL is 24 MHz * 192 / (2 * 2 * 7).
 * Verify the dividers rather than applying an exposure conversion to an
 * unknown clock. 1164 * 1180 clocks then predicts 119.816 fps on the bench.
 * The camera's ISP oscillator is a separate 48 MHz clock.
 */
static int x21_sensor_manual_exposure(const struct x21_timing_ops *ops,
                                      unsigned int us, int hdr)
{
 static const unsigned short pll[][2] = {
  {0x302a,7},{0x302c,2},{0x302e,2},{0x3030,192},
 };
 unsigned int i, value, line, frame, coarse;
 int ret;
 if (us < 100 || us > 4000) return -ERANGE;
 for (i=0; i<sizeof(pll)/sizeof(pll[0]); i++) {
  ret=x21_timing_sensor(ops,pll[i][0],&value,0,0);
  if (ret) return ret;
  /* The MCU may report output geometry before sensor initialization finishes.
   * An uninitialized divider is transient; an unknown nonzero PLL is not. */
  if (!value) return -EAGAIN;
  if (value != pll[i][1]) return -EOPNOTSUPP;
 }
 ret=x21_timing_sensor(ops,0x300c,&line,0,0);
 if (ret) return ret;
 ret=x21_timing_sensor(ops,0x300a,&frame,0,0);
 if (ret) return ret;
 if (!line || line > 65535 || frame < 5) return -ERANGE;
 coarse=(us*164571U+line*500U)/(line*1000U);
 if (!coarse) coarse=1;
 if (coarse > frame-4) return -ERANGE;
 ret=x21_timing_sensor(ops,0x3012,&coarse,1,0);
 if (ret || !hdr) return ret;
 /* Retain the qualified 16:1:1/16 three-exposure ratios and frame length. */
 value=coarse/16; if (!value) value=1;
 ret=x21_timing_sensor(ops,0x3212,&value,1,0);
 if (ret) return ret;
 value=coarse/256; if (!value) value=1;
 return x21_timing_sensor(ops,0x3216,&value,1,0);
}

static int x21_sensor_manual_gain(const struct x21_timing_ops *ops,
                                  unsigned int gain)
{
 /* Nearest ARDUCAM_OPENHD 0.375 dB gain-table step to the requested U8.8 gain.
  * Midpoints between 256 * 10^(code * 0.375 / 20), codes 0..48.
  */
 static const unsigned short thresholds[] = {
  262,273,285,298,311,325,339,354,370,386,403,421,
  440,459,479,500,522,545,569,595,621,648,677,707,
  738,770,804,840,877,916,956,998,1042,1088,1136,1186,
  1239,1293,1350,1410,1472,1537,1605,1676,1750,1827,1907,1992,
 };
 unsigned int code=0;
 if (gain < 256 || gain > 2048) return -ERANGE;
 while (code < sizeof(thresholds)/sizeof(thresholds[0]) &&
        gain > thresholds[code]) code++;
 return x21_timing_sensor(ops,0x5900,&code,1,0);
}
#endif
