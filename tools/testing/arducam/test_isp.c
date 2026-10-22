/* SPDX-License-Identifier: GPL-2.0 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../drivers/media/i2c/arducam-x21-isp.h"
static unsigned int regs[65536], writes;
static int fail;
static unsigned int fail_once_reg;
static int rd(void *c,unsigned short a,unsigned int *v){(void)c;if(fail)return -EIO;*v=regs[a];return 0;}
static int wr(void *c,unsigned short a,unsigned int v){(void)c;if(fail)return -EIO;if(fail_once_reg==a){fail_once_reg=0;return -EIO;}regs[a]=v;writes++;return 0;}
static void delay(void *c,unsigned int ms){(void)c;(void)ms;}
int main(void){
 struct x21_timing_ops ops={NULL,rd,wr,delay};
 regs[0x5002]=0x0290;
 assert(x21_isp_apply(&ops,X21_EXPOSURE_US,1000)==-EOPNOTSUPP && !writes);
 assert(x21_isp_apply(&ops,X21_GAIN,256)==-EOPNOTSUPP && !writes);
 assert(!x21_isp_apply(&ops,X21_AE_AUTO,1) && regs[0x5002]==0x028c);
 assert(!x21_isp_apply(&ops,X21_AE_AUTO,0) && regs[0x5002]==0x0280);
 /* Preserve unrelated flags on a transition, but never rewrite an unchanged
  * AE mode: this bootdata can reload its sensor readout even without IMM. */
 regs[0x5002]=0xa5d0;
 assert(!x21_isp_apply(&ops,X21_AE_AUTO,1) && regs[0x5002]==0xa1cc);
 unsigned int unchanged=writes;
 assert(!x21_isp_apply(&ops,X21_AE_AUTO,1) && writes==unchanged);
 regs[0x5002]=0xa5dc;
 assert(!x21_isp_apply(&ops,X21_AE_AUTO,1) && writes==unchanged && regs[0x5002]==0xa5dc);
 regs[0x5100]=0x115f;assert(!x21_isp_apply(&ops,X21_WB_K,4500));
 assert(regs[0x510a]==4500 && (regs[0x5100]&15)==7);
 assert((regs[0x5100]&0xffc0)==(0x115f&0xffc0));
 assert(!x21_isp_apply(&ops,X21_WB_AUTO,1) && (regs[0x5100]&15)==15);
 assert(!x21_isp_apply(&ops,X21_BRIGHTNESS,-4096) && regs[0x7000]==0xf000);
 assert(!x21_isp_apply(&ops,X21_CONTRAST,4096) && regs[0x7002]==4096);
 assert(!x21_isp_apply(&ops,X21_SATURATION,2048) && regs[0x7006]==2048);
 unsigned int before=writes;
 assert(x21_isp_apply(&ops,X21_WB_K,10001)==-ERANGE);assert(writes==before);
 assert(x21_isp_apply(&ops,X21_AE_AUTO,2)==-ERANGE);assert(writes==before);
 assert(!x21_isp_apply(&ops,X21_AE_BIAS,12) && regs[0x5014]==256);
 assert(!x21_isp_apply(&ops,X21_AE_BIAS,4) && regs[0x5014]==0xff00);
 assert(!x21_isp_apply(&ops,X21_AE_BIAS,8) && regs[0x5014]==0);
 assert(x21_isp_apply(&ops,X21_AE_BIAS,17)==-ERANGE);
 {
  struct x21_ae_scene_state state={0};
  regs[0x5454]=0xab00;regs[0x2024]=0;regs[0x2026]=32910;
  regs[0x2028]=0;regs[0x202a]=8333;regs[0x5440]=0x3c08;
  before=writes;
  assert(!x21_ae_scene_apply(&ops,&state,0,120) && writes==before);
  assert(!x21_ae_scene_apply(&ops,&state,11,120));
  assert(regs[0x5454]==0xab03 && regs[0x2026]==2000 && regs[0x202a]==2000);
  assert(regs[0x5440]==0x3c18 && regs[0x5002]==0xa5dc);
  assert(!x21_ae_scene_apply(&ops,&state,1,120));
  assert(regs[0x5454]==0xab07 && regs[0x202a]==4000);
  assert(!x21_ae_scene_apply(&ops,&state,0,120));
  assert(regs[0x5454]==0xab00 && regs[0x2026]==32910 && regs[0x202a]==8333);
  assert(regs[0x5440]==0x3c08);
  before=writes;
  assert(x21_ae_scene_apply(&ops,&state,7,120)==-ERANGE && writes==before);
  fail_once_reg=0x2026;
  assert(x21_ae_scene_apply(&ops,&state,11,120)==-EIO);
  assert(regs[0x5454]==0xab00 && regs[0x2026]==32910 && regs[0x202a]==8333);
  assert(regs[0x5440]==0x3c08);
 }
 fail=1;assert(x21_isp_apply(&ops,X21_AE_AUTO,1)==-EIO);
 puts("ISP units, AE/AWB bit preservation, ranges and I2C fault checks passed");return 0;
}
