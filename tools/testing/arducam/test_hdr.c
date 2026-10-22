/* SPDX-License-Identifier: GPL-2.0 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../drivers/media/i2c/arducam-x21-hdr.h"
struct mock {
 unsigned int isp[65536], sensor[65536];
 unsigned int sensor_writes, resume_attempts, busy_reads;
 unsigned short fail_reg;
 int busy, failed, bad_readback, stall_timeout;
};
static struct mock m;
static int read_reg(void *p,unsigned short reg,unsigned int *value) {
 struct mock *s=p;
 if(reg==0x60ac && s->busy){s->busy_reads++;*value=1;return 0;}
 *value=s->isp[reg]; return 0;
}
static unsigned int word(struct mock *s,unsigned short reg) {
 return s->isp[reg]<<16 | s->isp[reg+2];
}
static int write_reg(void *p,unsigned short reg,unsigned int value) {
 struct mock *s=p;
 /* No PLL, host packet format, MCU or nonvolatile operations are permitted. */
 assert(reg==0x5440 || reg==0x2020 || reg==0x500e || reg==0x500c ||
        reg==0x5002 || reg==0x601a ||
        (reg>=0x60a0 && reg<=0x60ac));
 if(reg==0x601a) {
  if(!s->stall_timeout)s->isp[reg]^=0x200;
  return 0;
 }
 s->isp[reg]=value;
 if(reg==0x60ac) {
  unsigned int src=word(s,0x60a0),dst=word(s,0x60a4),adr;
  if(value==0x0302) {
   adr=dst&0xffff;
   if(adr==0x301c && (src>>16)==1)s->resume_attempts++;
   if(adr==s->fail_reg && !s->failed){s->failed=1;s->isp[0x60ac]=0;return -EIO;}
   s->sensor[adr]=src>>16; s->sensor_writes++;
  } else {
   assert(value==0x0032);
   adr=src&0xffff;
   s->isp[0x60a4]=s->sensor[adr] << ((src&(1u<<25))?0:8);
   if(s->bad_readback && adr==0x31ac)s->isp[0x60a4]^=1;
  }
  s->isp[0x60ac]=0;
 }
 return 0;
}
static void delay(void *p,unsigned int ms){(void)p;(void)ms;}
static struct x21_timing_ops ops={&m,read_reg,write_reg,delay};
static void reset(void) {
 memset(&m,0,sizeof(m));
 m.isp[0]=0x0265; m.isp[0x2000]=1280; m.isp[0x2002]=720;
 m.isp[0x1010]=0x100; m.isp[0x6034]=600; m.isp[0x5440]=0x3c0e;
 m.isp[0x2014]=2; m.isp[0x202e]=1; m.sensor[0x300e]=0x2303;
 m.isp[0x2030]=0x0034; m.sensor[0x3000]=0x0f56; m.sensor[0x301c]=1;
}
int main(void) {
 reset(); assert(!x21_timing_hdr30(&ops));
 assert(m.sensor[0x3082]==8 && m.sensor[0x31ac]==0x140c);
 assert(m.sensor[0x3402]==1920 && m.sensor[0x3404]==1080);
 assert(m.sensor[0x30a2]==3 && m.sensor[0x30a6]==3);
 assert(m.sensor[0x3012]==512 && m.sensor[0x3212]==32 && m.sensor[0x3216]==2);
 assert(m.sensor[0x301c]==1 && !(m.isp[0x601a]&0x200));
 assert(m.isp[0x2030]==0x34 && m.isp[0x2000]==1280 && m.isp[0x2002]==720);
 reset();m.isp[0x2000]=1920;m.isp[0x2002]=1080;assert(!x21_timing_hdr30(&ops));
 reset();m.sensor[0x300e]=0x2300;assert(x21_timing_hdr30(&ops)==-ENODEV);assert(!m.sensor_writes);
 reset();m.isp[0x202e]=0;assert(x21_timing_hdr30(&ops)==-ENODEV);assert(!m.sensor_writes);
 reset();m.fail_reg=0x31d0;assert(x21_timing_hdr30(&ops)==-EIO);
 assert(m.resume_attempts==1 && m.sensor[0x301c]==1 && !(m.isp[0x601a]&0x200));
 reset();m.stall_timeout=1;assert(x21_timing_hdr30(&ops)==-ETIMEDOUT);assert(!m.sensor_writes);
 reset();m.bad_readback=1;assert(x21_timing_hdr30(&ops)==-EIO);
 assert(m.sensor[0x301c]==1 && !(m.isp[0x601a]&0x200));
 puts("7 HDR mode, geometry, identity, stall and recovery checks passed");
 return 0;
}
