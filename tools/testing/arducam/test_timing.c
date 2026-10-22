/* SPDX-License-Identifier: GPL-2.0 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../drivers/media/i2c/arducam-x21-timing.h"
struct mock {
 unsigned int isp[65536], sensor[65536];
 unsigned int sensor_writes, resume_attempts, busy_reads;
 unsigned short fail_reg;
 int busy, failed, bad_readback;
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
        reg==0x5002 ||
        (reg>=0x60a0 && reg<=0x60ac));
 s->isp[reg]=value;
 if(reg==0x60ac) {
  unsigned int src=word(s,0x60a0),dst=word(s,0x60a4),adr;
  if(value==0x0302) {
   adr=dst&0xffff;
   assert(adr==0x301c || adr==0x3012 || adr==0x3082 || adr==0x30ba ||
          adr==0x31d0 || adr==0x31ac || adr==0x3036 || adr==0x300a || adr==0x300c);
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
 m.isp[0x2030]=0x0034; m.sensor[0x3000]=0x0f56; m.sensor[0x301c]=1;
}
int main(void) {
 reset(); assert(!x21_timing_720p120(&ops));
 assert(m.sensor[0x300a]==1164 && m.sensor[0x300c]==1180);
 assert(m.sensor[0x31ac]==0x0c0c && m.sensor[0x301c]==1);
 assert(m.isp[0x2030]==0x0034 && m.isp[0x5440]==0x3c08);
 assert(m.isp[0x2020]==0x7800 && m.isp[0x500e]==4000);
 reset(); m.isp[0x2000]=1920; assert(x21_timing_720p120(&ops)==-ENODEV); assert(!m.sensor_writes);
 reset(); m.isp[0x6034]=1000; assert(x21_timing_720p120(&ops)==-ENODEV); assert(!m.sensor_writes);
 reset(); m.sensor[0x3000]=0x1234; assert(x21_timing_720p120(&ops)==-ENODEV); assert(!m.sensor_writes);
 reset(); m.busy=1; assert(x21_timing_720p120(&ops)==-ETIMEDOUT); assert(m.busy_reads==30 && !m.sensor_writes);
 reset(); m.fail_reg=0x31ac; assert(x21_timing_720p120(&ops)==-EIO); assert(m.resume_attempts==1 && m.sensor[0x301c]==1);
 reset(); m.bad_readback=1; assert(x21_timing_720p120(&ops)==-EIO); assert(m.resume_attempts==1 && m.sensor[0x301c]==1);
 reset(); m.fail_reg=0x301c; assert(x21_timing_720p120(&ops)==-EIO); assert(m.resume_attempts==1 && m.sensor[0x301c]==1);
 reset();m.isp[0x0006]=15;
 assert(x21_timing_720p120(&ops)==-ENODEV);assert(!m.sensor_writes);
 puts("9 timing, identity, transfer, readback and recovery tests passed");
 return 0;
}
