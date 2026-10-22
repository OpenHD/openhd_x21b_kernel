/* SPDX-License-Identifier: GPL-2.0 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../drivers/media/i2c/arducam-x21-isp.h"
static unsigned int ap[65536],sensor[65536],changed,writes;
static int fail;
static int rd(void *p,unsigned short r,unsigned int *v){
 (void)p;if(fail)return -EIO;*v=ap[r];return 0;
}
static unsigned int word(unsigned short r){return ap[r]<<16|ap[r+2];}
static int wr(void *p,unsigned short r,unsigned int v){
 unsigned int src,dst,reg;(void)p;if(fail)return -EIO;
 /* No ISP control, reset, PLL or stream-mode writes. Only sensor SIP DMA. */
 assert(r>=0x60a0 && r<=0x60ac);writes++;ap[r]=v;
 if(r!=0x60ac)return 0;
 src=word(0x60a0);dst=word(0x60a4);
 if(v==0x302){
  reg=dst&65535;
  assert(reg==0x3012 || reg==0x3212 || reg==0x3216 || reg==0x5900);
  sensor[reg]=src>>16;changed++;
 }else{assert(v==0x32);ap[0x60a4]=sensor[src&65535];}
 ap[r]=0;return 0;
}
static void delay(void *p,unsigned int ms){(void)p;(void)ms;}
static const struct x21_timing_ops ops={NULL,rd,wr,delay};
int main(void){
 sensor[0x302a]=7;sensor[0x302c]=2;sensor[0x302e]=2;sensor[0x3030]=192;
 sensor[0x300a]=1164;sensor[0x300c]=1180;sensor[0x3082]=1;
 assert(!x21_sensor_manual_exposure(&ops,2000,0));
 assert(sensor[0x3012]==279 && changed==1);
 assert(!x21_sensor_manual_exposure(&ops,4000,0) && sensor[0x3012]==558);
 assert(sensor[0x300a]==1164 && sensor[0x300c]==1180 && sensor[0x3082]==1);
 assert(!x21_sensor_manual_gain(&ops,256) && sensor[0x5900]==0);
 assert(!x21_sensor_manual_gain(&ops,512) && sensor[0x5900]==16);
 assert(!x21_sensor_manual_gain(&ops,2048) && sensor[0x5900]==48);
 sensor[0x300a]=2250;sensor[0x300c]=2440;sensor[0x3082]=8;
 assert(!x21_sensor_manual_exposure(&ops,2000,1));
 assert(sensor[0x3012]==135 && sensor[0x3212]==8 && sensor[0x3216]==1);
 assert(sensor[0x3082]==8 && sensor[0x300a]==2250 && sensor[0x300c]==2440);
 unsigned int before=changed;
 sensor[0x3030]=80;
 assert(x21_sensor_manual_exposure(&ops,2000,1)==-EOPNOTSUPP && changed==before);
 /* Startup geometry can be valid while the sensor still returns zero PLLs.
  * Every uninitialized divider must be retriable without writing the sensor. */
 sensor[0x3030]=192;
 const unsigned short pll_regs[]={0x302a,0x302c,0x302e,0x3030};
 for(unsigned int i=0;i<sizeof(pll_regs)/sizeof(pll_regs[0]);i++) {
  unsigned int saved=sensor[pll_regs[i]];
  sensor[pll_regs[i]]=0;
  assert(x21_sensor_manual_exposure(&ops,2000,1)==-EAGAIN && changed==before);
  sensor[pll_regs[i]]=saved;
 }
 before=writes;
 assert(x21_sensor_manual_exposure(&ops,0,0)==-ERANGE && writes==before);
 assert(x21_sensor_manual_gain(&ops,255)==-ERANGE && writes==before);
 fail=1;assert(x21_sensor_manual_gain(&ops,512)==-EIO);
 puts("Sensor IQ: real shutter units, quantized gain, HDR ratios, timing preservation and fault checks passed");
 return 0;
}
