#ifndef __USART2_H
#define __USART2_H 
#include "sys.h"
#include "stdio.h"	  
//////////////////////////////////////////////////////////////////////////////////	   
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//串口2初始化 
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/5/14
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 



extern u8 beep_flag;
extern u16 volt_init_flag;//测心跳初始化计数
extern u32 cnt_time,cnt_time_last,cnt_time2,cnt_time_last2;//计算间隔时间
extern s32 volt[5],*p_volt,volt_diff[5],*p_volt_diff,volt_diff_diff[20],*p_volt_diff_diff,volt_diff_diff_max[20],*p_volt_diff_diff_max;
extern u16 volt_cnt,volt_cnt2;//cnt_flag是用来识别到峰峰忽略后面几次，volt_cnt是用来遍历数组
extern s32 diff_threshold,volt_max,volt_min;;//阈值，差值，最大值，最小值
extern u16 bmp;
extern float bmp2[12];
extern u8 bmp_updata;

void usart2_init(u32 pclk1,u32 bound);
s32 cnt_diff(s32 *p);
s32 cnt_diff_diff(s32 *p,u16 cnt);
s32 cnt_diff_diff_max(s32 *p);
float mean_filter(float *p);

float kalmanFilter(float inData);
void uart2_send_data(const uint8_t *data, uint32_t len);
s32 get_volt(u32 num);

#endif	   
















