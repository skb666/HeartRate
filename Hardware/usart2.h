#ifndef __USART2_H
#define __USART2_H 
#include "sys.h"
#include "stdio.h"	  
//////////////////////////////////////////////////////////////////////////////////	   
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����2��ʼ�� 
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/5/14
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
////////////////////////////////////////////////////////////////////////////////// 



extern u8 beep_flag;
extern u16 volt_init_flag;//��������ʼ������
extern u32 cnt_time,cnt_time_last,cnt_time2,cnt_time_last2;//������ʱ��
extern s32 volt[5],*p_volt,volt_diff[5],*p_volt_diff,volt_diff_diff[20],*p_volt_diff_diff,volt_diff_diff_max[20],*p_volt_diff_diff_max;
extern u16 volt_cnt,volt_cnt2;//cnt_flag������ʶ�𵽷����Ժ��漸�Σ�volt_cnt��������������
extern s32 diff_threshold,volt_max,volt_min;;//��ֵ����ֵ�����ֵ����Сֵ
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
















