#include "Timer.h"
#include "led.h" 
#include "key.h"

u32 cnt_time2;
u8 beep_flag;
s32 diff_threshold=0,diff_threshold_last = 0;

//�߼� TIM1 TIM8
//ͨ�� TIM2 TIM3 TIM4 TIM5
//���� TIM6 TIM7

//��ʱ��5��ʼ��
void TIM2_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //TIM5ʱ��ʹ�� 
	 
		TIM_TimeBaseStructure.TIM_Period = arr-1;; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
		TIM_TimeBaseStructure.TIM_Prescaler =psc-1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�����жϷ���
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM5�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
		NVIC_Init(&NVIC_InitStructure); 
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //��������ж�����λ
		TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE );//TIM5 �������										
		TIM_Cmd(TIM2, ENABLE); 
}

void TIM2_IRQHandler(void)
{ 
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //�����ж� 
		{ 
				TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//���жϱ�־
				cnt_time2++;
		}				   
}




//��ʱ����ʼ��
//������ʱ�� +��������
void TIM4_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ�� 

    TIM_TimeBaseStructure.TIM_Period = arr-1;; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
    TIM_TimeBaseStructure.TIM_Prescaler =psc-1; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //�����жϷ���
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure); 

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //��������ж�����λ
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE );//�������										
    TIM_Cmd(TIM4, ENABLE); 

    //��������ʼ��A1
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//100MHz

    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
    
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}
u8 same_threshold_cnt;

//��ʱ���жϷ������	 
void TIM4_IRQHandler(void)
{ 
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //�����ж� 
    { 
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);//���жϱ�־		
        //����ʧ������
        if(beep_flag == 1) {GPIO_SetBits(GPIOA,GPIO_Pin_1);beep_flag = 0;}
        else{GPIO_ResetBits(GPIOA,GPIO_Pin_1);}
       
        if(diff_threshold == diff_threshold_last){same_threshold_cnt++;}
        else same_threshold_cnt = 0;
        diff_threshold_last = diff_threshold;
        
        if(same_threshold_cnt >3) diff_threshold = 0;                                                                                                          
    }
}


//		TIM3->CNT=0;//��ն�ʱ��������
//		TIM3->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
//		TIM3->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
//		TIM3->DIER|=1<<0;   //��������ж�	  
//		TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3





 