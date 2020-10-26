#include "sys.h"
#include "usart.h"
#include "usart2.h"	
#include "valuepack.h"
#include "led.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	   


//

u8 beep_flag;
u16 volt_init_flag;//��������ʼ������
u32 cnt_time,cnt_time_last,cnt_time2,cnt_time_last2;//������ʱ��
s32 volt[5],*p_volt,volt_diff[5],*p_volt_diff,volt_diff_diff[20],*p_volt_diff_diff,volt_diff_diff_max[20],*p_volt_diff_diff_max;
u16 volt_cnt,volt_cnt2;//cnt_flag������ʶ�𵽷����Ժ��漸�Σ�volt_cnt��������������
s32 diff_threshold=0,volt_max=0x80000000,volt_min=0x7fffffff;;//��ֵ����ֵ�����ֵ����Сֵ
float bmp2[12]={80},*p_bmp2,mean_bmp2;
u8 mean_cnt;
u16 bmp=80;
u8 bmp_updata;

//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������ 
void usart2_init(u32 pclk1,u32 bound)
{  	 
  
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��

	//USART1_TX   PA.2 PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //PA2,PA3,���ù���,�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù��� 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA2��PA3
	
 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
  
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_Mode =   USART_Mode_Rx | USART_Mode_Tx;	// ��ģʽ
    USART_Init(USART2, &USART_InitStructure); //��ʼ������
    
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
    
    USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);  //ʹ�ܴ��� 
    
//    p_volt = volt;  //ָ��ָ����
//    p_volt_diff = volt_diff;
//    p_volt_diff_diff = volt_diff_diff;
//    p_volt_diff_diff_max = volt_diff_diff_max;
//    p_bmp2 = bmp2;
}



s32 receive_volt = 0;//�յ���ֵ
u8 receive_cnt;
u8 receive_buff[10];  //uart_rx���ݻ�����

void USART2_IRQHandler(void)			   //����4ȫ���жϷ�����
{
  	//�����ж�
	if( USART_GetITStatus(USART2,USART_IT_RXNE)==SET )
	{
//        LED0=!LED0;//DS1��ת
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//����жϱ�־
        receive_buff[receive_cnt] = USART_ReceiveData(USART2);
        if(receive_cnt==0 && receive_buff[0]!=0xA5){return;}
        receive_cnt++;
        if(receive_cnt==6)
        {
            receive_cnt=0;
            if(receive_buff[5]==0x5A)
            {
                receive_volt = get_volt(receive_buff[1]<<16 | receive_buff[2]<<8 | receive_buff[3]);//��24λת��Ϊs32
                
                cnt_time++;//���ʱ��++
                
                if(volt_init_flag<5)
                {
                    volt[volt_init_flag] = receive_volt;   //��5������
                    volt_init_flag++;
                }
                
                else if(volt_init_flag<10)
                {
                    volt[volt_cnt] = receive_volt;
                    volt_diff[volt_cnt] = cnt_diff(volt);    //��5������
                    volt_cnt++;
                    if(volt_cnt==5) volt_cnt = 0;
                    volt_init_flag++;
                }
                else if(volt_init_flag<30)
                {
                    volt[volt_cnt] = receive_volt;
                    volt_diff[volt_cnt] = cnt_diff(volt);  
                    volt_diff_diff[volt_cnt2] = cnt_diff_diff(volt_diff,volt_cnt);    //��20������
                    volt_cnt++;
                    if(volt_cnt==5) volt_cnt = 0;
                    volt_cnt2++;
                    if(volt_cnt2==20) volt_cnt2 = 0;
                    volt_init_flag++;
                }
                else if(volt_init_flag<280)
                {
                    volt[volt_cnt] = receive_volt;
                    volt_diff[volt_cnt] = cnt_diff(volt);
                    volt_diff_diff[volt_cnt2] = cnt_diff_diff(volt_diff,volt_cnt);
                    if(volt_diff_diff[volt_cnt2]>diff_threshold){diff_threshold=volt_diff_diff[volt_cnt2];}
                    
                    volt_cnt++;
                    if(volt_cnt==5) volt_cnt = 0;
                    volt_cnt2++;
                    if(volt_cnt2==20) volt_cnt2 = 0;
                    volt_init_flag++;
                }
                else
                {
                    if (volt_init_flag==280)
                    {
                        diff_threshold *= 0.5f;
//                        printf(" Threshold = %d ",diff_threshold);
                        volt_init_flag++;
                    }
                    volt[volt_cnt] = receive_volt;
                    volt_diff[volt_cnt] = cnt_diff(volt);
                    volt_diff_diff[volt_cnt2] = cnt_diff_diff(volt_diff,volt_cnt);
                    
                    if((volt_diff_diff[volt_cnt2]>diff_threshold)&&cnt_time>10)    //is bmp++
                    {
                        cnt_time2 = kalmanFilter(cnt_time2);//�������˲�
                        bmp_updata = ~bmp_updata;
//                        bmp = (s16)(125.f/cnt_time*60.f);
                        bmp2[mean_cnt] = 1000.f/cnt_time2*60.f; 
                        if(cnt_time2<cnt_time_last2*0.8f||cnt_time2>cnt_time_last2*1.2f)
                        {beep_flag = 1;}
                        cnt_time_last2 = cnt_time2;
                        cnt_time_last = cnt_time;
//                        printf(" BMP:%d ",bmp);
                        uart2_tx_buf[0]=cnt_time2>>24;
                        uart2_tx_buf[1]=cnt_time2>>16;
                        uart2_tx_buf[2]=cnt_time2>>8;
                        uart2_tx_buf[3]=cnt_time2;
                        uart2_send_data(uart2_tx_buf,4);
                        
                        cnt_time = 0;
                        cnt_time2 = 0;
                        
                        mean_cnt++;
                        if(mean_cnt==12) mean_cnt = 0;
                    }
                    if(cnt_time<10)
                    {
                        volt_diff_diff_max[cnt_time] = volt_diff_diff[volt_cnt2];
                    }
                    else if(cnt_time==10)
                    {
                        diff_threshold = (s32)(cnt_diff_diff_max(volt_diff_diff_max) * 0.5f);
                    }
                    
                    
                    volt_cnt++;
                    if(volt_cnt==5) volt_cnt = 0;
                    volt_cnt2++;
                    if(volt_cnt2==20) volt_cnt2 = 0;
                }

           
                startValuePack(uart3_tx_buf);
                putBool(bmp_updata);
//                putShort(bmp);
                putInt(volt_diff_diff[volt_cnt2]);
                putInt(volt[volt_cnt]);
                putInt(diff_threshold);
                mean_bmp2 = mean_filter(bmp2);
                putFloat(mean_bmp2);
                endValuePack();
                uart3_send_data(uart3_tx_buf,20);
        
            }
        }
    }
}


s32 cnt_diff(s32 *p)    //ȡ�����е����ֵ-��Сֵ
{
    volt_max=0x80000000,volt_min=0x7fffffff;
    for(int i=0;i<5;i++)
    {
        if(p[i]>volt_max)
        {
            volt_max = p[i];
        }
        if(p[i]<volt_min)
        {
            volt_min = p[i];
        }
    }
    return volt_max-volt_min;
}

s32 cnt_diff_diff(s32 *p,u16 cnt)    //ȡǰ��5��ֵ-����ֵ
{
    return p[((cnt+1)==5?0:cnt+1)]-p[cnt];
}

s32 cnt_diff_diff_max(s32 *p)    //ȡ20��������ֵ
{
    s32 cnt_max=0x80000000;
    for(int i=0;i<20;i++)
    {
        if(p[i]>cnt_max)
        {
            cnt_max = p[i];
        }
    }
    return cnt_max;
}


float mean_filter(float *p)
{
    float sum = 0;
    for(int i=0;i<12;i++)
    {
        sum += p[i];
    }
    return sum/12.0;
}


float kalmanFilter(float inData) 
{
    static float prevData=0; 
    static float kalman_p=10, kalman_q=0.02, kalman_r=0.5, kGain=0;
    
    kalman_p = kalman_p+kalman_q; 
    kGain = kalman_p/(kalman_p+kalman_r);

    inData = prevData+(kGain*(inData-prevData)); 
    kalman_p = (1-kGain)*kalman_p;

    prevData = inData;

    return inData; 
}


/*
 * @description		: uart2 IRQHandler
 * @param - data  : ָ�����ݻ�����
 * @param - len   : ���ݳ��� ��λ�ֽ�
 * @return 				: void
 */
void uart2_send_data(const uint8_t *data, uint32_t len)
{
	uint32_t i;
	for (i = 0; i < len; i++) {
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		USART2->DR = (data[i] & (uint16_t)0x01FF);
	}
}


/*���ܣ��Ѳɵ���3���ֽ�ת���з���32λ�� */
s32 get_volt(u32 num)
{		
    s32 temp;
    temp = num;
    temp <<= 8;
    temp >>= 8;
    return temp;
}
