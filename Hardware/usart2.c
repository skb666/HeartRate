#include "sys.h"
#include "usart.h"
#include "usart2.h"	
#include "valuepack.h"
#include "led.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	   


//

u8 beep_flag;
u16 volt_init_flag;//测心跳初始化计数
u32 cnt_time,cnt_time_last,cnt_time2,cnt_time_last2;//计算间隔时间
s32 volt[5],*p_volt,volt_diff[5],*p_volt_diff,volt_diff_diff[20],*p_volt_diff_diff,volt_diff_diff_max[20],*p_volt_diff_diff_max;
u16 volt_cnt,volt_cnt2;//cnt_flag是用来识别到峰峰忽略后面几次，volt_cnt是用来遍历数组
s32 diff_threshold=0,volt_max=0x80000000,volt_min=0x7fffffff;;//阈值，差值，最大值，最小值
float bmp2[12]={80},*p_bmp2,mean_bmp2;
u8 mean_cnt;
u16 bmp=80;
u8 bmp_updata;

//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率 
void usart2_init(u32 pclk1,u32 bound)
{  	 
  
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟

	//USART1_TX   PA.2 PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //PA2,PA3,复用功能,上拉输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3
	
 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
  
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_Mode =   USART_Mode_Rx | USART_Mode_Tx;	// 发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口
    
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
    
    USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);  //使能串口 
    
//    p_volt = volt;  //指针指数组
//    p_volt_diff = volt_diff;
//    p_volt_diff_diff = volt_diff_diff;
//    p_volt_diff_diff_max = volt_diff_diff_max;
//    p_bmp2 = bmp2;
}



s32 receive_volt = 0;//收到的值
u8 receive_cnt;
u8 receive_buff[10];  //uart_rx数据缓冲区

void USART2_IRQHandler(void)			   //串口4全局中断服务函数
{
  	//接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE)==SET )
	{
//        LED0=!LED0;//DS1翻转
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志
        receive_buff[receive_cnt] = USART_ReceiveData(USART2);
        if(receive_cnt==0 && receive_buff[0]!=0xA5){return;}
        receive_cnt++;
        if(receive_cnt==6)
        {
            receive_cnt=0;
            if(receive_buff[5]==0x5A)
            {
                receive_volt = get_volt(receive_buff[1]<<16 | receive_buff[2]<<8 | receive_buff[3]);//将24位转换为s32
                
                cnt_time++;//间隔时间++
                
                if(volt_init_flag<5)
                {
                    volt[volt_init_flag] = receive_volt;   //存5个缓存
                    volt_init_flag++;
                }
                
                else if(volt_init_flag<10)
                {
                    volt[volt_cnt] = receive_volt;
                    volt_diff[volt_cnt] = cnt_diff(volt);    //存5个缓存
                    volt_cnt++;
                    if(volt_cnt==5) volt_cnt = 0;
                    volt_init_flag++;
                }
                else if(volt_init_flag<30)
                {
                    volt[volt_cnt] = receive_volt;
                    volt_diff[volt_cnt] = cnt_diff(volt);  
                    volt_diff_diff[volt_cnt2] = cnt_diff_diff(volt_diff,volt_cnt);    //存20个缓存
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
                        cnt_time2 = kalmanFilter(cnt_time2);//卡尔曼滤波
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


s32 cnt_diff(s32 *p)    //取数组中的最大值-最小值
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

s32 cnt_diff_diff(s32 *p,u16 cnt)    //取前第5个值-最新值
{
    return p[((cnt+1)==5?0:cnt+1)]-p[cnt];
}

s32 cnt_diff_diff_max(s32 *p)    //取20个里的最大值
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
 * @param - data  : 指向数据缓存区
 * @param - len   : 数据长度 单位字节
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


/*功能：把采到的3个字节转成有符号32位数 */
s32 get_volt(u32 num)
{		
    s32 temp;
    temp = num;
    temp <<= 8;
    temp >>= 8;
    return temp;
}
