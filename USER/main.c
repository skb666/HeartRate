#include "sys.h"         //系统配置
#include "delay.h"       //延时
#include "usart.h"       //串口  		
#include "led.h"       
#include "ADS1292.h"
#include "Timer.h"
#include "dma.h"
#include "valuepack.h"

uint8_t uart2_tx_buf[50];
u8 data_to_send[60];//串口发送缓存
u16 volt_init_flag;//测心跳初始化计数
u32 cnt_time,cnt_time_last,cnt_time_last2,cnt_time3;//计算间隔时间
s32 volt[5],*p_volt,volt_diff[5],*p_volt_diff,volt_diff_diff[20],*p_volt_diff_diff,volt_diff_diff_max[20],*p_volt_diff_diff_max;
u16 volt_cnt,volt_cnt2;//cnt_flag是用来识别到峰峰忽略后面几次，volt_cnt是用来遍历数组
s32 volt_max=0x80000000,volt_min=0x7fffffff;;//阈值，差值，最大值，最小值
float bmp2[50]={80},*p_bmp2,mean_bmp2;
u8 mean_cnt;
u16 bmp=80;
u8 bmp_updata;
s32 receive_volt;


/*功能：把采到的3个字节转成有符号32位数 */
s32 get_volt(u32 num)
{		
    s32 temp;			
    temp = num;
    temp <<= 8;
    temp >>= 8;
    return temp;
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
    for(int i=0;i<30;i++)
    {
        sum += p[i];
    }
    return sum/30.0;
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



int main(void)
{
    SystemInit();	
    delay_init();	
    delay_ms(100);
    uart1_init(115200);//串口初始化为115200	
    uart2_init(115200);//串口初始化为115200		
    DMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)data_to_send);//串口1DMA设置
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //DMA	
    LED_Init();			

    PBout(10)=1;
    PBout(11)=0;
    ADS1292_Init();	//初始化ads1292		

    while(Set_ADS1292_Collect(0))//0 正常采集  //1 1mV1Hz内部侧试信号 //2 内部短接噪声测试
    {	
        printf("1292寄存器设置失败\r\n");
        delay_s(1);		
        DS3 =!DS3;	
        DS4 =!DS4;	     
    } 
    printf("寄存器设置成功\r\n");
    delay_s(1);		
    DS3 =LEDOFF;		
    DS4 =LEDOFF;

    TIM2_Init(10,7200);//系统指示   
    TIM4_Init(5000,7200);//系统指示     

    EXTI->IMR |= EXTI_Line8;//开DRDY中断			
    while(1)//循环发送数据		
    {				
        if(ads1292_recive_flag)
        {
            receive_volt = get_volt(ads1292_Cache[6]<<16 | ads1292_Cache[7]<<8 | ads1292_Cache[8]);//将24位转换为s32
                
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
                    diff_threshold = diff_threshold * percent /100.f;
//                        printf(" Threshold = %d ",diff_threshold);
                    volt_init_flag++;
                }
                volt[volt_cnt] = receive_volt;
                volt_diff[volt_cnt] = cnt_diff(volt);
                volt_diff_diff[volt_cnt2] = cnt_diff_diff(volt_diff,volt_cnt);
                
                if((volt_diff_diff[volt_cnt2]>diff_threshold)&&cnt_time>hl_num)    //is bmp++
                {
					cnt_time3 = cnt_time2;
                    cnt_time2 = kalmanFilter(cnt_time2);//卡尔曼滤波
                    bmp_updata = ~bmp_updata;
//                        bmp = (s16)(125.f/cnt_time*60.f);
             					
                    //检测心率失常
					if(test_flag ==1){
                    if(cnt_time3<cnt_time_last2*0.8f||cnt_time3>cnt_time_last2*1.2f){beep_flag = 1;}}
                    cnt_time_last2 = cnt_time3;
                    cnt_time_last = cnt_time;
					
					bmp2[mean_cnt] = 1000.f/cnt_time2*60.f; 
//                        printf(" BMP:%d ",bmp);
//                        uart2_tx_buf[0]=cnt_time2>>24;
//                        uart2_tx_buf[1]=cnt_time2>>16;
//                        uart2_tx_buf[2]=cnt_time2>>8;
//                        uart2_tx_buf[3]=cnt_time2;
//                        uart2_send_data(uart2_tx_buf,4);
                    
                    cnt_time = 0;
                    cnt_time2 = 0;
                    
                    mean_cnt++;
                    if(mean_cnt==30) mean_cnt = 0;
                }
                if(cnt_time<10)
                {
                    volt_diff_diff_max[cnt_time] = volt_diff_diff[volt_cnt2];
                }
                else if(cnt_time==10)
                {
                    diff_threshold = (s32)(cnt_diff_diff_max(volt_diff_diff_max) * percent /100.f);
                }
                
                
                volt_cnt++;
                if(volt_cnt==5) volt_cnt = 0;
                volt_cnt2++;
                if(volt_cnt2==20) volt_cnt2 = 0;
            }
            
            startValuePack(uart2_tx_buf);
            putBool(bmp_updata);
//                putShort(bmp);
            putInt(volt_diff_diff[volt_cnt2]);
            putInt(volt[volt_cnt]);
            putInt(diff_threshold);
            mean_bmp2 = mean_filter(bmp2);
            putFloat(mean_bmp2);
            endValuePack();
            USARTx_Send(USART2,uart2_tx_buf,20);
            ads1292_recive_flag = 0;

            startValuePack(data_to_send);
            putBool(beep_flag);
            putInt(volt_diff_diff[volt_cnt2]);
            putInt(diff_threshold);
            putInt(mean_bmp2*1000);
            endValuePack();
            DMA_Enable(DMA1_Channel4,16);//串口1DMA 
            ads1292_recive_flag=0;
        }
    }		
}







/**********************************************************************
编译结果里面的几个数据的意义：
Code：表示程序所占用 FLASH 的大小（FLASH）
RO-data：即 Read Only-data， 表示程序定义的常量，如 const 类型（FLASH）
RW-data：即 Read Write-data， 表示已被初始化的全局变量（SRAM）
ZI-data：即 Zero Init-data， 表示未被初始化的全局变量(SRAM)
***********************************************************************/
