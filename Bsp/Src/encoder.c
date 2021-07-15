#include "encoder.h"

int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;
    switch (TIMX)
    {
    case 2:
        Encoder_TIM = (short)TIM2->CNT;
        TIM2->CNT = 0;
        break;
    case 3:
        Encoder_TIM = (short)TIM3->CNT;
        TIM3->CNT = 0;
        break;
    default:
        Encoder_TIM = 0;
    }
    Encoder_TIM = (Encoder_TIM * 11 / 17);

    return Encoder_TIM;
}
/**************************************************************************
函数功能：TIM3中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & 0X0001) //溢出中断
    {
    }
    TIM3->SR &= ~(1 << 0); //清除中断标志位
}
/**************************************************************************
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & 0X0001) //溢出中断
    {
    }
    TIM2->SR &= ~(1 << 0); //清除中断标志位
}
