#include "delay.h"

static uint8_t fac_us = 0;  // us延时倍乘数
static uint16_t fac_ms = 0; // ms延时倍乘数,在ucos下,代表每个节拍的ms数

void delay_init()
{
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK_DIV8);
    fac_us = SystemCoreClock / 8000000; //为系统时钟的1/8
    fac_ms = (uint16_t)fac_us * 1000;   //非OS下,代表每个ms需要的systick时钟数
}

void delay_us(uint32_t nus)
{
    SysTick->LOAD = nus * fac_us;             //时间加载
    SysTick->VAL = 0x00;                      //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //开始倒数
    while (!(SysTick->CTRL & (1 << 16)))
        ;
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //关闭计数器
    SysTick->VAL = 0X00;                       //清空计数器
}

void delay_ms(uint16_t nms)
{
    SysTick->LOAD = (uint32_t)nms * fac_ms;   //时间加载(SysTick->LOAD为24bit)
    SysTick->VAL = 0x00;                      //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //开始倒数
    while (!(SysTick->CTRL & (1 << 16)))
        ;
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //关闭计数器
    SysTick->VAL = 0X00;                       //清空计数器
}
