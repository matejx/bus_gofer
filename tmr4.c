#include <stm32f10x.h>

//-----------------------------------------------------------------------------
//  utility functions
//-----------------------------------------------------------------------------

uint32_t isqrt(uint32_t a)
{
	uint32_t x = 1;
	while( x*x <= a ) ++x;
	return x-1;
}

uint32_t bestdivs(uint32_t xy, uint32_t* bx, uint32_t* by, uint32_t miny, uint32_t maxy)
{
	uint32_t bd = -1;

	for( uint32_t x = 1; x <= isqrt(xy); ++x ) {
		uint32_t y = xy / x;

		if( y < miny ) continue;
		if( y > maxy ) continue;

		uint32_t d;
		if( x*y > xy ) {
			d = x*y - xy;
		} else {
			d = xy - x*y;
		}

		if( d < bd ) {
			bd = d;
			*bx = x;
			*by = y;
		}

		if( d == 0 ) break;
	}

	return bd;
}

void DDR(GPIO_TypeDef* port, uint16_t pin, GPIOMode_TypeDef mode)
{
	GPIO_InitTypeDef iotd;
	iotd.GPIO_Pin = pin;
	iotd.GPIO_Speed = GPIO_Speed_10MHz;
	iotd.GPIO_Mode = mode;
	GPIO_Init(port, &iotd);
}

//-----------------------------------------------------------------------------
//  Timer functions
//-----------------------------------------------------------------------------

uint8_t tmr4_pwm(uint32_t freq, float dc)
{
	if( freq == 0 ) return 1;
	if( freq > 1000000) return 1;
	if( dc <= 0 ) return 1;
	if( dc >= 1 ) return 1;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_Cmd(TIM4, DISABLE);

	uint32_t xy = SystemCoreClock / freq;
	uint32_t x,y;
	bestdivs(xy, &x, &y, 20, 0xffff);

	TIM_TimeBaseInitTypeDef tbis;
	tbis.TIM_Period = y;
	tbis.TIM_Prescaler = x-1;
	tbis.TIM_ClockDivision = TIM_CKD_DIV1;
	tbis.TIM_CounterMode = TIM_CounterMode_Up;
	tbis.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &tbis);

	// configure PB6 to AF (TIM4_CH1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	DDR(GPIOB, GPIO_Pin_6, GPIO_Mode_AF_PP);

	// configure OC on CH3
    TIM_OCInitTypeDef ocid;
	TIM_OCStructInit(&ocid);
    ocid.TIM_OCMode = TIM_OCMode_PWM1;
    ocid.TIM_Pulse = dc * y;
    ocid.TIM_OutputState = TIM_OutputState_Enable;
    ocid.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &ocid);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_Cmd(TIM4, ENABLE);

	return 0;
}
