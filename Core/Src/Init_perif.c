#include "main.h"

volatile uint32_t SysTimer_ms = 0; //Переменная, аналогичная HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //Счетчик для функции Delay_ms
volatile uint32_t counter_ms = 0; //Счетчик для функции pid
volatile uint32_t counter_ms1 = 0; //Счетчик для функции pid
extern struct _flags *point_flags;

void init_rcc(void) {
	SET_BIT(RCC->CR, RCC_CR_HSEON); //Запустим внешний кварцевый резонатор. Он у нас на 8 MHz.
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0); //Дождемся поднятия флага о готовности
	SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY);        // включим тактирование флеша
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);         // включим буфер предварительной выборки ??
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMUL, RCC_CFGR_PLLMUL6); // умножаем на 6 PLLMUL
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, RCC_CFGR_PPRE_DIV1); // APB1 делитель 1
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1); // AHB делитель 1
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC, RCC_CFGR_PLLSRC_HSE_PREDIV); // HSE как входящий для PLL
	SET_BIT(RCC->CR, RCC_CR_CSSON); //Включим CSS
	SET_BIT(RCC->CR, RCC_CR_PLLON); //Запустим PLL
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0); //Дожидемся поднятия флага включения PLL
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL); //Выберем PLL в качестве System Clock

	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN); //Включим тактирование порта А
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOBEN); //Включим тактирование порта В

}

void init_wdt(void) {
#ifndef DEBUG_MODE
	WRITE_REG(IWDG->KR, 0x5555);              // разрешим запись в регистры IWDG
	WRITE_REG(IWDG->PR, 0x111);               // запишем делитель
	WRITE_REG(IWDG->RLR, 0xFFF);              // запишем время до сброса
	WRITE_REG(IWDG->KR, 0xCCCC);              // запустим ватч дог
	while (IWDG->SR);                         // дождемся установки
	WRITE_REG(IWDG->KR, 0xAAAA);              // сброс ватч дога
#endif
}

void iwdt_reset(void) {
	WRITE_REG(IWDG->KR, 0xAAAA);              // сброс ватч дога
}

void CMSIS_SysTick_Timer_init(void) {
	/* п. 4.5.1 SysTick control and status register (STK_CTRL) (стр. 151)*/
	CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Выключим таймер для проведения настроек.
	SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk); //Разрешим прерывания по таймеру
	SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk); //Выберем без делителя. Будет 48MHz
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk,
			47999 << SysTick_LOAD_RELOAD_Pos); //Настроим прерывание на частоту в 1 кГц(т.е. сработка будет каждую мс)
	MODIFY_REG(SysTick->VAL, SysTick_VAL_CURRENT_Msk,
			47999 << SysTick_VAL_CURRENT_Pos); //Начнем считать с 47999
	SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //Запускаем таймер
}

void SysTick_Handler(void) {
	SysTimer_ms++;
	if (Delay_counter_ms) {
		Delay_counter_ms--;
	}

	if (counter_ms < 20)
		counter_ms++;
	else {
		point_flags->pid_ok = TRUE;
		counter_ms = 0;
	}

	if (point_flags->iwdt_res) {
		WRITE_REG(IWDG->KR, 0xAAAA);        // сброс ватч дога
	}

	if (point_flags->delay > 0) { // тут бы в дальнейшем переделать на другой таймер
		if (point_flags->delay == 5) {
			STEP3_OFF;
		}
		if (point_flags->delay == 4) {
			STEP3_ON;
		}
		if (point_flags->delay == 3) { // 5 вкл 3 раза, 3 вкл 2 раза, 1 вкл 1 раз
			STEP3_OFF;
		}
		if (point_flags->delay == 2) {
			STEP3_ON;
		}
		if (point_flags->delay == 1) {
			STEP3_OFF;
		}
		point_flags->delay--;
	}
}

void Delay_ms(uint32_t Milliseconds) {
	Delay_counter_ms = Milliseconds;
	while (Delay_counter_ms != 0)
		;
}

void init_tim3(void) {
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);   // включим тактирование
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER6_1);        // альтернативная функция
	SET_BIT(GPIOA->AFR[0], 1 << GPIO_AFRL_AFRL6_Pos);      // TIM3
	TIM3->PSC = 0;
	TIM3->ARR = 600;
	TIM3->CCR1 = 200;
	SET_BIT(TIM3->CR1, TIM_CR1_ARPE);    // включить автоматическую перезагрузку
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE); // загрузка значений в регистр сравнения немедленно
	SET_BIT(TIM3->CCMR1, 0b110 << TIM_CCMR1_OC1M_Pos); // шим режим 1 (счет вперед)
	SET_BIT(TIM3->CCER, TIM_CCER_CC1E);           // Сравнение 1 выхода включен.
	SET_BIT(TIM3->CR1, TIM_CR1_CEN);                       // включить таймер
}

void init_tim17(uint16_t prescaler, uint16_t reload) {
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM17EN);   // включим тактирование
	TIM17->PSC = prescaler - 1;
	TIM17->ARR = reload;
	SET_BIT(TIM17->CR1, TIM_CR1_ARPE);   // включить автоматическую перезагрузку
	SET_BIT(TIM17->CR1, TIM_CR1_CEN);             // включить таймер
	SET_BIT(TIM17->DIER, TIM_DIER_UIE);      // включим прерывание по обновлению

	NVIC_EnableIRQ(TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn, 2); // выставим приоритет
}

void init_tim16(uint16_t prescaler, uint16_t reload) {
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);   // включим тактирование
	TIM16->PSC = prescaler - 1;
	TIM16->ARR = reload;
	SET_BIT(TIM16->CR1, TIM_CR1_ARPE);   // включить автоматическую перезагрузку
	SET_BIT(TIM16->CR1, TIM_CR1_CEN);             // включить таймер
	//SET_BIT(TIM16->DIER, TIM_DIER_UIE);      // включим прерывание по обновлению

	NVIC_EnableIRQ(TIM16_IRQn);
	NVIC_SetPriority(TIM16_IRQn, 2); // выставим приоритет
}

void init_pins(void) {

	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER0_0);  // на выход А0
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR0);     // максимальная скорость

	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER1_0);  // на выход А1
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR1);     // максимальная скорость

	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER7_0);  // на выход А7
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR7);     // максимальная скорость

	SET_BIT(GPIOB->MODER, GPIO_MODER_MODER1_0);  // на выход В1
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR1);     // максимальная скорость

	SET_BIT(GPIOB->MODER, GPIO_MODER_MODER3_0);  // на выход В3
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR3);     // максимальная скорость

	SET_BIT(GPIOB->MODER, GPIO_MODER_MODER4_0);  // на выход В4
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR4);     // максимальная скорость

	SET_BIT(GPIOB->MODER, GPIO_MODER_MODER5_0);  // на выход В5
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR5);     // максимальная скорость

	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR6); // на вход В6
	SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEEDR7); // на вход В7

}

void init_debug_pin(void) {
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER13_Msk, GPIO_MODER_MODER13_0); // на выход А13
	CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_13);         // push_pull
	CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR13_Msk);     // без резисторов
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR13);    // максимальная скорость

	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER14_Msk, GPIO_MODER_MODER14_0); // на выход А14
	CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_14);         // push_pull
	CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR14_Msk);     // без резисторов
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEEDR14);    // максимальная скорость
}

