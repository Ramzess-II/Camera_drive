
#include "main.h"
//#define ADR  ((uint32_t)0x1FFF F7B8)
//----------------------- объявим внешние переменные --------------------------//
extern struct _flags *point_flags;

uint16_t adc_buf [2];
// hop hey

/*struct adc_dats {
	uint16_t adc_1;
	uint16_t adc_2;
};

struct adc_dats adc_dat;

uint16_t get_adc1 (void) {
	return adc_buf[1];
	return  adc_dat.adc_1;
}

uint16_t get_adc2 (void) {
	return  adc_dat.adc_2;
}*/

void ADC_init (void){
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER4);      // установить аналог мод
	//SET_BIT(GPIOA->MODER, GPIO_MODER_MODER5);    // установить аналог мод
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADCEN);      // включим тактирование
	SET_BIT(ADC1->SMPR, 0b011 << ADC_SMPR_SMP_Pos);// Количество циклов преобразования
	SET_BIT(ADC1->CHSELR, ADC_CHSELR_CHSEL4);      // выбор канала 4
	SET_BIT(ADC1->CHSELR, ADC_CHSELR_CHSEL5);      // выбор канала 5
//	SET_BIT(ADC1->CHSELR, ADC_CHSELR_CHSEL16);     // выбор канала temperature
//	SET_BIT(ADC->CCR, ADC_CCR_TSEN);               // температурный датчик включить
	SET_BIT(ADC1->CR, ADC_CR_ADCAL);               // Запустим калибровку
	while (READ_BIT(ADC1->CR, ADC_CR_ADCAL));      // Дождемся поднятия флага о готовности
	//SET_BIT(ADC1->IER, ADC_IER_EOCIE);           // включить прерывание
	SET_BIT(ADC1->CFGR1, ADC_CFGR1_CONT);          // постоянный режим преобразования
	//SET_BIT(ADC1->CFGR1, ADC_CFGR1_ALIGN);       // левое правое выравнивание
	SET_BIT(ADC1->CFGR1, ADC_CFGR1_DMAEN);         // ДМА ключить
	SET_BIT(ADC1->CFGR1, ADC_CFGR1_DMACFG);        // ДМА в круговом режиме
	SET_BIT(ADC1->CR, ADC_CR_ADEN);                // Включим АЦП
	SET_BIT(ADC1->CR, ADC_CR_ADSTART);             // Запустим преобразование
	//NVIC_EnableIRQ(ADC1_IRQn);
	//NVIC_SetPriority(ADC1_IRQn, 3);              // выставим приоритет

	//----------------------- DMA --------------------------//
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMAEN);                 // включим тактирование ДМА
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);            // адрес для переферии для считывания
	DMA1_Channel1->CMAR = (uint32_t)adc_buf;                // адрес памяти
	DMA1_Channel1->CNDTR = 2;                               // размер буффера приема
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_PL_1);              // максимальный приоритет
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_CIRC);              // циклический режим
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MSIZE_0);           // Размер памяти 16 бит
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_PSIZE_0);           // Размер переферии 16 бит
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_MINC);              // инкрементировать память
	//SET_BIT(DMA1_Channel1->CCR, DMA_CCR_TCIE);              // прерывание после передачи, а надо?
	//NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN);                // включить дма
}

/*void filtr_adc (void) {                                     // фильтр ацп (скользящее среднее) и только для холла
	static uint8_t count = 0;
	uint32_t temporary = 0;
	static uint16_t adc_buf1 [8];
	if (count < 8) {
		adc_buf1 [count] =  adc_buf [1];
		count ++;
	} else count = 0;

	for (int i = 0; i < 8; i ++){
		temporary += adc_buf1 [i];
	}
	adc_dat.adc_1 = temporary >> 3;                         // смещение в место деления
	adc_dat.adc_2 = adc_buf [0];
	temporary = 0;
	for (int i = 0; i < 8; i ++){
		temporary += adc_buf2 [i];
	}
	adc_dat.adc_2 = temporary >> 3;
}*/

/*void DMA1_Channel1_IRQHandler (void) {                   // само прерывание ДМА
	point_flags->dma_ok = TRUE;                          // поднимем флаг что можно фильтровать
	SET_BIT(DMA1->IFCR, DMA_IFCR_CTCIF1);                // Сбросим флаг прерывания
}*/

/*void ADC1_IRQHandler(void) {                              // прерывание от АЦП
	asm("NOP");
	adc_convert [adc] = ADC1->DR;
	 SET_BIT(ADC1->ISR, ADC_ISR_EOC); // сбросить прерывание
	adc ++;
	if (adc == 2) {
		adc = 0;
	}
}
*/

// в режиме непрерывного преобразования, когда несколько режимов включено, АЦП автоматом переключает каналы
