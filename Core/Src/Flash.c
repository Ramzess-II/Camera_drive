
#include "main.h"

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint32_t new_baud (uint32_t baud){
	if (baud == 9600 || baud == 19200 || baud == 38400 || baud == 57600 || baud == 115200) {
		return baud;
	} else return 0x00;
}

void new_baud_set (uint32_t baud){
		CLEAR_BIT(USART1->CR1, USART_CR1_UE);  // отключим USART
		set_baud(USART2, baud);
		SET_BIT(USART1->CR1, USART_CR1_UE);    // включим USART
}

uint32_t new_speed (uint32_t speed, uint32_t num_motor){     // новая скорость вращения.
	if (speed > 1000) return 0;                              // поиграться с этими параметрами
	if (num_motor == 0) {
		TIM17->ARR = map (speed, 0, 1000, 30000, 100);
	}
	if (num_motor == 1) {
		TIM16->ARR = map (speed, 0, 1000, 30000, 100);
	}
	return speed;
}

void init_flash_data (void) {                                // инициализация параметров из памяти
	TIM17->ARR = 10000;                     // скорость
	write_to_TMC2300 (1,0x6C, 0x3008001);   //  выберем шаг 32
	Delay_ms(2);
	write_to_TMC2300 (3,0x6C, 0x3008001);   //  выберем шаг 32
	Delay_ms(2);

}

/*void new_step_no_memory (void) {            // записать новые шаги, без изменения памяти
	write_to_TMC2300 (1,0x6C, 0x8008001);   //  выберем шаг 1
	Delay_ms(2);
	write_to_TMC2300 (3,0x6C, 0x8008001);   //  выберем шаг 1
	Delay_ms(2);
}*/

/*uint32_t verif_and_write (uint8_t adr, uint32_t data, uint8_t num_motor) {    // проверка правильные ли данные + запись по юарту в драйвер
	if (data == 1 || data == 2 || data == 4 || data == 8 || data == 16 || data == 32 || data == 64 || data == 128 || data == 256){
		switch (num_motor){
		case 1:
			flash.step_motor1 = data;
			break;
		case 2:
			flash.step_motor2 = data;
			break;
		case 3:
			break;
		}
		switch (data) {
		case 1:
			write_to_TMC2300 (adr,0x6C, 0x8008001);   //  выберем шаг
			break;
		case 2:
			write_to_TMC2300 (adr,0x6C, 0x7008001);   //  выберем шаг
			break;
		case 4:
			write_to_TMC2300 (adr,0x6C, 0x6008001);   //  выберем шаг
			break;
		case 8:
			write_to_TMC2300 (adr,0x6C, 0x5008001);   //  выберем шаг
			break;
		case 16:
			write_to_TMC2300 (adr,0x6C, 0x4008001);   //  выберем шаг
			break;
		case 32:
			write_to_TMC2300 (adr,0x6C, 0x3008001);   //  выберем шаг
			break;
		case 64:
			write_to_TMC2300 (adr,0x6C, 0x2008001);   //  выберем шаг
			break;
		case 128:
			write_to_TMC2300 (adr,0x6C, 0x1008001);   //  выберем шаг
			break;
		case 256:
			write_to_TMC2300 (adr,0x6C, 0x10008001);   //  выберем шаг
			break;
		}
		Delay_ms(2);
		return 0x01;
	} else return 0x00;
}*/
























































/*void WriteFlash(void* Src, void* Dst, int Len)   // чтоб записать много данных
{
  uint16_t* SrcW = (uint16_t*)Src;
  volatile uint16_t* DstW = (uint16_t*)Dst;

  FLASH->CR |= FLASH_CR_PG; // Programm the flash
  while (Len)
  {
    *DstW = *SrcW;
    while ((FLASH->SR & FLASH_SR_BSY) != 0 )
      ;
    if (*DstW != *SrcW )
    {
      goto EndPrg;
    }
    DstW++;
    SrcW++;
    Len = Len - sizeof(uint16_t);
  }
EndPrg:
  FLASH->CR &= ~FLASH_CR_PG;
} */
