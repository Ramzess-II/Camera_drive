
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

//----------------------- подключим файлы ------------------------------------//
#include "stm32f0xx.h"
//----------------------- дефайним значения ----------------------------------//

//#define DEBUG_MODE

#define SYS_CLOCK 48000000
#define UP 1
#define DOWN 2
#define TRUE 1
#define FALSE 0
#define VERSIONS 1

#define RESOLUTION 10000   // разрешение системы (отношение количества шагов к входным данным)
#define PROTECT 10         // отступ от максимальных точек, чтоб не врезалось

//----------------------- дефайним ножки -------------------------------------//
#define STEP1_ON        SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_0)
#define STEP1_OFF       SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_0)
#define STEP2_ON        SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_1)
#define STEP2_OFF       SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_1)
#define IRIS_DRIVE_ON   SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_6)
#define IRIS_DRIVE_OFF  SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_6)
#define STEP3_ON        SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_7)
#define STEP3_OFF       SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_7)

#define EN_ON           SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_1)
#define EN_OFF          SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_1)
#define DIR1_ON         SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_5)
#define DIR1_OFF        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_5)
#define DIR2_ON         SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_4)
#define DIR2_OFF        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_4)
#define DIR3_ON         SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_3)
#define DIR3_OFF        SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR_3)

#define MOTOR_MOV_ON    SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_13)
#define MOTOR_MOV_OFF   SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_13)
#define MOTOR_STP_ON    SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS_14)
#define MOTOR_STP_OFF   SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_14)

#define READ_LIM1       READ_BIT(GPIOB->IDR, GPIO_IDR_7)
#define READ_LIM2       READ_BIT(GPIOB->IDR, GPIO_IDR_6)

#define READ_MOTOR1     READ_BIT(GPIOA->IDR, GPIO_IDR_13)
#define READ_MOTOR2     READ_BIT(GPIOA->IDR, GPIO_IDR_14)


//----------------------- объявим функции ------------------------------------//
void init_rcc (void);
void Delay_ms(uint32_t Milliseconds);
void init_pins (void);
void CMSIS_SysTick_Timer_init(void);
void init_Uart1 (uint32_t baud);
void init_Uart2 (uint32_t baud);
void UART1_transmit (uint8_t lenght, uint8_t *data);
void UART2_transmit (uint8_t lenght, uint8_t *data);
void init_tim17 (uint16_t prescaler, uint16_t reload);
void init_tim16(uint16_t prescaler, uint16_t reload);
void zero_position(void);
uint32_t stepper( int32_t stepper, uint32_t num_motor);
void setting_TMC230 (void);
void ADC_init (void);
void parsing_data (void);
void new_data_flag (uint32_t flag);
uint32_t poz_motor (uint8_t num_motor);
uint32_t new_baud (uint32_t baud);
void new_baud_set (uint32_t baud);
void stop_motor (void);
void iwdt_reset(void);
uint32_t new_speed (uint32_t speed, uint32_t num_motor);
void read_TMC2300(uint8_t adr, uint8_t reg);
void write_to_TMC2300(uint8_t adr, uint8_t reg, uint32_t data);
void set_baud (USART_TypeDef *usart, uint32_t baud);
void filtr_adc (void) ;
void init_flash_data (void);
void init_debug_pin (void);
void init_struct (void);
void new_step_no_memory (void);
void zero_in_program (void);
void init_wdt (void);
void init_tim3 (void);
float constrain(float x, float a, float b);
uint32_t computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut);
long map(long x, long in_min, long in_max, long out_min, long out_max);
uint32_t set_pid (uint32_t new_pid) ;
void search_steps (void);
uint32_t extrn_step(uint32_t stepper, uint32_t num_motor);
uint32_t setting_pin_rasbery (uint32_t sign);


//----------------------- объявим глобальные переменные --------------------------//
struct _flags {         // битовое поле
	uint32_t zero_pos1 :1;
	uint32_t zero_pos2 :1;
	uint32_t start_zero :1;
	uint32_t zero_ok :2;
	uint32_t balance_1 :1;
	uint32_t balance_2 :1;
	uint32_t ir_filter :1;
	uint32_t iris_drive :1;
	uint32_t set_led :1;
	uint32_t revers1 :1;
	uint32_t revers2 :1;
	uint32_t dma_ok :1;
	uint32_t delay :4;
	uint32_t zero_in_programm :1;
	uint32_t reset_setting :1;
	uint32_t iwdt_res :1;
	uint32_t pid_ok :1;
	uint32_t stop_test :1;
	uint32_t change_pin_conf :1;
};

struct stepp {
	//struct bits setting;   // добавим структуру
	int32_t last_steps;
	int32_t curent_steps;
	uint32_t step_up;
	int32_t step_down;
};

struct USART_TX_only {
	uint8_t tx_buffer[10];   // Буфер под выходящие данные
	uint8_t rx_buffer[15];   // Буфер под выходящие данные
	uint8_t tx_counter;      // счетчик на передачу,
	uint8_t rx_counter;       // счетчик на прием,
	uint8_t tx_size;         // размер передачи
	uint8_t rx_ok;           // данные приняты
};

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
