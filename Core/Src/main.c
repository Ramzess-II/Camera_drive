
#include "main.h"
//----------------------- объявим внешние переменные --------------------------//
extern struct _flags *point_flags;
extern uint16_t adc_buf [2] ;
extern struct USART_TX_only husart1;
extern struct stepp stepp_1;
extern struct stepp stepp_2;
static inline void check_iris (uint8_t poz);
static inline void check_mov (void);
//----------------------- объявим внутренние переменные ------------------------//
float kp = 0.15;     // 0.25
float ki = 1.8;      // 2.2
float kd = 0.000005; // 0.00005
int ks = 0;

int open = 2000;
int min = 100;
int max = 600; //420
uint8_t pid_param = 40;

volatile uint32_t count = 0;
uint8_t flg = 0;

int main(void) {
	init_rcc();                              // 48Мгц
	CMSIS_SysTick_Timer_init();              // систик
	init_pins();                             // пины
	init_tim17(3, 10000);                    // настройка таймера делитель + счетчик
	init_tim16(3, 10000);                    // настройка таймера делитель + счетчик
	init_Uart1 (115200);                     // юарт ТМС
	ADC_init();                              // АЦП + ДМА
	EN_ON;                                   // включить разрешить работу драйверов
#ifndef DEBUG_MODE
    Delay_ms(4000);                          // это чтоб можно было ножки программирования схватить
    init_debug_pin ();                       // настроить пины программирования на выход
#endif
    Delay_ms(300);
    setting_TMC230();                        // отправим настройки ТМС
    init_struct();                           // инициализация данных в структуре, чтоб правильно шагать
    stepper (9000, 0);                       // улетим в крайнюю точку чтоб от нее искать 0
    stepper (9000, 1);                       // улетим в крайнюю точку чтоб от нее искать 0
    Delay_ms(4000);
    search_steps ();                         // поиск от крайней точки до отключения концевика
    Delay_ms(6000);
    stepper (-5000, 0);                      // улетим в крайнюю точку чтоб от нее искать 0
    stepper (-5000, 1);                      // улетим в крайнюю точку чтоб от нее искать 0
    Delay_ms(4000);                          // ждем пока едет
    zero_position();                         // ищем ноль
    Delay_ms(4000);                          // ждем, это для того чтоб с той же скоростью искало 0
    init_flash_data() ;                      // запишем все переменные из флеша в используемые ячейки (скорость, шаг)
    init_tim3 ();                            // инит таймера 3 для шима
	husart1.rx_counter = 0;                  // сбросим счетчик чтоб точно началось с 0 байта запись
	read_TMC2300 (0, 0x6A);                  // чтоб понять в каком положении был двигатель и инициализировать его
    Delay_ms(4);
    check_iris (husart1.rx_buffer[9]);       // проверить в какой позиции сейчас шторка
    init_wdt ();                             // инициализация вач дога
    init_Uart2 (57600);                      // юарт Малинка

   //const uint16_t  FLASH_SIZE = (*((uint16_t*)FLASHSIZE_BASE)) << 10;   // размер памяти узнать

	for(;;){
		iwdt_reset();                          // сбросить ватч дог
		/*if (point_flags->dma_ok){            // если сработало ДМА то можно фильтровать значения
			point_flags->dma_ok = FALSE;
			filtr_adc ();
		}*/
		if (!point_flags->zero_in_programm) parsing_data();    // обработать данные с юарта только когда не обнуляемся
		if (point_flags->reset_setting) {                      // вернуть в исходное положение настройки скорости и шага после обнуления
			point_flags->reset_setting = FALSE;
			init_flash_data ();                                // вернуть настройку скорости
		}
		if (point_flags->pid_ok) {                             // если сработал флаг ПИД
			point_flags->pid_ok = FALSE;
			TIM3->CCR1 = computePID (adc_buf[1], open, kp, ki, kd, 0.02, min, max);
		}
		check_mov ();
	}
}

static inline void check_mov (void) {           // проверим есть у нас движение или нет
	if (point_flags->change_pin_conf == 0) {
		if (stepp_1.curent_steps != stepp_1.last_steps && stepp_2.curent_steps != stepp_2.last_steps) {
			MOTOR_MOV_ON;                           // вывод на малинку
			MOTOR_STP_OFF;
		} else {
			MOTOR_MOV_OFF;
			MOTOR_STP_ON;                           // вывод на малинку
		}
	} else {
		if (stepp_1.curent_steps != stepp_1.last_steps) MOTOR_MOV_ON;
				else MOTOR_MOV_OFF;
		if (stepp_2.curent_steps != stepp_2.last_steps) MOTOR_STP_ON;
				else MOTOR_STP_OFF;
	}
}

static inline void check_iris (uint8_t poz) {
	switch (poz) {
	case 0:
    	STEP3_ON;                            // выключим его
    	point_flags->delay = 6;
		break;
	case 1:
    	STEP3_ON;                            // выключим его
    	point_flags->delay = 4;
		break;
	case 2:
    	STEP3_ON;                            // выключим его
    	point_flags->delay = 2;
		break;
	case 3:
		break;
	}
}

float constrain(float x, float a, float b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

// (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
uint32_t computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = (setpoint- ks) - input  ;
  if (ks > 0) ks -= 15;                                           // для замедления реакции
  if (ks < 0) ks += 15;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + err * dt * ki, minOut, maxOut);  // ???? ki
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}

uint32_t set_pid (uint32_t new_pid) {
	if (new_pid < 0 || new_pid > 100) return 0;
	pid_param = new_pid;
	uint16_t mirror = 0;
	/*mirror = map (new_pid, 0, 100, 1000, 4000);   // вот тут зависит от АЦП 600 - 3000 олд
	if (mirror > open){
		if (mirror > open ){ ks = mirror - open; } //ks = ks /2;
		open = mirror;
	}

	if (mirror < open) {
		if (mirror < open ){ ks = mirror - open; } // ks = ks /2;
		open = mirror;
	}*/
	mirror = map (new_pid, 0, 100, 450, 4000);   // вот тут зависит от АЦП 600 - 3000 олд
	ks = mirror - open;
	open = mirror;
	return new_pid;
}

uint32_t setting_pin_rasbery (uint32_t sign) {
	if (sign == 0) {point_flags->change_pin_conf = 0; return 1;}
	if (sign == 1) {point_flags->change_pin_conf = 1; return 1;}
	return 0;
}

	//	__NVIC_SetVector(0, 0x08002800);  // таблица векторов прерываний?
    //SCB->VTOR
