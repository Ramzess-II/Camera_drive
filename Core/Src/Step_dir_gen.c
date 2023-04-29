#include "main.h"

/*struct bits {         // битовое поле
	uint8_t start :1;
	uint8_t upp_down :2;
};*/



struct stepp stepp_1;
struct stepp stepp_2;

struct _flags flags;
struct _flags *point_flags = &flags;

void new_data_flag (uint32_t flag) {                           // проверяем если биты совпадают то ничего не делаем
	if (point_flags->ir_filter == 0 && (flag & 0x01)){         // а если не совпадают то переставляем бит и делаем операцию
		point_flags->ir_filter = 1;
		STEP3_ON;
		flags.delay = 4;
	}
	if (point_flags->ir_filter == 1 && !(flag & 0x01)) {
		point_flags->ir_filter = 0;
		STEP3_ON;
		flags.delay = 4;
	}
}

void stop_motor (void){
	stepp_1.curent_steps = stepp_1.last_steps;
	stepp_2.curent_steps = stepp_2.last_steps;
}

uint32_t poz_motor (uint8_t num_motor){
	if (num_motor == 1) {
		return map (stepp_1.last_steps, (stepp_1.step_down<<4)-PROTECT, (stepp_1.step_up<<4)-PROTECT, RESOLUTION, 0);
	}
	if (num_motor == 2) {
		return map (stepp_2.last_steps, (stepp_2.step_down<<4)-PROTECT, (stepp_2.step_up<<4)-PROTECT, 0,RESOLUTION);
	}
	return 0;
}

static inline void search_zero(void) {              // поиск нуля
	if (!flags.start_zero) {                        // если мы первый раз заходим в прерывание
		flags.start_zero = TRUE;                    // сбросим флаг первого раза
		DIR1_ON;                                    // установим дир в нужное положение, чтоб шагать к концевику
		DIR2_ON;                                    // сюда заходим один раз чтоб сначала установить дир, а потом шагать
	} else {                                        // если уже не первый раз заходим
		if (READ_LIM1 && !flags.zero_pos1) {        // считаем состояние концевика, если не ноль, шагаем
			stepp_1.step_down --;                   // считаем переменную количества шагов от конца
			if (GPIOA->IDR & GPIO_IDR_0)            // считаем что у нас сейчас в регистре выхода,
				STEP1_OFF;                          // если 1 то выставим 0
			else
				STEP1_ON;                           // если же там 0 то установим 1 на выходе
		} else {
			flags.zero_pos1 = TRUE;                 // если вдруг сработал концевик то ставим флаг
			STEP1_OFF;                              // и сбрасываем степ в 0
		}
		if (READ_LIM2 && !flags.zero_pos2) {        // считаем состояние концевика, если не ноль, шагаем
			stepp_2.step_down --;
			if (GPIOA->IDR & GPIO_IDR_1)
				STEP2_OFF;                          // считаем что у нас сейчас в регистре выхода, если 1 то выставим 0
			else
				STEP2_ON;                           // если же там 0 то установим 1 на выходе
		} else {
			flags.zero_pos2 = TRUE;                 // если вдруг сработал концевик то ставим флаг
			STEP2_OFF;
		}
	}

	if (flags.zero_pos1 && flags.zero_pos2) {       // если сработало два флага значит мы в нулях
		flags.zero_ok = TRUE;                       // установим флаг
		stepp_1.last_steps = 0;                     // обнулим нафиг все
		stepp_2.last_steps = 0;
		stepp_1.curent_steps = stepp_1.last_steps;
		stepp_2.curent_steps = stepp_2.last_steps;
		//flags.zero_in_programm = FALSE;             // сбросить флаг обнуления в программе
		//flags.reset_setting = TRUE;                 // установить флаг что нужно перенастроить шаги, скорость
		CLEAR_BIT(TIM17->DIER, TIM_DIER_UIE);       // выключим прерывание по обновлению
	}
}

static inline void revers_zero (void){
	if (!flags.start_zero) {                        // если мы первый раз заходим в прерывание
		flags.start_zero = TRUE;                    // сбросим флаг первого раза
		DIR1_OFF;                                   // установим дир в нужное положение, чтоб шагать к концевику
		DIR2_OFF;                                   // сюда заходим один раз чтоб сначала установить дир, а потом шагать
	} else {                                        // если уже не первый раз заходим
		if (!READ_LIM1 && !flags.zero_pos1) {       // считаем состояние концевика, если не ноль, шагаем
			stepp_1.step_up ++;                     // считаем переменную количества шагов от конца
			if (GPIOA->IDR & GPIO_IDR_0)            // считаем что у нас сейчас в регистре выхода,
				STEP1_OFF;                          // если 1 то выставим 0
			else
				STEP1_ON;                           // если же там 0 то установим 1 на выходе
		} else {
			flags.zero_pos1 = TRUE;                 // если вдруг сработал концевик то ставим флаг
			STEP1_OFF;                              // и сбрасываем степ в 0
		}
		if (!READ_LIM2 && !flags.zero_pos2) {       // считаем состояние концевика, если не ноль, шагаем
			stepp_2.step_up ++;
			if (GPIOA->IDR & GPIO_IDR_1)
				STEP2_OFF;                          // считаем что у нас сейчас в регистре выхода, если 1 то выставим 0
			else
				STEP2_ON;                           // если же там 0 то установим 1 на выходе
		} else {
			flags.zero_pos2 = TRUE;                 // если вдруг сработал концевик то ставим флаг
			STEP2_OFF;
		}
	}

	if (flags.zero_pos1 && flags.zero_pos2) {       // если сработало два флага значит мы в нулях
		flags.zero_ok = TRUE;                       // установим флаг
		CLEAR_BIT(TIM17->DIER, TIM_DIER_UIE);       // выключим прерывание по обновлению
	}
}

static inline void new_step_go1(void) {
	if (flags.balance_1) {                                   // если новое движение
		if (stepp_1.curent_steps > stepp_1.last_steps)       // если у нас движение в одну сторону
			DIR1_ON;                                         // установим единичку на дир
		else                                                 // если в другую
			DIR1_OFF;                                        // установим нолик на дир
		flags.balance_1 = FALSE;                             // сбросим новое движение
	} else {                                                 // если мы уже двигаемся в заданном направлении
		if (stepp_1.curent_steps == stepp_1.last_steps) {    // если шаги равны то пропускаем движение
			CLEAR_BIT(TIM17->DIER, TIM_DIER_UIE);            // если шагать не нужно выключим прерывания
			stepp_1.last_steps = stepp_1.curent_steps;       // приравняем значения
			STEP1_OFF;                                       // и сбросим степ
		} else {
			if (GPIOA->IDR & GPIO_IDR_0) {                   // считаем регистр, если там единичка
				STEP1_OFF;                                   // переведем степ в 0
			} else {                                         // если там нолик то
				STEP1_ON;                                    // переведем степ в 1
			if (stepp_1.curent_steps < stepp_1.last_steps)
				stepp_1.last_steps --;                       // а так же изменим текущие шаги
			else
				stepp_1.last_steps ++;
			}
		}
	}
}

static inline void new_step_go2(void) {
	if (flags.balance_2) {
		if (stepp_2.curent_steps > stepp_2.last_steps)
			DIR2_ON;                                         // установим единичку на дир
		else
			DIR2_OFF;                                        // установим нолик на дир
		flags.balance_2 = FALSE;
	}else {                                                  // если мы уже двигаемся в заданном направлении
		if (stepp_2.curent_steps == stepp_2.last_steps) {    // если шаги равны то пропускаем движение
			CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);            // если шагать не нужно выключим прерывания
			stepp_2.last_steps = stepp_2.curent_steps;       // приравняем значения
			STEP2_OFF;                                       // и сбросим степ
		} else {
			if (GPIOA->IDR & GPIO_IDR_1) {                   // считаем регистр, если там единичка
				STEP2_OFF;                                   // переведем степ в 0
			} else {                                         // если там нолик то
				STEP2_ON;                                    // переведем степ в 1
			if (stepp_2.curent_steps < stepp_2.last_steps)
				stepp_2.last_steps --;                       // а так же изменим текущие шаги
			else
				stepp_2.last_steps ++;
			}
		}
	}
}

void TIM17_IRQHandler(void) {
	TIM17->SR &= ~TIM_SR_UIF;                                // сбросим флаг прерывания
	if (flags.zero_ok == 0) {
		search_zero();
	} else if (flags.zero_ok == 1) {
		new_step_go1();
	} else {
		revers_zero ();
	}
}

void TIM16_IRQHandler(void) {
	TIM16->SR &= ~TIM_SR_UIF;                                // сбросим флаг прерывания
	new_step_go2();
}

uint32_t stepper( int32_t stepper, uint32_t num_motor) {
	if (num_motor > 1) return 0;
	if (num_motor == 0) {
		stepp_1.curent_steps = stepper;
		flags.balance_1 = 1;
		CLEAR_REG(TIM17->CNT);                        // сбросим счетчик в ноль
	    SET_BIT(TIM17->DIER, TIM_DIER_UIE);           // включим прерывание по обновлению
	}
	if (num_motor == 1) {
		stepp_2.curent_steps = stepper;
		flags.balance_2 = 1;
		CLEAR_REG(TIM16->CNT);                        // сбросим счетчик в ноль
	    SET_BIT(TIM16->DIER, TIM_DIER_UIE);           // включим прерывание по обновлению
	}
	return 1;
}

uint32_t extrn_step(uint32_t stepper, uint32_t num_motor){
	if (num_motor > 1) return 0;
	if (stepper > RESOLUTION) return 0;
	if (num_motor == 0) {
		stepp_1.curent_steps = map (stepper, 0, RESOLUTION, (stepp_1.step_up<<4)-PROTECT, (stepp_1.step_down<<4)-PROTECT); // >>1 для полного шага
		flags.balance_1 = 1;
		CLEAR_REG(TIM17->CNT);                        // сбросим счетчик в ноль
	    SET_BIT(TIM17->DIER, TIM_DIER_UIE);           // включим прерывание по обновлению
	}
	if (num_motor == 1) {
		stepp_2.curent_steps = map (stepper, 0,RESOLUTION, (stepp_2.step_down<<4)-PROTECT, (stepp_2.step_up<<4)-PROTECT); // тут умножение на 32 заменено смещением на 4
		flags.balance_2 = 1;     // но так как у нас переменная считается в прерывании 2 раза то нужно еще и разделит на 2, но можем просто делить на 16
		CLEAR_REG(TIM16->CNT);                        // сбросим счетчик в ноль
	    SET_BIT(TIM16->DIER, TIM_DIER_UIE);           // включим прерывание по обновлению
	}
	return 1;
}

void init_struct (void){                         // инициализируем ячейки
	flags.ir_filter = TRUE;
	flags.iris_drive = FALSE;
	flags.zero_ok = TRUE;                        // это первоначальная инициализация чтоб можно было пошагать
	flags.zero_in_programm = FALSE;
	flags.reset_setting = TRUE;
	flags.change_pin_conf = FALSE;
	flags.iwdt_res = FALSE;
	stepp_1.last_steps = 0;                      // это чтоб с самого начала могли пошагать в -8000
	stepp_2.last_steps = 0;
	STEP3_OFF;                                   // ?
}

void zero_position(void) {
	CLEAR_REG(TIM16->CNT);                        // сбросим счетчик в ноль
	CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);         // а тут выключим прерывания
	flags.zero_ok = FALSE;
	flags.zero_pos1 = FALSE;
	flags.zero_pos2 = FALSE;
	flags.start_zero = FALSE;
	stepp_1.step_down = 0;                        // сбросим счетчик шагов "вниз"
	stepp_2.step_down = 0;
	CLEAR_REG(TIM17->CNT);                        // сбросим счетчик в ноль
	SET_BIT(TIM17->DIER, TIM_DIER_UIE);           // включим прерывание по обновлению
}

void search_steps (void) {                        // поиск количества шагов от максимума
	CLEAR_REG(TIM16->CNT);                        // сбросим счетчик в ноль
	CLEAR_BIT(TIM16->DIER, TIM_DIER_UIE);         // а тут выключим прерывания
	stepp_1.step_up = 0;                          // сбросим счетчик шагов "вверх"
	stepp_2.step_up = 0;
	flags.zero_ok = 2;
	flags.zero_pos1 = FALSE;
	flags.zero_pos2 = FALSE;
	flags.start_zero = FALSE;                     // это для первичного выбора направления движения
	CLEAR_REG(TIM17->CNT);                        // сбросим счетчик в ноль
	SET_BIT(TIM17->DIER, TIM_DIER_UIE);           // включим прерывание по обновлению

}

/*void zero_in_program (void) {                 // обнулить в момент выполнения программы
	flags.zero_in_programm = TRUE;              // поднять флаг чтоб не ловить данные из юарта пока
	TIM17->ARR = 10000;                         // скорость вращения
	new_step_no_memory() ;
    stepper (-8000, 0);                         // улетим в крайнюю точку чтоб от нее искать 0
    stepper (-8000, 1);                         // улетим в крайнюю точку чтоб от нее искать 0
    flags.iwdt_res = TRUE;                      // включим сброс ватч дога
    Delay_ms(4000);                             // ждем пока едет
    zero_position();                            // ищем ноль
    Delay_ms(3000);                             // ждем, это для того чтоб с той же скоростью искало
    stepper (stepp_1.step_up >> 1, 0);          // отправим в относительный ноль
    stepper (stepp_2.step_down >> 1, 1);
    Delay_ms(3000);
	flags.zero_in_programm = FALSE;             // сбросить флаг обнуления в программе
	flags.reset_setting = TRUE;                 // установить флаг что нужно перенастроить шаги, скорость
    flags.iwdt_res = FALSE;
}*/

/*void stepper( int32_t stepper_1, int32_t stepper_2) {  // старый вариант
	//stepp_1.last_steps = stepp_1.curent_steps;
	stepp_1.curent_steps = stepper_1;
	flags.balance_1 = 1;
	//stepp_2.last_steps = stepp_2.curent_steps;
	stepp_2.curent_steps = stepper_2;
	flags.balance_2 = 1;
	CLEAR_REG(TIM17->CNT);                        // сбросим счетчик в ноль
	SET_BIT(TIM17->DIER, TIM_DIER_UIE);           // включим прерывание по обновлению
}*/




