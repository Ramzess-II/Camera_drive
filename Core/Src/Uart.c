#include "main.h"

//----------------------- объявим функции ------------------------------------//
void swuart_calcCRC(uint8_t *datagram, uint8_t datagramLength);
uint8_t calcCRC(uint8_t len, uint8_t *buf);
void form_send_byte(uint8_t number_send, uint8_t num_byte);
void divide_32bit(uint32_t data, uint8_t start_num);
void set_flag(uint32_t flag);
uint32_t make_32bit(uint8_t *start_num);
uint32_t read_flag(void);
//----------------------- объявим внешние переменные --------------------------//
extern struct _flags *point_flags;
extern struct stepp stepp_1;
extern struct stepp stepp_2;
extern uint8_t pid_param;

enum {             // для протокола обмена
	PING = 0x01,
	SET_PARAM,
	MOTOR_POS,
	SET_BAUD,
	NEW_POS,
	SIGN,
	NEW_SPEED,
	ZERO_POS,
	STOP_MOTOR,
	VERSION,
	NEW_PID,
	RESTARTS,
};

struct data {    // сюда пишем данные для отправки в юарт
	uint32_t one;
	uint32_t two;
	uint32_t three;
};

struct USART {
	uint8_t tx_buffer[32];    // Буфер под входящие данные
	uint8_t rx_buffer[32];    // Буфер под выходящие данные
	uint8_t tx_counter;       // счетчик на передачу,
	uint8_t rx_counter;       // счетчик на прием,
	uint8_t tx_size;          // размер передачи
	uint8_t rx_ok;            // данные приняты
};

struct USART_TX_only husart1;
struct USART husart2;
struct data send_data;

void init_Uart1(uint32_t baud) {
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);  // включим тактирование

	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER9_1); // альтернативную функцию включим
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR9); // максимальную скорость
	SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_9); // открытый колектор
	SET_BIT(GPIOA->AFR[1], 1<<GPIO_AFRH_AFSEL9_Pos); // альтернативную функцию включим см даташит

	//SET_BIT(USART1->CR1, USART_CR1_M1);  размер байта 7,8,9
	//SET_BIT(USART1->CR1, USART_CR1_M0);  размер байта 7,8,9
	SET_BIT(USART1->CR1, USART_CR1_OVER8);// делитель на 8
	//SET_BIT(USART1->CR1, USART_CR1_PCE);  // включить контроль паритета
	//SET_BIT(USART1->CR1, USART_CR1_PS);   // Even or Odd паритет
	//SET_BIT(USART1->CR1, USART_CR1_TXEIE);  // TXE прерывание при передаче нужно включать только после отправки первого байта
	SET_BIT(USART1->CR1, USART_CR1_TE);       // передатчик включить
	SET_BIT(USART1->CR1, USART_CR1_RE);       // приемник включить
	SET_BIT(USART1->CR3, USART_CR3_HDSEL);    // однопроводной режим

	set_baud(USART1, baud);

	SET_BIT(USART1->CR1, USART_CR1_RXNEIE);   // прерывание на прием
	//SET_BIT(USART1->CR1, USART_CR1_IDLEIE);   // прерывание IDLEIE для надежности обнулим руками
	SET_BIT(USART1->CR1, USART_CR1_UE); // включим USART
	NVIC_EnableIRQ(USART1_IRQn);        //Включим прерывания по USART
	NVIC_SetPriority(USART1_IRQn, 1);   // выставим приоритет
}

void init_Uart2(uint32_t baud) {
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);   // включим тактирование юарта

	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER2_1); // альтернативную функцию включим для ножки А2
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR2);    // максимальную скорость
	SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_2);           // открытый колектор
	SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR2_0);         // резистор к +
	SET_BIT(GPIOA->AFR[0], 1<<GPIO_AFRL_AFSEL2_Pos);    // альтернативную функцию включим см даташит

	SET_BIT(GPIOA->MODER, GPIO_MODER_MODER3_1); // альтернативную функцию включим
	SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDER_OSPEEDR3);  // максимальную скорость
	SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPDR3_0);       // резистор к +
	SET_BIT(GPIOA->AFR[0], 1<<GPIO_AFRL_AFSEL3_Pos); // альтернативную функцию включим см даташит

	SET_BIT(USART2->CR1, USART_CR1_OVER8);    // делитель на 8
	SET_BIT(USART2->CR1, USART_CR1_TE);       // передатчик включить
	SET_BIT(USART2->CR1, USART_CR1_RE);       // приемник включить
	SET_BIT(USART2->CR1, USART_CR1_RXNEIE);   // прерывание на прием
	SET_BIT(USART2->CR1, USART_CR1_IDLEIE);   // прерывание IDLEIE

	set_baud(USART2, baud);

	SET_BIT(USART2->CR1, USART_CR1_UE);   // включим USART
	NVIC_EnableIRQ(USART2_IRQn);          //Включим прерывания по USART
	NVIC_SetPriority(USART2_IRQn, 1);     // выставим приоритет
}

void set_baud(USART_TypeDef *usart, uint32_t baud) {   //добавить защиту если не один из баудов не совпал пилить на какой то скорости
	uint16_t push;
	usart->BRR = (2 * SYS_CLOCK / baud);       // настройка частоты
	push = usart->BRR;
	push = push << 12;
	push = push >> 13;
	usart->BRR &= ~0xF;
	usart->BRR |= push;
}

void UART1_transmit(uint8_t lenght, uint8_t *data) {
	while (husart1.tx_counter)
		;   //Ждем, пока линия не освободится
	USART1->TDR = *data;          //Кидаем данные
	husart1.tx_size = lenght;
	husart1.tx_counter = 1;         // увеличиваем счетчик
	USART1->CR1 |= USART_CR1_TXEIE; // включим прерывание
}

void UART2_transmit(uint8_t lenght, uint8_t *data) {
	while (husart2.tx_counter)
		;   //Ждем, пока линия не освободится
	USART2->TDR = *data;          //Кидаем данные
	husart2.tx_size = lenght;
	husart2.tx_counter = 1;
	USART2->CR1 |= USART_CR1_TXEIE; // включим прерывание
}

void write_to_TMC2300(uint8_t adr, uint8_t reg, uint32_t data) {
	husart1.tx_buffer[0] = 0x55; // это синхронизация
	husart1.tx_buffer[1] = adr; // адрес модуля
	husart1.tx_buffer[2] = 0x80; // устновим бит записи
	husart1.tx_buffer[2] |= reg; // регистр в который пишем + бит записи
	husart1.tx_buffer[3] = data >> 24;
	husart1.tx_buffer[4] = data >> 16;
	husart1.tx_buffer[5] = data >> 8;
	husart1.tx_buffer[6] = data;
	swuart_calcCRC(husart1.tx_buffer, 8);
	UART1_transmit(8, husart1.tx_buffer);
}

void read_TMC2300(uint8_t adr, uint8_t reg) {  //0x6A считать
 husart1.tx_buffer[0] = 0x55; // это синхронизация
 husart1.tx_buffer[1] = adr; // адрес модуля
 husart1.tx_buffer[2] = reg; // регистр в который пишем + бит записи
 swuart_calcCRC (husart1.tx_buffer, 4);
 UART1_transmit(4, husart1.tx_buffer);
 }

void setting_TMC230(void) {
	write_to_TMC2300(0, 0x6C, 0x8008001);        // включим, выберем шаг STEP3 // 8
	Delay_ms(2);
	write_to_TMC2300(0, 0x10, 0x10208);          // выберем ток STEP3
	Delay_ms(2);
	write_to_TMC2300(1, 0x6C, 0x8008001);        // включим, полный шаг STEP1
	Delay_ms(2);
	write_to_TMC2300(3, 0x6C, 0x8008001);        // включим, полный шаг STEP2
	Delay_ms(2);
}

void parsing_data(void) {  // парсим данные
	uint32_t bdt = 0;      // для запоминания скорости перед установкой
	if (husart2.rx_ok) {   // если флаг ок, сбросим его
		husart2.rx_ok = 0;
		if (husart2.rx_buffer[0] == 0x02) { // если совпадает начало посылки  умножение на 4 заменим смещением на 2
			if (calcCRC(((husart2.rx_buffer[1] << 2) + 3), husart2.rx_buffer)
					== husart2.rx_buffer[((husart2.rx_buffer[1] << 2) + 3)]) { // проверим срс
				switch (husart2.rx_buffer[2]) { // в зависимости от типа посылки выполняем операции
				case 1:
					send_data.one = 0x01; // запишем что нужно передать в первую переменную структуры
					form_send_byte(PING, 1); // отправим ответ на пинг (тип посылки, одна переменная)
					break;
				case 2:
					new_data_flag(make_32bit(husart2.rx_buffer + 3)); // соберем из переданного буфера переменную с 3 элемента
					send_data.one = read_flag();
					form_send_byte(SET_PARAM, 1);
					break;
				case 3:
					send_data.one = poz_motor(1); // записать текущую позицию мотора
					send_data.two = poz_motor(2);
					send_data.three = read_flag(); // и флаг (доделать движение моторов)
					form_send_byte(MOTOR_POS, 3);
					break;
				case 4:
					send_data.one = new_baud(make_32bit(husart2.rx_buffer + 3)); // установить новую скорость, перенастроить
					bdt = send_data.one;
					form_send_byte(SET_BAUD, 1);  // ответить скоростью
					Delay_ms(5);
					if (bdt) new_baud_set (bdt);  // если бауд правильный то установить его
					break;
				case 5:
					send_data.one = extrn_step(make_32bit(husart2.rx_buffer + 3),
							make_32bit(husart2.rx_buffer + 7)); // установить поз моторов
					form_send_byte(NEW_POS, 1);
					break;
				case 6:
					send_data.one = setting_pin_rasbery (make_32bit(husart2.rx_buffer + 3));
					form_send_byte(SIGN, 1);
					break;
				case 7:
					send_data.one = new_speed(make_32bit(husart2.rx_buffer + 3),make_32bit(husart2.rx_buffer + 7));
					form_send_byte(NEW_SPEED, 1);
					break;
				case 8:
					break;
				case 9:
					send_data.one = 0x01;   // отправить ок
					form_send_byte(STOP_MOTOR, 1);
					stop_motor();           // остановить моторы
					break;
				case 10:
					send_data.one = VERSIONS; // отправить версию
					form_send_byte(VERSION, 1);
					break;
				case 11:
					send_data.one = set_pid(make_32bit(husart2.rx_buffer + 3));
					form_send_byte(NEW_PID, 1);
					break;
				case 12:
					send_data.one = 0x01;
					form_send_byte(RESTARTS, 1);
					while (1);
					break;
				}
			} else {                                       // если не совпал срс
				send_data.one = 0x00;   // отправить не ок
				form_send_byte(husart2.rx_buffer[2], 0);
			}
		}
	}
}

uint32_t read_flag(void) {           // считем отдельные флаги в одну переменную
	uint32_t data = 0;
	data = point_flags->ir_filter;
	data |= point_flags->iris_drive << 1;
	data |= point_flags->set_led << 2;        //??
	if (READ_MOTOR1)
		data |= 1 << 3;
	if (READ_MOTOR2)
		data |= 1 << 4;
	data |= pid_param << 5;
	return data;
}

void form_send_byte(uint8_t number_send, uint8_t num_byte) { // функция для формирования ответа. указываем количество 32байтных данных для отправки
	husart2.tx_buffer[0] = 0x02;        // начало посылки
	husart2.tx_buffer[1] = num_byte;    // сколько полезных данных
	husart2.tx_buffer[2] = number_send; // номер посылки (ее смысл)
	divide_32bit(send_data.one, 3);    // раскладываем переменную в буфер
	switch (num_byte) {
	case 0:
		husart2.tx_buffer[3] = calcCRC(3, husart2.tx_buffer);
		UART2_transmit(4, husart2.tx_buffer);
		break;
	case 1:
		husart2.tx_buffer[7] = calcCRC(7, husart2.tx_buffer);
		UART2_transmit(8, husart2.tx_buffer);
		break;
	case 2:
		divide_32bit(send_data.two, 7);
		husart2.tx_buffer[11] = calcCRC(11, husart2.tx_buffer);
		UART2_transmit(12, husart2.tx_buffer);
		break;
	case 3:
		divide_32bit(send_data.two, 7);
		divide_32bit(send_data.three, 11);
		husart2.tx_buffer[15] = calcCRC(15, husart2.tx_buffer);
		UART2_transmit(16, husart2.tx_buffer);
		break;
	}
}

uint32_t make_32bit(uint8_t *start_num) { // собрать 32битную переменную из буфера
	uint32_t data = 0;
	data = *start_num << 24;
	start_num++;
	data |= *start_num << 16;
	start_num++;
	data |= *start_num << 8;
	start_num++;
	data |= *start_num;
	return data;
}

void divide_32bit(uint32_t data, uint8_t start_num) { // разложить 32битную переменную в буфер
	husart2.tx_buffer[start_num] = data >> 24;
	husart2.tx_buffer[start_num + 1] = data >> 16;
	husart2.tx_buffer[start_num + 2] = data >> 8;
	husart2.tx_buffer[start_num + 3] = data;
}

void swuart_calcCRC(uint8_t *datagram, uint8_t datagramLength) {
	int i, j;
	uint8_t *crc = datagram + (datagramLength - 1); // CRC located in last byte of message
	uint8_t currentByte;
	*crc = 0;
	for (i = 0; i < (datagramLength - 1); i++) { // Execute for all bytes of a message
		currentByte = datagram[i]; // Retrieve a byte to be sent from Array
		for (j = 0; j < 8; j++) {
			if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
					{
				*crc = (*crc << 1) ^ 0x07;
			} else {
				*crc = (*crc << 1);
			}
			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
}

uint8_t calcCRC(uint8_t len, uint8_t *buf) {
	uint8_t r = 0;
	for (int i = 0; i < len; i++) {
		r -= buf[i];
	}
	r &= 0x7F;
	if (r < 0x21) {
		r += 0x21;
	}
#ifdef DEBUG_MODE
     return 1;
#endif
	return r;
}

void USART1_IRQHandler(void) {
	if (READ_BIT(USART1->ISR, USART_ISR_TXE)) {
		if (husart1.tx_size > husart1.tx_counter) {
			USART1->TDR = husart1.tx_buffer[husart1.tx_counter];
			husart1.tx_counter++;
		} else {
			CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE);
			husart1.tx_counter = 0;
			husart1.tx_size = 0;
		}
	}

	if (READ_BIT(USART1->ISR, USART_ISR_RXNE)) {   //Если пришли данные по USART
		husart1.rx_buffer[husart1.rx_counter] = USART1->RDR; //Считаем данные в соответствующую ячейку в rx_buffer
		husart1.rx_counter++;              //Увеличим счетчик принятых байт на 1
		if (husart1.rx_counter > 14)
			husart1.rx_counter = 0;
	}
	if (READ_BIT(USART1->ISR, USART_ISR_ORE)) {       //Если прилетел флаг OVRN
		SET_BIT(USART1->ICR, USART_ICR_ORECF);              //Сбросим флаг OVRN
	}
	/*if (READ_BIT(USART1->ISR, USART_ISR_IDLE)) {       //Если прилетел флаг IDLE
		SET_BIT(USART1->ICR, USART_ICR_IDLECF);              //Сбросим флаг IDLE
		husart1.rx_ok = 1;
		husart1.rx_counter = 0;              //сбросим счетчик приходящих данных
	}*/
}

void USART2_IRQHandler(void) {
	if (READ_BIT(USART2->ISR, USART_ISR_TXE)) {
		if (husart2.tx_size > husart2.tx_counter) {
			USART2->TDR = husart2.tx_buffer[husart2.tx_counter];
			husart2.tx_counter++;
		} else {
			USART2->CR1 &= ~USART_CR1_TXEIE;      // сбросим прерывание
			husart2.tx_counter = 0;
			husart2.tx_size = 0;
		}
	}
	if (READ_BIT(USART2->ISR, USART_ISR_RXNE)) {   //Если пришли данные по USART
		husart2.rx_buffer[husart2.rx_counter] = USART2->RDR; //Считаем данные в соответствующую ячейку в rx_buffer
		husart2.rx_counter++;              //Увеличим счетчик принятых байт на 1
		if (husart2.rx_counter > 31)
			husart2.rx_counter = 0;
	}
	if (READ_BIT(USART2->ISR, USART_ISR_IDLE)) {       //Если прилетел флаг IDLE
		SET_BIT(USART2->ICR, USART_ICR_IDLECF);              //Сбросим флаг IDLE
		husart2.rx_ok = 1;
		husart2.rx_counter = 0;              //сбросим счетчик приходящих данных
	}
	if (READ_BIT(USART2->ISR, USART_ISR_ORE)) {       //Если прилетел флаг OVRN
		SET_BIT(USART2->ICR, USART_ICR_ORECF);              //Сбросим флаг OVRN
	}
}

// структура посылки 0x02 стартовый байт, uint8_t - длина посылки (количество 32битных пакетов), uint8_t - номер посылки, uint32_t данные, срс.
// если длина посылки 0, то это команда.
// 0х01 - пинг, ответ 01 - ок, 00 не ок
// 0х02 - установка параметров за один заход.
// 0 бит - set_ir_filter, 1 бит - set_iris, 2 бит set_led, 3 бит ... , ответ новые параметры
// 0х03 - запрос позиции моторов
// 0х03 - ответ, длина (3 переменных 32бита), 1 позиция 1 мотора, 2 позиция 2 мотора, 3 - 0 бит ir_filter (вкл/выкл),
// 1 бит iris(вкл/выкл) 2 бит led (вкл/выкл), 3 бит мотор 1 (в движении/остановлен), 4 бит мотор 2 (в движении/остановлен)
// 0х04 - установить скорость обмена (9600-11520. только стандарт), ответ новой скоростью, 0x01 - err
// 0х05 - установить новое положение двигателя (первый uint32_t) номер движка,  (второй int32_t) позиция.  ответ 0x01
// 0х07 - установить скорость двигателей (0-1000).  ответ новыми параметрами или 0 если ошибка
// 0х08 - режим работы ножек 0 - если хоть один двигается 1 на первом, если не двигаются 1 на втором. 1 - на каждый движок своя ножка
// 0х09 - остановить движки. ответ 01 - ок, 00 не ок
// 0х0А - получить версию. ответ версия
// 0х0B - установить открытие лепестков 0 - 100%. ответ новыми параметрами или 0 ошибка
// 0х0C - команда рестарта
// ping 02 01 01 00 00 00 01 7B
// set_new_pos 02 02 05 00 00 FF FF 00 00 FF FF 7B or 02 02 05 00 00 00 AA 00 00 00 AA 23 or 02 02 05 00 00 FF F0 00 00 FF F0 3A

