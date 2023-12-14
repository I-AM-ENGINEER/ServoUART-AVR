// Такая частота выбрана потому, что хорошо подходит для синхронизации на стандартных скоростях UART
#define F_CPU 7372800UL 

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>

#define DEFAULT_ANGLE		90
#define DEFAULT_MIX_ANGLE	0
#define DEFAULT_MAX_ANGLE	180

#define PDM_MIN_TIMING		1000	// мкс
#define PDM_MAX_TIMING		2000	// мкс

#define CMD_MAX_LENGTH		16
#define CMD_SET_ANGLE		"ANGLE_"
#define CMD_SET_MIN			"SET_MIN_"
#define CMD_SET_MAX			"SET_MAX_"

#define LCD_PORT			PORTA
#define LCD_PIN_RS			PA5
#define LCD_PIN_E			PA4

void usart_rx_complete_irq();

char usart_rx_buffer[CMD_MAX_LENGTH]; // Буфер для принимаемой команды
// Буффер для отображения последней принятой команды
char received_cmd[CMD_MAX_LENGTH];
volatile uint8_t angle_required = DEFAULT_ANGLE;
volatile uint8_t angle_min = DEFAULT_MIX_ANGLE;
volatile uint8_t angle_max = DEFAULT_MAX_ANGLE;

/**************************** Драйвер USART ******************************/

void usart_init( uint16_t ubrr ){
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;
	// Включение приема, передачи, прерыванея при приеме байта
	UCSR0B = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE0);
	// Длина пакета 8 бит
	UCSR0C = (3<<UCSZ0);
}

void usart_send_byte( uint8_t data ){
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void usart_send_str( const char* str ){
	while(*str){
		usart_send_byte((char)*str);
		str++;
	}
}

ISR( USART0_RX_vect ){ // Прерывание при приеме байта по UART
	static uint8_t usart_rx_len;
	uint8_t received_byte = UDR0;
	// Для того, что бы эхо было
	usart_send_byte(received_byte);
	usart_rx_buffer[usart_rx_len++] = received_byte;
	// Backspace для удобного ввода через терминал
	if(received_byte == 0x08){
		if(usart_rx_len >= 2){
			usart_rx_len -= 2;
		}else{
			usart_rx_len = 0;
		}
	// Проверяем на терминирующий символ, или выход за пределы стека
	}else if( 
		(received_byte == '\0') ||\
		(received_byte == '\r') ||\
		(received_byte == '\n') ||\
		((usart_rx_len+1) == sizeof(usart_rx_buffer))
	){
		if(usart_rx_len <= 2){
			return;
		}
		// Для того, что бы явно обозначить конец строки
		usart_rx_buffer[usart_rx_len] = '\0';
		// Вызов функции обработчика принятой команды
		usart_rx_complete_irq(usart_rx_buffer);
		usart_rx_len = 0;
	}
}

/******************** Драйвер дисплея ********************/

void lcd_latch( void ){
	LCD_PORT |=  (1 << LCD_PIN_E); // Установить высокий уровень на тактовом пине шины
	_delay_us(20); // Задержка, что бы дисплей успел считать данные с шины, должно быть больше 450нс
	LCD_PORT &= ~(1 << LCD_PIN_E); // Установить низкий уровень на тактовом пине шины
	_delay_us(20); // Задержка, что бы дисплей успел считать данные с шины, должно быть больше 450нс
}

void lcd_write( uint8_t byte ){
	LCD_PORT = (byte >> 4) | (LCD_PORT & 0xF0); // Записываем старшие 4 байта на шину
	lcd_latch();
	LCD_PORT = (byte & 0x0F) | (LCD_PORT & 0xF0); // Записываем младшие 4 байта на шину
	lcd_latch();
	_delay_us(100); // Выполнение записи может занять до 40мкс, ждем
}

void lcd_write_cmd( uint8_t cmd ){
	LCD_PORT &= ~(1 << LCD_PIN_RS); // Дисплей в режим команд (0 на пине RS)
	lcd_write(cmd); // Запись команды
}

void lcd_putc( uint8_t data ){
	LCD_PORT |= (1 << LCD_PIN_RS); // Дисплей в режим данных
	lcd_write(data); // Запись данных
}

void lcd_puts( const char* str ){
	while(*str){
		lcd_putc(*str);
		str++;
	}
}

void lcd_set_cursor( uint8_t line, uint8_t columm ){
	uint8_t position = (line << 6) | (columm); // Установка позиции курсора
	lcd_write_cmd(0x80 | position);
}

void lcd_clear( void ){
	lcd_write_cmd(0x01); // Команда очистки дисплея
	_delay_ms(2); // Очистка занимает до 1.52мс
}

void lcd_init( void ){
	// Последовательность иницаиализации из даташита
	_delay_ms(20); // Задержка, что бы дисплей успел включиться
	LCD_PORT = 0x03 | (LCD_PORT & 0xF0); // Записываем старшие 4 байта на шину
	lcd_latch();
	_delay_ms(5);
	LCD_PORT = 0x03 | (LCD_PORT & 0xF0); // Записываем старшие 4 байта на шину
	lcd_latch();
	_delay_us(100); // Выполнение записи может занять до 40мкс, ждем
	
	lcd_write_cmd(0x32);
	lcd_write_cmd(0x28);
	lcd_write_cmd(0x0C);
	lcd_write_cmd(0x06);

	lcd_clear();
}

/******************** Драйвер генератора PDM сигнала ********************/

void pdm_init( void ){
	// PWM пин OC1A в режим выхода
	DDRB = (1 << PB5);
	// Настройка таймера генерарующего PDM сигнал для управления серво (TIM1)
	// Выход OC1A установить в лог 0 при сравнении таймера с каналом А
	TCCR1A = (1<<COM1A1)|(1<<WGM11);
	// Делитель 8, максимальный период таймера (2^16)/(F_CPU/8) = 70,5мс, надо 20мс, подходит
	// Генерация ШИМ с точной частотой
	TCCR1B = (1<<CS11)|(1<<WGM13)|(1<<WGM12);
	// Установка частоты, в данном случае для F_CPU=7.3728МГц и выходной частоты 50Гц,
	// надо сбрасывать таймер по достижению 18431
	ICR1 = (uint16_t)(F_CPU/(50*8)-1);
}

void pdm_set_angle( uint8_t angle ){
	uint32_t us = (uint32_t)((uint32_t)angle*(PDM_MAX_TIMING - PDM_MIN_TIMING)/180UL + PDM_MIN_TIMING);
	OCR1A = (uint16_t)(((F_CPU/100)*us)/80000UL);
	// Реальная формула F_CPU*us/8E6, немного оптимизации что бы влезть в 32 бита точности
}

/******************** Высокоуровневые обработчики ********************/

bool pdm_parse_angle( const char* angle_str, uint8_t* angle ){
	// Проверка на то, что это число
	if(angle_str[0] < '0'){
		return false;
	}
	if(angle_str[0] > '9'){
		return false;
	}
	// Так как atoi возвращает 32 число, надо сохранить в 32 битную переменную
	int32_t parsed_angle = atoi(angle_str);
	// Проверка на валидность
	if(parsed_angle > 180) {
		return false;
	}
	if(parsed_angle < 0) {
		return false;
	}
	// Если все ок, записываем угол
	*angle = (uint8_t)parsed_angle;
	return true;
}

void usart_rx_complete_irq( const char *buffer ){
	static uint8_t old_angle = DEFAULT_ANGLE;
	// Сохранить последнюю принятую строку в буфер для отображения на дисплее
	strcpy(received_cmd, buffer);
	// Команда на запись требуемого угла
	if(!memcmp_P(received_cmd, PSTR(CMD_SET_ANGLE), sizeof(CMD_SET_ANGLE)-1)){
		uint8_t parsed_angle;
		// Парсим угол, если он вне рабочего диапозона сервопривода, выход
		if(!pdm_parse_angle(&received_cmd[sizeof(CMD_SET_ANGLE)-1], &parsed_angle)){
			return;
		}
		// Если указанный угол больше максимума, ставим на максимум
		// Если меньше минимума - минимум, иначе установка требуемого угла
		if(parsed_angle > angle_max){
			angle_required = angle_max;
		}else if(parsed_angle < angle_min){
			angle_required = angle_min;
		}else{
			angle_required = parsed_angle;
		}
	}else if(!memcmp_P(received_cmd, PSTR(CMD_SET_MAX), sizeof(CMD_SET_MAX)-1)){
		uint8_t parsed_angle;
		// Парсим угол, если он вне рабочего диапозона сервопривода, выход
		if(!pdm_parse_angle(&received_cmd[sizeof(CMD_SET_MAX)-1], &parsed_angle)){
			return;
		}
		// Если максимальный угол больше минимального, установка нового ограничения
		if(parsed_angle > angle_min){
			angle_max = parsed_angle;
			// И если сервопривод теперь за пределами, устанавливаем максимум
			if(angle_required > angle_max){
				angle_required = angle_max;
			}
		}
	}else if(!memcmp_P(received_cmd, PSTR(CMD_SET_MIN), sizeof(CMD_SET_MIN)-1)){
		uint8_t parsed_angle;
		// Парсим угол, если он вне рабочего диапозона сервопривода, выход
		if(!pdm_parse_angle(&received_cmd[sizeof(CMD_SET_MIN)-1], &parsed_angle)){
			return;
		}
		// Если максимальный угол больше минимального, установка нового ограничения
		if(parsed_angle < angle_max){
			angle_min = parsed_angle;
			// И если сервопривод теперь за пределами, устанавливаем максимум
			if(angle_required < angle_min){
				angle_required = angle_min;
			}
		}
	}
	// Если угол изменился
	if(old_angle != angle_required){
		pdm_set_angle(angle_required);
		old_angle = angle_required;
	}
}

int main( void ){
	// При UBRR=47, скорость UART 9600 бод при FCPU=7.372МГц
	usart_init(47); // USART0
	
	pdm_init();
	// Нужные пины в режим выхода
	DDRA = 0x3F;
	// Инициализация дисплея
	lcd_init();
	sei(); // Глобальные прерывания включить
	pdm_set_angle(DEFAULT_ANGLE);
	char print_str[8];
	while (1){
		lcd_clear();
		lcd_set_cursor(0,0);
		lcd_puts(itoa(angle_min, print_str, 10));
		lcd_set_cursor(0,4);
		lcd_puts(itoa(angle_required, print_str, 10));
		lcd_set_cursor(0,8);
		lcd_puts(itoa(angle_max, print_str, 10));
		lcd_set_cursor(1,0);
		lcd_puts(received_cmd);
		_delay_ms(50);
	}
}
