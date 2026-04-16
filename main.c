// ATmega88P 7seg LED Clock (RTC=RX-8025)
// PONフラグクリア対応 + 日付無効日対策 + 12/24時間表示改善版

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2c.h"

#define MODE_NORMAL     0
#define MODE_SET_HOUR   1
#define MODE_SET_MIN    2
#define MODE_SET_SEC    3
#define MODE_SAVE       4
#define MODE_DATE_DISP  5
#define MODE_SET_YEAR   6
#define MODE_SET_MONTH  7
#define MODE_SET_DAY    8

// グローバル変数
volatile uint8_t seg[8];
volatile uint8_t mx = 0;
volatile uint8_t hour = 12, min = 34, sec = 36;
volatile uint8_t year = 25, month = 6, day = 9;
volatile uint8_t mode = MODE_NORMAL;
volatile uint16_t switch_press_time = 0;
volatile uint8_t is_am = 1;
volatile uint8_t is_24hour = 1;
volatile uint8_t rtc_update_flag = 0;
volatile uint8_t colon_blink_state = 0;
volatile uint16_t colon_timer = 0;
volatile uint8_t led8_state = 0;
volatile uint16_t led8_timer = 0;
volatile uint8_t waiting_for_release = 0;
volatile uint16_t switch2_hold_time = 0;
volatile uint8_t blink_enabled = 1;
volatile uint16_t led7_timer = 0;
volatile uint16_t buzzer_timer = 0;
volatile uint16_t date_display_timer = 0;
volatile uint8_t last_mode = MODE_NORMAL;
volatile uint8_t led7_always_on = 0;

volatile uint16_t format_change_timer = 0;
volatile uint8_t  showing_leading_zero = 0;

// 定数定義
//#define COLON_MASK (0x1D)	//7セグホルダー用
#define COLON_MASK	(0x1E)
#define AM_MASK		(1<<6)
#define PM_MASK		(1<<7)
#define LED8_MASK	(1<<5)
//#define LED7_MASK (1<<1)	//7セグホルダー用
#define LED7_MASK (1<<0)

#define BLINK_CYCLES       250
#define COLON_CYCLES       500
#define LED8_CYCLES        50
#define BUZZER_CYCLES      100
#define BUZZER_ON1_START   100
#define BUZZER_OFF1        80
#define BUZZER_ON2         20
#define BUZZER_OFF2        1
#define DATE_DISP_TIME     2000

#define RTC_WRITE 0x64
#define RTC_READ  0x65

static const uint8_t blink_range[9][2] = {
	{0,6},{4,6},{2,4},{0,2},{0,6},{0,6},{4,6},{2,4},{0,2}
};

// 日付有効性用関数
uint8_t days_in_month(uint8_t m, uint8_t y) {
	static const uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	uint8_t d = days[m-1];
	if (m == 2 && (y % 4 == 0)) d = 29;
	return d;
}

uint8_t mask(uint8_t num);
uint8_t dec2bcd(uint8_t v);
uint8_t bcd2dec(uint8_t v);
void rtc_init_full(void);
void rtc_read_time(volatile uint8_t *h, volatile uint8_t *m, volatile uint8_t *s);
void rtc_read_date(volatile uint8_t *y, volatile uint8_t *m, volatile uint8_t *d);
void rtc_write_date(uint8_t y, uint8_t m, uint8_t d);
void process_rtc_update(void);
void read_switches(void);
void set_rtc_time(uint8_t h, uint8_t m, uint8_t s);
void buzzer_start(void);
void buzzer_stop(void);

/*
// ================= 7seg =================(7セグホルダー用）
uint8_t mask(uint8_t num){
	switch(num){
		case 0:return 0x22; case 1:return 0x7B; case 2:return 0x25;
		case 3:return 0x29; case 4:return 0x78; case 5:return 0xA8;
		case 6:return 0xA0; case 7:return 0x3A; case 8:return 0x20;
		case 9:return 0x28; default:return 0xFF;
	}
}
*/

// 7セグメントマスク
uint8_t mask(uint8_t num) {
	switch (num) {
		case 0:return 0x21; case 1:return 0x77; case 2:return 0x2A;
		case 3:return 0x26; case 4:return 0x74; case 5:return 0xA4;
		case 6:return 0xA0; case 7:return 0x35; case 8:return 0x20;
		case 9:return 0x24; default:return 0xFF;
	}
}

uint8_t dec2bcd(uint8_t v) { return ((v/10)<<4) | (v%10); }
uint8_t bcd2dec(uint8_t v) { return ((v>>4)*10) + (v & 0x0F); }

// RX-8025初期化（PONクリアを追加）
void rtc_init_full(void) {
	uint8_t ctrl2;

	// PONフラグ確認
	i2c_start(RTC_WRITE); i2c_send(0xF0);
	i2c_start(RTC_READ);
	ctrl2 = i2c_recv(0);
	i2c_stop();

	if (ctrl2 & 0x10) {  // PON bit (bit4) が立っている場合
		// PONを0にクリア（他のビットは0推奨）
		i2c_start(RTC_WRITE);
		i2c_send(0xF0);
		i2c_send(0x00);
		i2c_stop();
		_delay_us(100);
	}

	// 既存の初期化シーケンス（アラームなどクリア）
	i2c_start(RTC_WRITE); i2c_send(0xE0); i2c_send(0xA3); i2c_stop();
	i2c_start(RTC_WRITE); i2c_send(0x80); i2c_send(0x00); i2c_stop();
	i2c_start(RTC_WRITE); i2c_send(0x90); i2c_send(0x00); i2c_stop();
	i2c_start(RTC_WRITE); i2c_send(0xA0); i2c_send(0x00); i2c_stop();
	i2c_start(RTC_WRITE); i2c_send(0xB0); i2c_send(0x00); i2c_stop();
	i2c_start(RTC_WRITE); i2c_send(0xC0); i2c_send(0x00); i2c_stop();
	// 0x60は削除済み（年レジスタに影響しないように）
}

// 時刻読み書き（変更なし）
void rtc_read_time(volatile uint8_t *h, volatile uint8_t *m, volatile uint8_t *s) {
	i2c_start(RTC_WRITE); i2c_send(0x00);
	i2c_start(RTC_READ);
	*s = bcd2dec(i2c_recv(1) & 0x7F);
	*m = bcd2dec(i2c_recv(1) & 0x7F);
	*h = bcd2dec(i2c_recv(0) & 0x3F);
	i2c_stop();
}

void rtc_read_date(volatile uint8_t *y, volatile uint8_t *m, volatile uint8_t *d) {
	i2c_start(RTC_WRITE); i2c_send(0x40);
	i2c_start(RTC_READ);
	*d = bcd2dec(i2c_recv(1) & 0x3F);
	*m = bcd2dec(i2c_recv(1) & 0x1F);
	*y = bcd2dec(i2c_recv(0));
	i2c_stop();
}

void rtc_write_date(uint8_t y, uint8_t m, uint8_t d) {
	if (y > 99) y = 0;
	if (m < 1 || m > 12) m = 1;
	uint8_t maxd = days_in_month(m, y);
	if (d < 1 || d > maxd) d = maxd;
	i2c_start(RTC_WRITE);
	i2c_send(0x40);
	i2c_send(dec2bcd(d));
	i2c_send(dec2bcd(m));
	i2c_send(dec2bcd(y));
	i2c_stop();
}

ISR(INT0_vect) {
	rtc_update_flag = 1;
	colon_blink_state = 1;
	colon_timer = 0;
	led8_state = 1;
	led8_timer = 0;
	if ((hour == 23 || hour == 11) && min == 59 && sec == 59) buzzer_timer = 100;
}

void process_rtc_update(void) {
	rtc_read_time(&hour, &min, &sec);
	is_am = (hour < 12);
}

void buzzer_start(void) {
	TCCR0A = (1 << COM0A0) | (1 << WGM01);
	TCCR0B = (1 << CS00);
	OCR0A = 124;
}

void buzzer_stop(void) {
	TCCR0A = 0; TCCR0B = 0; PORTD &= ~(1<<PD6);
}

// スイッチ処理（変更なし）
void read_switches(void) {
	static uint8_t last_s1 = 0, last_s2 = 0;
	uint8_t s1 = !(PINC & (1 << PC2));
	uint8_t s2 = !(PINC & (1 << PC3));
	static uint16_t auto_count_timer = 0;
	static uint16_t date_long_press_time = 0;
	static uint8_t skip_next_s2_release = 0;
	static uint8_t skip_next_s1_release = 0;

	if (mode == MODE_NORMAL) {
		if (s1 && s2 && last_s1 && last_s2) {
			if (++switch_press_time >= 2000) {
				mode = MODE_SET_HOUR;
				switch_press_time = 0;
				waiting_for_release = 1;
				skip_next_s2_release = 1;
				skip_next_s1_release = 1;
				rtc_read_time(&hour, &min, &sec);
				is_am = (hour < 12);
			}
			} else {
			switch_press_time = 0;
			if (s1 == 0 && last_s1 == 1 && !s2) {
				is_24hour ^= 1;
				if (!is_24hour) {
					led7_timer = 2000;
					showing_leading_zero = 0;
					} else {
					format_change_timer = 2000;
					showing_leading_zero = 1;
				}
				_delay_us(100);
			}
			if (s2 == 0 && last_s2 == 1 && !s1) {
				mode = MODE_DATE_DISP;
				date_display_timer = DATE_DISP_TIME;
				rtc_read_date(&year, &month, &day);
				_delay_us(100);
			}
		}
		} else if (mode == MODE_DATE_DISP) {
		if (s2 && last_s2) {
			if (++date_long_press_time >= 2000) {
				mode = MODE_SET_YEAR;
				date_long_press_time = 0;
				waiting_for_release = 1;
				skip_next_s1_release = 1;
				skip_next_s2_release = 1;
				rtc_read_date(&year, &month, &day);
			}
			} else {
			date_long_press_time = 0;
		}
		if (!s2 && !last_s2) {
			if (date_display_timer > 0) date_display_timer--;
			if (date_display_timer == 0) mode = MODE_NORMAL;
		}
		} else {
		if (waiting_for_release) {
			if (s1 == 0 && s2 == 0) waiting_for_release = 0;
			return;
		}

		if (s1 == 0 && last_s1 == 1 && !skip_next_s1_release) {
			if (mode == MODE_SET_DAY || mode == MODE_SET_SEC) mode = MODE_SAVE;
			else { mode++; if (mode == MODE_DATE_DISP) mode = MODE_NORMAL; }

			if (mode == MODE_SAVE) {
				if (last_mode >= MODE_SET_HOUR && last_mode <= MODE_SET_SEC)
				set_rtc_time(hour, min, sec);
				else if (last_mode >= MODE_SET_YEAR && last_mode <= MODE_SET_DAY)
				rtc_write_date(year, month, day);

				mode = MODE_NORMAL;
				waiting_for_release = 1;
				skip_next_s1_release = 1;
				skip_next_s2_release = 1;
			}
			switch2_hold_time = 0;
			_delay_us(100);
		}
		if (s1 == 0 && last_s1 == 1) skip_next_s1_release = 0;

		if (!waiting_for_release) {
			if (s2 == 0 && last_s2 == 1 && !skip_next_s2_release) {
				if (mode == MODE_SET_HOUR)      { hour = (hour + 1) % 24; is_am = (hour < 12); }
				else if (mode == MODE_SET_MIN)  min = (min + 1) % 60;
				else if (mode == MODE_SET_SEC)  sec = (sec + 1) % 60;
				else if (mode == MODE_SET_YEAR) year = (year + 1) % 100;
				else if (mode == MODE_SET_MONTH) month = (month % 12) + 1;
				else if (mode == MODE_SET_DAY) {
					day = (day % days_in_month(month, year)) + 1;
				}
				_delay_us(100);
			}
			if (s2 == 1) {
				switch2_hold_time++;
				if (switch2_hold_time >= 500) {
					if (++auto_count_timer >= 100) {
						if (mode == MODE_SET_HOUR)      { hour = (hour + 1) % 24; is_am = (hour < 12); }
						else if (mode == MODE_SET_MIN)  min = (min + 1) % 60;
						else if (mode == MODE_SET_SEC)  sec = (sec + 1) % 60;
						else if (mode == MODE_SET_YEAR) year = (year + 1) % 100;
						else if (mode == MODE_SET_MONTH) month = (month % 12) + 1;
						else if (mode == MODE_SET_DAY) {
							day = (day % days_in_month(month, year)) + 1;
						}
						auto_count_timer = 0;
					}
				}
				} else {
				switch2_hold_time = 0;
				auto_count_timer = 0;
			}
			if (s2 == 0 && last_s2 == 1) skip_next_s2_release = 0;
		}
		blink_enabled = (s2 == 0 && switch2_hold_time < 500);
	}

	last_s1 = s1;
	last_s2 = s2;
	last_mode = mode;
}

void set_rtc_time(uint8_t h, uint8_t m, uint8_t s) {
	if (h > 23) h = 0; if (m > 59) m = 0; if (s > 59) s = 0;
	i2c_start(RTC_WRITE);
	i2c_send(0x00);
	i2c_send(dec2bcd(s));
	i2c_send(dec2bcd(m));
	i2c_send(dec2bcd(h));
	i2c_stop();
	led7_always_on = 0;
}

// Timer1 ISR（変更なし）
ISR(TIMER1_COMPA_vect) {
	static uint16_t blink = 0;
	static uint8_t blink_state = 1;

	if (++blink >= BLINK_CYCLES) { blink_state ^= 1; blink = 0; }
	if (colon_blink_state && ++colon_timer >= COLON_CYCLES) { colon_blink_state = 0; colon_timer = 0; }
	if (led8_state && ++led8_timer >= LED8_CYCLES) { led8_state = 0; led8_timer = 0; }
	if (led7_timer) led7_timer--;
	if (format_change_timer) {
		format_change_timer--;
		if (format_change_timer == 0) showing_leading_zero = 0;
	}
	if (buzzer_timer) {
		uint8_t t = buzzer_timer--;
		if (t == BUZZER_ON1_START || t == BUZZER_ON2) buzzer_start();
		else if (t == BUZZER_OFF1 || t == BUZZER_OFF2) buzzer_stop();
	}

	read_switches();

	if (mode == MODE_DATE_DISP || mode == MODE_SET_YEAR || mode == MODE_SET_MONTH || mode == MODE_SET_DAY) {
		seg[0] = mask(day % 10);
		seg[1] = mask(day / 10);
		seg[2] = mask(month % 10) & ~LED8_MASK;
		seg[3] = mask(month / 10);
		seg[4] = mask(year % 10) & ~LED8_MASK;
		seg[5] = mask(year / 10);
		} else {
		uint8_t display_hour = hour;
		uint8_t leading_zero = 0;

		if (!is_24hour) {
			display_hour = (hour == 0 ? 12 : (hour > 12 ? hour-12 : hour));
			} else {
			if (showing_leading_zero) leading_zero = 1;
		}
		if (hour == 12 && !is_24hour) display_hour = 12;

		seg[0] = mask(sec % 10); seg[1] = mask(sec / 10);
		seg[2] = mask(min % 10); seg[3] = mask(min / 10);
		seg[4] = mask(display_hour % 10);
		seg[5] = mask(display_hour / 10 ? display_hour / 10 : (leading_zero ? 0 : 99));

		if (mode == MODE_SET_HOUR) {
			seg[4] = mask(hour % 10);
			seg[5] = mask(hour / 10 ? hour / 10 : 99);
		}
	}

	if (blink_enabled && mode != MODE_NORMAL && mode != MODE_SAVE && mode != MODE_DATE_DISP && !blink_state) {
		for (uint8_t i = blink_range[mode][0]; i < blink_range[mode][1]; i++) seg[i] = 0xFF;
	}

	uint8_t com = 0xFF;
	if (mode == MODE_NORMAL || mode == MODE_SET_HOUR || mode == MODE_SET_MIN || mode == MODE_SET_SEC || mode == MODE_SAVE) {
		if (mode != MODE_NORMAL || colon_blink_state) com &= ~COLON_MASK;
		com &= is_am ? ~AM_MASK : ~PM_MASK;
	}
	if (led8_state) com &= ~LED8_MASK;
	if (led7_timer || led7_always_on) com &= ~LED7_MASK;
	seg[6] = com;
}

// Timer2 多重化（変更なし）
ISR(TIMER2_COMPA_vect) {
	PORTB = 0xFF;
	PORTD &= 0x4C;
	PORTC &= 0xFC;
	PORTB = seg[mx];
	if ((mode == MODE_DATE_DISP || mode == MODE_SET_YEAR || mode == MODE_SET_MONTH || mode == MODE_SET_DAY) &&
	(mx == 2 || mx == 4 || mx == 0)) {
		PORTB &= ~_BV(PORTB5);
	}
	switch (mx) {
		case 0: PORTD |= (1 << PD0); break;
		case 1: PORTD |= (1 << PD1); break;
		case 2: PORTC |= (1 << PC0); break;
		case 3: PORTC |= (1 << PC1); break;
		case 4: PORTD |= (1 << PD4); break;
		case 5: PORTD |= (1 << PD5); break;
		case 6: PORTD |= (1 << PD7); break;
	}
	mx = (mx + 1) % 7;
}

int main(void) {
	DDRB = 0xFF; DDRC = 0x03; DDRD = 0xF3;
	PORTC = 0; PORTD = 0;

	TCCR2A = (1<<WGM21); TCCR2B = (1<<CS21);
	OCR2A = (F_CPU/(8*1000)-1); TIMSK2 = (1<<OCIE2A);

	TCCR1A = 0; TCCR1B = (1<<WGM12)|(1<<CS11);
	OCR1A = (F_CPU/8/1000)-1; TIMSK1 = (1<<OCIE1A);

	EICRA = (1<<ISC01); EIMSK = (1<<INT0);

	buzzer_stop();
	i2c_init();

	rtc_init_full();      // ← PONクリア付き初期化
	buzzer_timer = 50;

	_delay_us(10);
	process_rtc_update();
	sei();

	while (1) {
		if (rtc_update_flag && mode == MODE_NORMAL) {
			rtc_update_flag = 0;
			process_rtc_update();
		}
	}
}