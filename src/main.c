#include <asf.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "tasks.c"
#include "timers.h"
#include "queue.c"
#include "semphr.h"
#include "list.c"
#include "port.c"
#include "heap_3.c"
#include "FreeRTOSConfig.h"
#include "timers.c"

// Define semaphore
SemaphoreHandle_t xSemaphore;

// Define sensor pins
#define PIR_SENSOR_PIN    PIN1_bm  // Sensor PIR terhubung ke PIN1
#define LDR_SENSOR_PIN    PIN0_bm  // Sensor LDR terhubung ke PIN0

// Define PWM parameters
#define PWM_MAX_VALUE 1000

// Buffer untuk tampilan string
static char strbuf[128];

// -----------------------------------------------------------------------------
// TAMBAHAN: Definisi & Fungsi USART
// -----------------------------------------------------------------------------
#define USART_BAUDRATE         9600
// Misal clock = 2MHz internal; BSEL=12, BSCALE=0 -> Baud ~9600
#define USARTC0_BAUDCTRLA_VAL  12
#define USARTC0_BAUDCTRLB_VAL  0

// Inisialisasi USART di PORTC (TX=PC3, RX=PC2)
static void usart_init(void)
{
	// TX pin (PC3)
	PORTC.DIRSET = PIN3_bm;
	PORTC.OUTSET = PIN3_bm;  // idle TX = HIGH

	// RX pin (PC2)
	PORTC.DIRCLR = PIN2_bm;

	// Nonaktifkan interrupt
	USARTC0.CTRLA = 0;
	// Mode: 8 bit, no parity, 1 stop bit
	USARTC0.CTRLC = USART_CHSIZE_8BIT_gc;

	// Baud rate register
	USARTC0.BAUDCTRLA = USARTC0_BAUDCTRLA_VAL;
	USARTC0.BAUDCTRLB = USARTC0_BAUDCTRLB_VAL;

	// Enable TX dan RX
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

static void usart_sendChar(char c)
{
	while (!(USARTC0.STATUS & USART_DREIF_bm)) {
		// Tunggu buffer data kosong
	}
	USARTC0.DATA = c;
}

static void usart_sendString(const char *str)
{
	while (*str) {
		usart_sendChar(*str++);
	}
}
// -----------------------------------------------------------------------------
// Akhir Penambahan Kode USART
// -----------------------------------------------------------------------------

// Task untuk membaca nilai LDR
static portTASK_FUNCTION_PROTO(vReadLdr, t_);

// Task untuk mendeteksi gerakan dengan PIR
static portTASK_FUNCTION_PROTO(vMotionDetection, t_);

// Task untuk mengatur intensitas LED berdasarkan nilai LDR
static portTASK_FUNCTION_PROTO(vLedControlBasedOnLdr, t_);

// Task untuk mengirim data LDR via USART
static portTASK_FUNCTION_PROTO(vSendLdrData, t_);

// Task untuk mengirim status PIR via USART
static portTASK_FUNCTION_PROTO(vSendPirData, t_);

// Inisialisasi PWM
void PWM_Init(void) {
	PORTC.DIR |= PIN0_bm;  // Set pin PC0 untuk PWM output
	// Catatan: TCC0.CTRLA = PIN1_bm hanya sekadar contoh;
	// Pastikan sesuai prescaler yg diinginkan
	TCC0.CTRLA = PIN1_bm;
	TCC0.CTRLB = (PIN4_bm) | (PIN2_bm) | (PIN1_bm);
	TCC0.PER = PWM_MAX_VALUE;
	TCC0.CCA = 0;
}

void led_init() {
	PORTC.DIRSET = PIN3_bm; // Set PC3 as output
}

void led_control(bool state) {
	if (state) {
		PORTC.OUTSET = PIN3_bm; // Turn on the LED
		} else {
		PORTC.OUTCLR = PIN3_bm; // Turn off the LED
	}
}

// Inisialisasi ADC (LDR)
void ADC_Init(void) {
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&ADCA, &adc_conf);
	adcch_read_configuration(&ADCA, ADC_CH0, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_write_configuration(&ADCA, &adc_conf);

	adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);

	adc_enable(&ADCA);
}

// Membaca nilai ADC (LDR)
uint16_t ADC_Read(void) {
	adc_start_conversion(&ADCA, ADC_CH0);
	adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
	return adc_get_result(&ADCA, ADC_CH0);
}

// Inisialisasi sensor PIR
void PIR_Init(void) {
	PORTC.DIRCLR = PIR_SENSOR_PIN; // Set pin PIR sebagai input
}

// Membaca status sensor PIR
bool PIR_Read(void) {
	return (PORTC.IN & PIR_SENSOR_PIN) ? 1 : 0;  // 1 jika ada gerakan, 0 jika tidak ada
}

// Fungsi untuk menampilkan status sensor PIR
void display_pir_value(bool pir_sensor_active) {
	if (pir_sensor_active) {
		gfx_mono_draw_string("PIR: Active  ", 0, 16, &sysfont);
		} else {
		gfx_mono_draw_string("PIR: Inactive", 0, 16, &sysfont);
	}
}

// Fungsi untuk menampilkan nilai LDR dan PWM
void display_ldr_value(uint16_t ldr_value, uint16_t pwm_value) {
	snprintf(strbuf, sizeof(strbuf), "LDR: %4d PWM: %4d", ldr_value, pwm_value);
	gfx_mono_draw_string(strbuf, 0, 0, &sysfont);
}

// Task untuk membaca nilai LDR dan mengatur PWM
static portTASK_FUNCTION(vReadLdr, t_) {
	uint16_t ldr_value;
	uint16_t pwm_value;

	while (1) {
		// Baca nilai ADC dari LDR
		ldr_value = ADC_Read();

		// Convert LDR value to PWM value (0-1000)
		if (ldr_value > 3500) {
			pwm_value = 1000;  // Maximum brightness
			} else if (ldr_value < 1000) {
			pwm_value = 0;     // Minimum brightness
			} else {
			// Map LDR value to 8 discrete steps
			uint8_t step = (ldr_value - 1000) / 312;  // Divide range into 8 steps
			pwm_value = step * 125;  // Each step increases PWM by 125
		}

		// Atur duty cycle PWM
		TCC0.CCA = pwm_value;

		// Tampilkan nilai LDR dan PWM
		display_ldr_value(ldr_value, pwm_value);

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

// Task untuk mendeteksi gerakan dengan PIR
static portTASK_FUNCTION(vMotionDetection, t_) {
	bool pir_sensor_active;
	static bool prev_led_state = false;

	while (1) {
		// Baca status sensor PIR
		pir_sensor_active = PIR_Read();

		if (pir_sensor_active) {
			if (!prev_led_state) {
				// Turn on PWM when motion is first detected
				PWM_Init();  // Reinitialize PWM
				prev_led_state = true;
			}
			display_pir_value(true);
			led_control(true);
			} else {
			if (prev_led_state) {
				// Turn off PWM when no motion
				TCC0.CCA = 0;
				prev_led_state = false;
			}
			display_pir_value(false);
			led_control(false);
		}

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

// Task untuk membaca nilai LDR dan mengirim ke USART
static portTASK_FUNCTION(vSendLdrData, t_) {
    uint16_t ldr_value;

    while (1) {
        // Baca nilai ADC dari LDR
        ldr_value = ADC_Read();

        // Kirim data LDR via USART
        snprintf(strbuf, sizeof(strbuf), "LDR Value: %u\r\n", ldr_value);
        usart_sendString(strbuf);

        // Delay task untuk mengurangi beban USART
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// Task untuk membaca status PIR dan mengirim ke USART
static portTASK_FUNCTION(vSendPirData, t_) {
    bool pir_sensor_active;

    while (1) {
        // Baca status sensor PIR
        pir_sensor_active = PIR_Read();

        // Kirim status PIR via USART
        if (pir_sensor_active) {
            usart_sendString("PIR: Active\r\n");
        } else {
            usart_sendString("PIR: Inactive\r\n");
        }

        // Delay task untuk mengurangi beban USART
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

int main(void) {
    /* Initialize system and peripherals */
    sysclk_init();
    board_init();
    gfx_mono_init();
    gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

    // Inisialisasi USART
    usart_init();

    // Initialize ADC (LDR) dan PIR sensor
    ADC_Init();
    PIR_Init();
    led_init();
    PWM_Init();

    // Create tasks
    xTaskCreate(vReadLdr, "LDR Task", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vMotionDetection, "PIR Task", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vSendLdrData, "LDR USART", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vSendPirData, "PIR USART", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();

    while (1) {
        // Should never get here
    }
}