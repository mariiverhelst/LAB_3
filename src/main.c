#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_task_wdt.h"
#include "esp_rom_sys.h"

//DEFINES

#define PIN_LED        GPIO_NUM_26
#define PIN_RELAY      GPIO_NUM_25

#define PIN_IN1        GPIO_NUM_18
#define PIN_IN2        GPIO_NUM_19
#define PIN_IN3        GPIO_NUM_21
#define PIN_IN4        GPIO_NUM_22

#define ADC_CH_TEMP    ADC_CHANNEL_6
#define ADC_CH_LDR     ADC_CHANNEL_7

#define UART_USED      UART_NUM_0

#define LDR_MIN_VAL    150
#define LDR_MAX_VAL    3500

#define LAMP_INV       0

/* PWM */
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_MODE       LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_RES        LEDC_TIMER_10_BIT
#define PWM_FREQ       500

//VARIABLES GLOBALES

adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_calibration;

volatile float temp_ref = 25.0f;
volatile float temp_actual = 0.0f;

float luz_percent = 0.0f;
float led_output = 0.0f;

//MOTOR

typedef enum {
    ST_STOP = 0,
    ST_HEAT,
    ST_VENT_L,
    ST_VENT_M,
    ST_VENT_H
} motor_state_e;

typedef struct {
    int a,b,c,d;
} step_phase_t;

static const step_phase_t motor_seq[8] = {
    {1,0,0,0},{1,0,1,0},{0,0,1,0},{0,1,1,0},
    {0,1,0,0},{0,1,0,1},{0,0,0,1},{1,0,0,1}
};

volatile motor_state_e motor_state = ST_STOP;
volatile int lamp_state = 0;

//UART

void init_uart() {

    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_USED, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_USED, &cfg);
}

void task_uart_rx(void *arg) {

    char buffer[64];
    int index = 0;

    while (1) {

        uint8_t c;
        int len = uart_read_bytes(UART_USED, &c, 1, pdMS_TO_TICKS(10));

        if (len > 0) {

            uart_write_bytes(UART_USED, (const char*)&c, 1);

            if (c == '\n' || c == '\r') {

                if (index > 0) {

                    buffer[index] = '\0';

                    if (strncmp(buffer, "SET_TEMP:", 9) == 0) {

                        float val = atof(buffer + 9);

                        if (val > -50 && val < 200) {

                            temp_ref = val;

                            char msg[64];
                            sprintf(msg, "\r\n>> Tc = %.1f C\r\n\r\n", temp_ref);

                            uart_write_bytes(UART_USED, msg, strlen(msg));
                        }
                    }
                    index = 0;
                }

            } else if (index < 63) {
                buffer[index++] = (char)c;
            }
        }

        vTaskDelay(1);
    }
}

//ADC

void init_adc() {

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_oneshot_config_channel(adc_handle, ADC_CH_TEMP, &ch_cfg);
    adc_oneshot_config_channel(adc_handle, ADC_CH_LDR, &ch_cfg);

    adc_cali_line_fitting_config_t cal_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_cali_create_scheme_line_fitting(&cal_cfg, &adc_calibration);
}

//SENSORES

float get_temp() {

    int raw, mv, sum = 0;

    for (int i = 0; i < 64; i++) {

        adc_oneshot_read(adc_handle, ADC_CH_TEMP, &raw);
        adc_cali_raw_to_voltage(adc_calibration, raw, &mv);

        sum += mv;
    }

    return ((sum / 64) / 10.0f) - 7.3;
}

float get_light() {

    int raw;
    long sum = 0;

    for (int i = 0; i < 32; i++) {

        adc_oneshot_read(adc_handle, ADC_CH_LDR, &raw);
        sum += raw;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    raw = sum / 32;

    if (raw < LDR_MIN_VAL) raw = LDR_MIN_VAL;
    if (raw > LDR_MAX_VAL) raw = LDR_MAX_VAL;

    return ((float)(raw - LDR_MIN_VAL) / (LDR_MAX_VAL - LDR_MIN_VAL)) * 100.0f;
}

//LED

void update_led(float p) {

    if (p < 20) led_output = 100;
    else if (p < 30) led_output = 80;
    else if (p < 40) led_output = 60;
    else if (p < 60) led_output = 50;
    else if (p < 80) led_output = 30;
    else led_output = 0;

    uint32_t duty = (led_output * 1023) / 100;

    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

//LAMPARA

void init_lamp() {

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_RELAY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&cfg);

    gpio_set_level(PIN_RELAY, LAMP_INV ? 1 : 0);
}

void set_lamp(int on) {

    lamp_state = on ? 1 : 0;

    int lvl = LAMP_INV ? !lamp_state : lamp_state;

    gpio_set_level(PIN_RELAY, lvl);
}

//MOTOR

void init_motor() {

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL<<PIN_IN1)|(1ULL<<PIN_IN2)|(1ULL<<PIN_IN3)|(1ULL<<PIN_IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&cfg);

    gpio_set_level(PIN_IN1,0);
    gpio_set_level(PIN_IN2,0);
    gpio_set_level(PIN_IN3,0);
    gpio_set_level(PIN_IN4,0);
}

void motor_stop_all() {

    gpio_set_level(PIN_IN1,0);
    gpio_set_level(PIN_IN2,0);
    gpio_set_level(PIN_IN3,0);
    gpio_set_level(PIN_IN4,0);
}

//LOGICA CONTROL

void control_temp(float t, float ref) {

    if (t < ref - 1) {
        motor_state = ST_HEAT;
        set_lamp(1);

    } else if (t <= ref + 1) {
        motor_state = ST_STOP;
        set_lamp(0);

    } else if (t < ref + 3) {
        motor_state = ST_VENT_L;
        set_lamp(0);

    } else if (t <= ref + 5) {
        motor_state = ST_VENT_M;
        set_lamp(0);

    } else {
        motor_state = ST_VENT_H;
        set_lamp(0);
    }
}

//TASK MOTOR

void task_motor(void *arg) {

    int step_idx = 0;
    int count = 0;

    while (1) {

        motor_state_e st = motor_state;

        if (st == ST_STOP) {
            motor_stop_all();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int dir;
        uint32_t delay_us;

        switch (st) {

            case ST_HEAT:     dir=1; delay_us=10000; break;
            case ST_VENT_L:   dir=0; delay_us=10000; break;
            case ST_VENT_M:   dir=0; delay_us=3333;  break;
            case ST_VENT_H:   dir=0; delay_us=1667;  break;

            default:
                motor_stop_all();
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
        }

        gpio_set_level(PIN_IN1, motor_seq[step_idx].a);
        gpio_set_level(PIN_IN2, motor_seq[step_idx].b);
        gpio_set_level(PIN_IN3, motor_seq[step_idx].c);
        gpio_set_level(PIN_IN4, motor_seq[step_idx].d);

        step_idx = dir ? (step_idx+1)%8 : (step_idx+7)%8;

        esp_rom_delay_us(delay_us);

        if (++count >= 100) {
            count = 0;
            vTaskDelay(1);
        }
    }
}

//MAIN

void app_main() {

    esp_task_wdt_deinit();

    init_uart();
    init_adc();
    init_motor();
    init_lamp();

    ledc_timer_config_t tcfg = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ,
        .duty_resolution = PWM_RES,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg = {
        .gpio_num = PIN_LED,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ccfg);

    xTaskCreatePinnedToCore(task_motor,"motor",4096,NULL,5,NULL,1);
    xTaskCreatePinnedToCore(task_uart_rx,"uart",2048,NULL,4,NULL,0);

    vTaskDelay(pdMS_TO_TICKS(100));

    uart_write_bytes(UART_USED,
        "\r\n==============================\r\n"
        " SISTEMA DOMOTICO ESP32\r\n"
        "==============================\r\n"
        " Use SET_TEMP:XX\r\n"
        "==============================\r\n\r\n",
        118);

    char out[200];

    while (1) {

        temp_actual = get_temp();
        luz_percent = get_light();

        control_temp(temp_actual, temp_ref);
        update_led(luz_percent);

        char mtxt[64];

        switch(motor_state) {
            case ST_STOP: strcpy(mtxt,"Motor apagado"); break;
            case ST_HEAT: strcpy(mtxt,"Motor horario 100 steps/s"); break;
            case ST_VENT_L: strcpy(mtxt,"Motor antihorario 100 steps/s"); break;
            case ST_VENT_M: strcpy(mtxt,"Motor antihorario 300 steps/s"); break;
            case ST_VENT_H: strcpy(mtxt,"Motor antihorario 600 steps/s"); break;
            default: strcpy(mtxt,"Motor desconocido"); break;
        }

        sprintf(out,
            "Tc: %.1f C | T: %.2f C | LUZ: %.1f%% | LED: %.1f%% | LAMP:%s | MOTOR:%s\r\n",
            temp_ref,
            temp_actual,
            led_output,
            luz_percent,
            lamp_state ? "ON":"OFF",
            mtxt
        );

        uart_write_bytes(UART_USED, out, strlen(out));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}