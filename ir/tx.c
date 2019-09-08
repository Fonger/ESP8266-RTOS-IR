#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/hw_timer.h"

#include "esp8266/pin_mux_register.h"
#include "esp8266/i2s_register.h"
#include "esp8266/i2s_struct.h"
#include "esp8266/timer_struct.h"
#include "esp8266/gpio_struct.h"
#include "esp_system.h"

#include "tx.h"

// The following definitions is taken from ESP8266_MP3_DECODER demo
// https://github.com/espressif/ESP8266_RTOS_SDK/blob/master/components/esp8266/driver/i2s.c
// It is required to set clock to I2S subsystem
void rom_i2c_writeReg_Mask(uint8_t block, uint8_t host_id,
        uint8_t reg_add, uint8_t msb, uint8_t lsb, uint8_t data);

#ifndef i2c_bbpll
#define i2c_bbpll                               0x67
#define i2c_bbpll_en_audio_clock_out            4
#define i2c_bbpll_en_audio_clock_out_msb        7
#define i2c_bbpll_en_audio_clock_out_lsb        7
#define i2c_bbpll_hostid                        4
#endif

#define IR_GPIO_NUM 14 // Must be GPIO14 MTMS pin (I2S CLK pin)

EventGroupHandle_t tx_flags;
#define TX_FLAG_READY (1 << 0)

#define ALWAYS_INLINE __attribute__((always_inline))

static void ALWAYS_INLINE gen_carrier() {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);
    I2S0.conf.rx_start = 1;
}


static void ALWAYS_INLINE clr_carrier() {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
    GPIO.out_w1tc |= (0x1 << IR_GPIO_NUM); // equivalent to gpio_set_level(IR_GPIO_NUM, 0);
    I2S0.conf.rx_start = 0;
}


static void ALWAYS_INLINE hw_timer_pause() {
    frc1.ctrl.en = 0;
}


static void ALWAYS_INLINE hw_timer_arm(uint32_t us) {
    // equivalent to hw_timer_alarm_us() with less overhead
    frc1.ctrl.reload = 0;
    frc1.ctrl.div = TIMER_CLKDIV_16;
    frc1.ctrl.intr_type = TIMER_EDGE_INT;
    frc1.load.data = ((TIMER_BASE_CLK >> frc1.ctrl.div) / 1000000) * us;
    frc1.ctrl.en = 1;
}


static void IRAM_ATTR ir_tx_timer_handler(ir_encoder_t *encoder) {
    hw_timer_pause();
    int16_t pulse = encoder->get_next_pulse(encoder);
    if (pulse == 0) {
        // Done with transmission
        clr_carrier();
        encoder->free(encoder);

        hw_timer_deinit();
        BaseType_t task_woken = 0;
        xEventGroupSetBitsFromISR(tx_flags, TX_FLAG_READY, &task_woken);

        portEND_SWITCHING_ISR(task_woken);
        return;
    }

    if (pulse > 0) {
        gen_carrier();
    } else {
        clr_carrier();
    }
    hw_timer_arm(abs(pulse));
}


void ir_tx_init() {
    // Start I2C clock for I2S subsystem
    rom_i2c_writeReg_Mask(
        i2c_bbpll, i2c_bbpll_hostid,
        i2c_bbpll_en_audio_clock_out,
        i2c_bbpll_en_audio_clock_out_msb,
        i2c_bbpll_en_audio_clock_out_lsb,
        1
    );

    // Clear I2S subsystem
    I2S0.conf.val &= ~I2S_I2S_RESET_MASK;
    I2S0.conf.val |= I2S_I2S_RESET_MASK;
    I2S0.conf.val &= ~I2S_I2S_RESET_MASK;

    // Set i2s clk freq for IR pulse
    I2S0.conf.bck_div_num = 62 & I2S_BCK_DIV_NUM;
    I2S0.conf.clkm_div_num = 2 & I2S_CLKM_DIV_NUM;
    I2S0.conf.bits_mod = 1 & I2S_BITS_MOD;

    // Set normal GPIO mode for IR space
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << IR_GPIO_NUM;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(IR_GPIO_NUM, 0);

    tx_flags = xEventGroupCreate();
    xEventGroupSetBits(tx_flags, TX_FLAG_READY);
}


int ir_tx_send(ir_encoder_t *encoder) {
    if (xEventGroupWaitBits(tx_flags, TX_FLAG_READY, pdTRUE, pdTRUE, portMAX_DELAY) == 0)
        return -1;

    hw_timer_init((void (*)(void *))ir_tx_timer_handler, encoder);

    ir_tx_timer_handler(encoder);

    return 0;
}

