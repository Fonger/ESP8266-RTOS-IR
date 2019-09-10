#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include <ir/rx.h>
#include <ir/debug.h>

typedef enum {
    ir_rx_state_idle,
    ir_rx_state_mark,
    ir_rx_state_space,
    ir_rx_state_overflow,
} ir_rx_state_t;

typedef struct {
    uint8_t gpio;
    int16_t excess;

    uint32_t timeout;
    os_timer_t timeout_timer;

    ir_rx_state_t state;
    uint32_t last_time;

    volatile QueueHandle_t receive_queue;

    int16_t *buffer;
    uint16_t buffer_size;
    uint16_t buffer_pos;
} ir_rx_context_t;


static ir_rx_context_t ir_rx_context;

// from https://github.com/espressif/ESP8266_RTOS_SDK/issues/454
extern uint32_t esp_get_time(void);

static void ir_rx_timeout(void *arg) {
    if ((ir_rx_context.state == ir_rx_state_idle) ||
        (esp_get_time() - ir_rx_context.last_time < ir_rx_context.timeout))
        return;

    if (ir_rx_context.buffer_pos && (ir_rx_context.state != ir_rx_state_overflow)) {
        int16_t *pulses = malloc(sizeof(int16_t) * (ir_rx_context.buffer_pos + 1));
        for (int i=0; i < ir_rx_context.buffer_pos; i++)
            pulses[i] = ir_rx_context.buffer[i];
        pulses[ir_rx_context.buffer_pos] = 0;

        #ifdef IR_DEBUG
        ir_debug("Received raw pulses:\n");
        for (int16_t i = 0, *p = pulses; *p; i++, p++) {
            ir_debug1("%5d ", *p);
            if (i % 16 == 15)
                ir_debug1("\n");
        }
        ir_debug1("\n");

        ir_debug("Queue length: %d\n",
                 (int)uxQueueMessagesWaiting(ir_rx_context.receive_queue));
        #endif

        xQueueSendToBack(ir_rx_context.receive_queue, &pulses, 10);
    }

    ir_rx_context.state = ir_rx_state_idle;
    ir_rx_context.buffer_pos = 0;
}

static void IRAM_ATTR ir_rx_interrupt_handler(uint8_t gpio_num) {
    uint32_t now = esp_get_time();

    switch (ir_rx_context.state) {
        case ir_rx_state_idle:
            break;

    case ir_rx_state_mark: {
            uint32_t us = now - ir_rx_context.last_time;
            ir_rx_context.buffer[ir_rx_context.buffer_pos++] = us - ir_rx_context.excess;
            break;
        }

        case ir_rx_state_space: {
            uint32_t us = now - ir_rx_context.last_time;
            ir_rx_context.buffer[ir_rx_context.buffer_pos++] = -(us + ir_rx_context.excess);
            break;
        }

        case ir_rx_state_overflow:
            break;
    }

    if (ir_rx_context.buffer_pos >= ir_rx_context.buffer_size) {
        ir_rx_context.state = ir_rx_state_overflow;
    } else {
        ir_rx_context.state = !gpio_get_level(ir_rx_context.gpio) ? ir_rx_state_mark : ir_rx_state_space;
    }

    ir_rx_context.last_time = now;
}


void ir_rx_init(uint8_t gpio, uint16_t rx_buffer_size) {
    ir_rx_context.gpio = gpio;
    ir_rx_context.excess = 0;
    ir_rx_context.buffer = malloc(sizeof(int16_t) * rx_buffer_size);
    ir_rx_context.buffer_size = rx_buffer_size;
    ir_rx_context.buffer_pos = 0;

    ir_rx_context.state = ir_rx_state_idle;
    ir_rx_context.timeout = 20000;
    os_timer_setfn(&ir_rx_context.timeout_timer, ir_rx_timeout, NULL);
    os_timer_arm(&ir_rx_context.timeout_timer, 10, 1);

    ir_rx_context.receive_queue = xQueueCreate(10, sizeof(int16_t *));
    if (!ir_rx_context.receive_queue) {
        printf("Failed to create IR receive queue\n");
    }

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << ir_rx_context.gpio;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(ir_rx_context.gpio, ir_rx_interrupt_handler, (void *)ir_rx_context.gpio);
}

void ir_rx_set_excess(int16_t excess) {
    ir_rx_context.excess = excess;
}

int ir_recv(ir_decoder_t *decoder, uint32_t timeout, void *receive_buffer, uint16_t receive_buffer_size) {
    uint32_t start_time = esp_get_time();
    while (!timeout || (esp_get_time() - start_time) * portTICK_PERIOD_MS < timeout)
    {
        int16_t *pulses = NULL;

        uint32_t time_left = portMAX_DELAY;
        if (timeout)
            time_left = (esp_get_time() - start_time) * portTICK_PERIOD_MS;

        if (xQueueReceive(ir_rx_context.receive_queue, &pulses, time_left) != pdTRUE) {
            break;
        }

        int pulse_count = 0;
        while (pulses[pulse_count])
            pulse_count++;

        int r = decoder->decode(decoder, pulses, pulse_count, receive_buffer, receive_buffer_size);
        free(pulses);
        if (r > 0) {
            return r;
        }
        ir_debug("recv: got %d error code\n", r);
    }

    // timed out
    return -1;
}
