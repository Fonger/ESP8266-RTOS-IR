#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rx.h"
#include "raw.h"


#define IR_RX_GPIO 12

void ir_dump_task(void *arg) {
    ir_rx_init(IR_RX_GPIO, 1024);
    ir_decoder_t *raw_decoder = ir_raw_make_decoder();

    uint16_t buffer_size = sizeof(int16_t) * 1024;
    int16_t *buffer = malloc(buffer_size);
    while (1) {
        int size = ir_recv(raw_decoder, 0, buffer, buffer_size);
        if (size <= 0)
            continue;

        printf("Decoded packet (size = %d):\n", size);
        for (int i=0; i < size; i++) {
            printf("%5d ", buffer[i]);
            if (i % 16 == 15)
                printf("\n");
        }

        if (size % 16)
            printf("\n");
    }
}
void app_main()
{
    xTaskCreate(ir_dump_task, "IR dump", 2048, NULL, tskIDLE_PRIORITY, NULL);
}