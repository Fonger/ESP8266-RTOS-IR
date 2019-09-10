#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <ir/tx.h>
#include <ir/raw.h>

static const int16_t AC_POWER[] = {
    9000, -4500, 562, -562, 562, -562, 562, -562, 562, -562, 562, -562,
    562, -562, 562, -562, 562, -1688, 562, -1688, 562, -1688, 562, -1688,
    562, -1688, 562, -1688, 562, -1688, 562, -1688, 562, -1688, 562, -562,
    562, -562, 562, -562, 562, -562, 562, -562, 562, -562, 562, -562,
    562, -562, 562, -1688, 562, -1688, 562, -1688, 562, -1688, 562, -1688,
    562, -1688, 562, -1688, 562, -1688, 562};

// IR signal pin must be GPIO14 (D5 pin)

void ir_tx_task(void *arg)
{
    ir_tx_init();
    while (1)
    {
        ir_raw_send(AC_POWER, sizeof(AC_POWER) / sizeof(*AC_POWER));
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}
void app_main()
{
    xTaskCreate(ir_tx_task, "IR TX", 2048, NULL, tskIDLE_PRIORITY, NULL);
}