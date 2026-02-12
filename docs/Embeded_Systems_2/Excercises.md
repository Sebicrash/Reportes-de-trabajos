# Excercises

## Task Excercise
### Goal

Use the task´s learned commands to create a program that uses 7 different tasks

### What to watch for

-Task 1: Heartbeat

-Task 2: Alive task

-Task 3: Queue Struct Send

-Task 4: Queue Struct Receive

-Task 5 and 6: Mutex reading a button

-Task 7: Error loggin for task 1-6

### Code Lab 1
```
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_20
#define BUTTON   GPIO_NUM_21 

static const char *TAG = "LAB1";

static SemaphoreHandle_t btn_mutex = NULL;
static uint32_t task_counters[6] = {0};

typedef struct {
    char id[20];
    int value;
} DataMessage;

QueueHandle_t structQueue;

static void blink_task(void *pvParameters)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        task_counters[0]++;
        
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void Live(void *pvParameters)
{
    while (1) {
        task_counters[1]++;
        ESP_LOGI(TAG, "The pacient is alive (BPM=95)");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void sender_task(void *pvParameters)
{
    DataMessage myData;
    strcpy(myData.id, "CHARLY");
    int count = 0;

    while (1) {
        task_counters[2]++;
        myData.value = count;
        
        if (xQueueSend(structQueue, &myData, pdMS_TO_TICKS(100)) == pdPASS) {
             
        }
        
        count++;
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

static void receiver_task(void *pvParameters)
{
    DataMessage receivedData;

    while (1) {
        task_counters[3]++;

        if (xQueueReceive(structQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "QUEUE RECIEVED, PATIENT: %s, VALUE: %d", receivedData.id, receivedData.value);
        }
    }
}

static void mutex_button_task(void *pvParameters) {
    int task_idx = (int)pvParameters; 
    const char* message = (task_idx == 4) ? "I hate charly" : "I hate Javi";

    while (1) {
        task_counters[task_idx]++;
        
        if (xSemaphoreTake(btn_mutex, portMAX_DELAY) == pdTRUE) {
            
            if (gpio_get_level(BUTTON) == 0) {
                 ESP_LOGI(TAG, "%s", message);
                 vTaskDelay(pdMS_TO_TICKS(200)); 
            }
            
            xSemaphoreGive(btn_mutex); 
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void monitor_task(void *pvParameters) {
    uint32_t last_counters[6] = {0};
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        for (int i = 0; i < 6; i++) {
            if (task_counters[i] == last_counters[i]) {
                ESP_LOGE("MONITOR", "ERROR: Tarea %d detenida", i);
            } else {
                last_counters[i] = task_counters[i];
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 1");

    gpio_reset_pin(BUTTON);
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON);

    structQueue = xQueueCreate(10, sizeof(DataMessage));
    btn_mutex = xSemaphoreCreateMutex();

    if (structQueue == NULL || btn_mutex == NULL) {
        ESP_LOGE(TAG, "Error creating Queue or Mutex");
        return;
    }

    xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL);
    xTaskCreate(Live, "Live", 2048, NULL, 5, NULL);
    xTaskCreate(sender_task, "sender", 2048, NULL, 5, NULL);
    xTaskCreate(receiver_task, "receiver", 2048, NULL, 5, NULL);
    xTaskCreate(mutex_button_task, "BtnCharly", 2048, (void*)4, 5, NULL);
    xTaskCreate(mutex_button_task, "BtnJavi", 2048, (void*)5, 5, NULL);
    xTaskCreate(monitor_task, "Monitor", 2048, NULL, 6, NULL);
}
```
### Evidence

<iframe width="560" height="315" src="https://www.youtube.com/embed/btbJw3lPjE4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

