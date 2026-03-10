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

## MQTT 
### LAB PT1

Documentation and code for Motor Control via MQTT
#### Instructions

Write a code that controls the **"on/off"** of a DC motor, and its speed using MQTT communication

#### Code
```
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT_LAB";

#define TEAM_ID     "team69"
#define DEVICE_ID   "c6_01"
#define TOPIC_MOTOR "ibero/ei2/" TEAM_ID "/" DEVICE_ID "/cmd/motor"
#define TOPIC_SPEED "ibero/ei2/" TEAM_ID "/" DEVICE_ID "/cmd/speed"
#define TOPIC_TEMP  "ibero/ei2/" TEAM_ID "/" DEVICE_ID "/read/temp"
#define TOPIC_HUM   "ibero/ei2/" TEAM_ID "/" DEVICE_ID "/read/hum"

static esp_mqtt_client_handle_t client = NULL;

extern void update_motor_from_mqtt(int is_speed, int value);

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_subscribe(client, TOPIC_MOTOR, 0);
        esp_mqtt_client_subscribe(client, TOPIC_SPEED, 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_DATA:
    
        if (strncmp(event->topic, TOPIC_MOTOR, event->topic_len) == 0) {
            char val_str[16] = {0};
            snprintf(val_str, sizeof(val_str), "%.*s", event->data_len, event->data);
            int state = atoi(val_str);
            update_motor_from_mqtt(0, state);
        } 
        else if (strncmp(event->topic, TOPIC_SPEED, event->topic_len) == 0) {
            char val_str[16] = {0};
            snprintf(val_str, sizeof(val_str), "%.*s", event->data_len, event->data);
            int speed = atoi(val_str);
            update_motor_from_mqtt(1, speed);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

void mqtt_publish_sensor_data(int is_temp, int value)
{
    if (client == NULL) return;
    
    char payload[16];
    snprintf(payload, sizeof(payload), "%d", value);
    
    if (is_temp) {
        esp_mqtt_client_publish(client, TOPIC_TEMP, payload, 0, 0, 0);
    } else {
        esp_mqtt_client_publish(client, TOPIC_HUM, payload, 0, 0, 0);
    }
}

esp_err_t mqtt_app_start(const char *broker_uri)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = broker_uri,   
        .session.keepalive = 30,
    };

    client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    
    return ESP_OK;
}
```
#### Evidence

<iframe width="560" height="315" src="https://www.youtube.com/embed/Gm7z012qhcQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### LAB PT2

Connect 2 ESPS with MQTT to control on state and brightness

#### Instrucctions 

Write a code that conects 2 ESP32 via MQTT communication, each ESP controls the **"on/off"** of 2 leds of the opposite ESP and controls the brightness of the leds with a potenciometer.

#### Code
```
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MQTT_LAB";


#define IS_DEVICE_1 0 // 0 for the 2nd esp, 1 for the first

//same pins on both esp
#define GPIO_LED_A   GPIO_NUM_20
#define GPIO_LED_B   GPIO_NUM_21
#define GPIO_BUTTON_A GPIO_NUM_15
#define GPIO_BUTTON_B GPIO_NUM_3
#define ADC_CHAN      ADC_CHANNEL_2 

#define TEAM_ID "teamWhopper"
#define BASE_TOPIC "ibero/ei2/" TEAM_ID "/"


#if IS_DEVICE_1
    //ESP 1 publish buttons and pot
    #define PUB_BTN_A BASE_TOPIC "button1"
    #define PUB_BTN_B BASE_TOPIC "button2"
    #define PUB_POT   BASE_TOPIC "pot1"
    // And sub to ESP 2
    #define SUB_BTN_A BASE_TOPIC "button3"
    #define SUB_BTN_B BASE_TOPIC "button4"
    #define SUB_POT   BASE_TOPIC "pot2"
#else
    //ESP 2 publish buttons and pot2
    #define PUB_BTN_A BASE_TOPIC "button3"
    #define PUB_BTN_B BASE_TOPIC "button4"
    #define PUB_POT   BASE_TOPIC "pot2"
    // And subscribe to ESP1
    #define SUB_BTN_A BASE_TOPIC "button1"
    #define SUB_BTN_B BASE_TOPIC "button2"
    #define SUB_POT   BASE_TOPIC "pot1"
#endif

static esp_mqtt_client_handle_t client = NULL;
static adc_oneshot_unit_handle_t adc1_handle;


static int led_a_state = 0;
static int led_b_state = 0; 
static int current_pwm_duty = 0; 


// Update pwm in the leds
static void update_leds() {
    int duty_a = led_a_state ? current_pwm_duty : 0;
    int duty_b = led_b_state ? current_pwm_duty : 0;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_a);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_b);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}


//Reading and publishing

static void publisher_task(void *pvParameters)
{
    int last_button_a = 1;
    int last_button_b = 1;
    int last_pot_8bit = -1;

    while (1)
    {
        // pull up
        int button_a = gpio_get_level(GPIO_BUTTON_A);
        int button_b = gpio_get_level(GPIO_BUTTON_B);

        // detect button A
        if (button_a == 0 && last_button_a == 1) {
            esp_mqtt_client_publish(client, PUB_BTN_A, "TOGGLE", 0, 0, 0);
        }
        last_button_a = button_a;

        // detect button B
        if (button_b == 0 && last_button_b == 1) {
            esp_mqtt_client_publish(client, PUB_BTN_B, "TOGGLE", 0, 0, 0);
        }
        last_button_b = button_b;

        // read ADC
        int adc_raw;
        adc_oneshot_read(adc1_handle, ADC_CHAN, &adc_raw);
        int pot_8bit = (adc_raw * 255) / 4095; 

        // Publishing
        if (abs(pot_8bit - last_pot_8bit) > 2) {
            char msg[10];
            sprintf(msg, "%d", pot_8bit);
            esp_mqtt_client_publish(client, PUB_POT, msg, 0, 0, 0);
            last_pot_8bit = pot_8bit;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

//mqtt handler

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        // subscribing to the other esp topics
        esp_mqtt_client_subscribe(client, SUB_BTN_A, 0);
        esp_mqtt_client_subscribe(client, SUB_BTN_B, 0);
        esp_mqtt_client_subscribe(client, SUB_POT, 0);
        break;

    case MQTT_EVENT_DATA:
    {
        //Check if is the remote A
        if (event->topic_len == strlen(SUB_BTN_A) &&
            memcmp(event->topic, SUB_BTN_A, event->topic_len) == 0)
        {
            if (event->data_len >= 6 && memcmp(event->data, "TOGGLE", 6) == 0) {
                led_a_state = !led_a_state; 
                update_leds();
                ESP_LOGI(TAG, "LED A state toggled to: %d", led_a_state);
            }
        }
        // check if is the remote B
        else if (event->topic_len == strlen(SUB_BTN_B) &&
                 memcmp(event->topic, SUB_BTN_B, event->topic_len) == 0)
        {
            if (event->data_len >= 6 && memcmp(event->data, "TOGGLE", 6) == 0) {
                led_b_state = !led_b_state;
                update_leds();
                ESP_LOGI(TAG, "LED B state toggled to: %d", led_b_state);
            }
        }
        // Check pot
        else if (event->topic_len == strlen(SUB_POT) &&
                 memcmp(event->topic, SUB_POT, event->topic_len) == 0)
        {
            char data_str[10];
            int len = event->data_len < 9 ? event->data_len : 9;
            memcpy(data_str, event->data, len);
            data_str[len] = '\0';

            current_pwm_duty = atoi(data_str);
            update_leds();
        }
        break;
    }
    default:
        break;
    }
}



esp_err_t mqtt_app_start(const char *broker_uri)
{
   
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << GPIO_LED_A) | (1ULL << GPIO_LED_B),
        .mode = GPIO_MODE_OUTPUT};
    gpio_config(&led_conf);

   
    gpio_config_t button_conf = {
        .pin_bit_mask = (1ULL << GPIO_BUTTON_A) | (1ULL << GPIO_BUTTON_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&button_conf);

    //8bit configurated pwm
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT, 
        .freq_hz = 5000};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = GPIO_LED_A,
        .duty = 0};
    ledc_channel_config(&ledc_channel1);

    ledc_channel_config_t ledc_channel2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = GPIO_LED_B,
        .duty = 0};
    ledc_channel_config(&ledc_channel2);

    //ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1};
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12};
    adc_oneshot_config_channel(adc1_handle, ADC_CHAN, &config);

    // MQTT
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = broker_uri};
    client = esp_mqtt_client_init(&cfg);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    xTaskCreate(publisher_task, "publisher", 4096, NULL, 5, NULL);

    return ESP_OK;
}
```

#### Evidence

<iframe width="560" height="315" src="https://www.youtube.com/embed/AXs9M5ey-cw" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
