# Assignments

## Class Act1
### Exercise Objective
The goal of this exercise is to train you to identify logical FreeRTOS tasks from system behavior, even when no RTOS code is shown.

You should focus on:

- Timing requirements
- Blocking behavior
- Safety and criticality
- Independent execution flows

Think in terms of "what must happen independently", not functions or lines of code.

### System Description
You are given the following description of an embedded system:

#### The system:

- Reads a temperature sensor every 50 ms
- Sends sensor data via Wi-Fi every 2 seconds
- Monitors an emergency button continuously
- Blinks a status LED at 1 Hz
- Stores error messages when failures occur

| Task name | Trigger | Periodic or Event-based| Is time critical? | Can it Block safely? | What if delayed | Priority |
|-----------:|:-----:|:-----:|:-----:|:-----:|:-----:|-------------|
| Temperature sensor | 50ms Timer     | Periodic         |YES|NO|Reading error|Medium|
| WiFi     | 2s timer     | Periodic    |NO|NO|Missing data|Medium|
| Emergency Button | Press button  | Event   |YES|NO|An accident| High|
| LED Status     | 2s timer  | Periodic    |NO|YES|Just a delay of the status|Low|
| Error Message | Error    | Event    |NO|YES|We store it later|Low|

## RTOS Basics with ESP-IDF LAB

### Lab 1
#### Goal
Create two tasks:

**blink_task:** toggles an LED every 300 ms

**hello_task:** prints a message every 1 second

#### What to watch for

- Both tasks run (interleave).
- Changing priority can change which task runs “more” or “first”.
- If you remove a delay from a task, it may hog the CPU.

#### Code Lab 1
```
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_2   // CHANGE for your board

static const char *TAG = "LAB1";

static void blink_task(void *pvParameters)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static void hello_task(void *pvParameters)
{
    int n = 0;
    while (1) {
        ESP_LOGI(TAG, "hello_task says hi, n=%d", n++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 1 (two tasks)");

    // Stack size in ESP-IDF FreeRTOS is in BYTES
    xTaskCreate(blink_task, "blink_task", 2048, NULL, 5, NULL);
    xTaskCreate(hello_task, "hello_task", 2048, NULL, 5, NULL);
}
```

#### Excersises

1. **Priority experiment:** change hello_task priority from 5 to 2.
2. **Does behavior change?** Why might it (or might it not)?
3. **Starvation demo:** temporarily remove vTaskDelay(...) from hello_task.
4. What happens to **blinking**?
5. Put the delay back and explain in one sentence **why blocking helps.**

#### Answers

1. **Priority experiment (hello_task priority 5 → 2):** The blink_task runs more consistently, while hello_task still runs but may execute less frequently.

2. **Does behavior change? Why might it (or might it not)?** The behavior changes slightly because a higher-priority task is scheduled first, but both tasks still run since they block using delays.

3. **Starvation demo (remove vTaskDelay from hello_task):** hello_task runs continuously and at some point the CPU crashes.

4. **What happens to blinking?** The LED stops blinking or becomes very irregular because blink_task is starved of CPU time.

5. **Why blocking helps (one sentence):** Blocking with delays allows to switch tasks fairly and prevents one task from monopolizing the CPU.

### Lab 2
#### Goal
Use a queue to pass integers from a producer task to a consumer task.

**Why it matters:** - Queues are a clean way to pass data without sharing global variables.

#### Code Lab 2
```
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "LAB2";
static QueueHandle_t q_numbers;

static void producer_task(void *pvParameters)
{
    int value = 0;

    while (1) {
        value++;

        // Send to queue; wait up to 50ms if full
        if (xQueueSend(q_numbers, &value, pdMS_TO_TICKS(50)) == pdPASS) {
            ESP_LOGI(TAG, "Produced %d", value);
        } else {
            ESP_LOGW(TAG, "Queue full, dropped %d", value);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void consumer_task(void *pvParameters)
{
    int rx = 0;

    while (1) {
        // Wait up to 1000ms for data
        if (xQueueReceive(q_numbers, &rx, pdMS_TO_TICKS(1000)) == pdPASS) {
            ESP_LOGI(TAG, "Consumed %d", rx);
        } else {
            ESP_LOGW(TAG, "No data in 1s");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 2 (queue)");

    q_numbers = xQueueCreate(5, sizeof(int)); // length 5
    if (q_numbers == NULL) {
        ESP_LOGE(TAG, "Queue create failed");
        return;
    }

    xTaskCreate(producer_task, "producer_task", 2048, NULL, 5, NULL);
    xTaskCreate(consumer_task, "consumer_task", 2048, NULL, 5, NULL);
}
```

#### Excersises

1. **Make the producer faster:** change producer delay 200ms → 20ms.
2. When do you see **“Queue full”?**
3. Increase the queue length **5 → 20.**
4. What changes?
5. **Make the consumer “slow”:** after a successful receive, add:
6. What pattern is happening now (buffering / backlog)?

#### Answers

1. **Make the producer faster (200ms → 20ms):** The producer generates data much faster than the consumer can process it.

2. **When do you see “Queue full”?:** Right now it never appears, but it would appear if the producer fills the queue faster than the consumer removes items from it.

3. **Increase the queue length (5 → 20):** The queue can store more items before becoming full.

4. **What changes?:** Queue full never appears or takes a lot of time to appear.

5. **Make the consumer “slow”:** After adding a delay, the consumer processes items more slowly than they are produced.

6. **What pattern is happening now (buffering / backlog)?:** A backlog forms where items accumulate in the queue faster than they are consumed.

### Lab 3 Mutex

#### Goal
See a race condition happen with a shared counter, then fix it with a mutex.

#### Code Lab 3 Part A

```
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "LAB3A";

static volatile int shared_counter = 0;

static void increment_task(void *pvParameters)
{
    const char *name = (const char *)pvParameters;

    while (1) {
        // NOT safe: read-modify-write without protection
        int local = shared_counter;
        local++;
        shared_counter = local;

        if ((shared_counter % 1000) == 0) {
            ESP_LOGI(TAG, "%s sees counter=%d", name, shared_counter);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 3A (race demo)");

    xTaskCreate(increment_task, "incA", 2048, "TaskA", 5, NULL);
    xTaskCreate(increment_task, "incB", 2048, "TaskB", 5, NULL);
}
```
**Why can the counter be wrong?**

#### Answer

Because both tasks access and modify the shared variable at the same time without synchronization, causing race conditions.

#### Code Lab 3 Part B

```
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "LAB3B";

static volatile int shared_counter = 0;
static SemaphoreHandle_t counter_mutex;

static void increment_task(void *pvParameters)
{
    const char *name = (const char *)pvParameters;

    while (1) {
        xSemaphoreTake(counter_mutex, portMAX_DELAY);

        int local = shared_counter;
        local++;
        shared_counter = local;

        xSemaphoreGive(counter_mutex);

        if ((shared_counter % 1000) == 0) {
            ESP_LOGI(TAG, "%s sees counter=%d", name, shared_counter);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Lab 3B (mutex fix)");

    counter_mutex = xSemaphoreCreateMutex();
    if (counter_mutex == NULL) {
        ESP_LOGE(TAG, "Mutex create failed");
        return;
    }

    xTaskCreate(increment_task, "incA", 2048, "TaskA", 5, NULL);
    xTaskCreate(increment_task, "incB", 2048, "TaskB", 5, NULL);
}
```

#### Excersises

1. **Remove the mutex again.** Do you ever see weird behavior?
2. **Change priorities:** TaskA priority 6, TaskB priority 4.
3. What do you expect and why?
4. **In one sentence:** what does a mutex “guarantee”?

#### Answers

1. **Remove the mutex again. Do you ever see weird behavior?:** Yes, the counter sometimes skips or repeats values due to race conditions.

2. **Change priorities (TaskA = 6, TaskB = 4):** TaskA runs more often because it has higher priority.

3. **What do you expect and why?:** TaskA increments the counter more frequently since the scheduler favors higher-priority tasks.

4. **In one sentence: what does a mutex “guarantee”?:** A mutex guarantees exclusive access to a shared variable so only one task can use it at a time.

## Wi-FI Programming Lab: HTTP Server


## Assignment 1: Browser

---

### Instructions

A running system where:

Phone/PC opens http://<ESP_IP>/ and sees a webpage

- Webpage can turn LED ON/OFF
- API works:
- GET /api/led → {"state":0|1}
- POST /api/led with {"state":0|1} → {"ok":true}

With this base wee need to document the codes to:

- turn2 leds from a browser
- read a button and print it to the browser
- control the speed of a motor with a slider from the browser

Also answer the questions from the labs of HTTP.

---

### Excercise 1 code:
```
/*
 * LAB 3 — ESP32-C6 Wi-Fi + HTTP LED Control 
 *
 * Features:
 *  - Wi-Fi STA connect + reconnect using events
 *  - Wait for IP (WIFI_CONNECTED_BIT)
 *  - Simple GPIO LED control (gpio_reset_pin + gpio_set_direction)
 *  - HTTP server:
 *      GET  /         -> HTML UI
 *      GET  /api/led  -> JSON {"state":0|1}
 *      POST /api/led  -> JSON {"state":0|1} sets LED, returns {"ok":true}
 *
 */
// String handling and standard libraries
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
// FreeRTOS and event groups
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
// ESP-IDF Logging and error handling
#include "esp_log.h"
#include "esp_err.h"
// Wi-Fi and network
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
// GPIO control
#include "driver/gpio.h"
// HTTP server
#include "esp_http_server.h"

/* ===================== User config ===================== */
#define WIFI_SSID "OPPO Reno7 5G"
#define WIFI_PASS "Ar1356673"

/* LED pin */
#define LED_GPIO  21
#define LED2_GPIO 20
#define BUTTON 22
/* Reconnect policy */
#define MAX_RETRY 10

/* ===================== Globals ===================== */
static const char *TAG = "LAB_3";

// Wi-Fi event group and bit seen in Lab 2
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0


static int s_retry = 0;// Retry count for Wi-Fi reconnects

static int s_led_state = 0;               // 0=OFF, 1=ON
static int s_led2_state = 0;
static int button_state = 0;
static httpd_handle_t s_server = NULL;    // HTTP server handle

/* ===================== LED helpers ===================== */
static void led_init(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_reset_pin(LED2_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);
    s_led_state = 0;
    s_led2_state = 0;
    gpio_set_level(LED_GPIO, s_led_state);
    gpio_set_level(LED2_GPIO, s_led2_state);

    ESP_LOGI(TAG, "LED initialized on GPIO %d (state=%d)", LED_GPIO, s_led_state);
    ESP_LOGI(TAG, "LED2 initialized on GPIO %d (state=%d)", LED2_GPIO, s_led2_state);
}

/*Function to set the LED from the state*/
static void led_set(int on)
{
    s_led_state = (on != 0);
    gpio_set_level(LED_GPIO, s_led_state);
    ESP_LOGI(TAG, "LED set to %d", s_led_state);
}

static void led2_set(int on)
{
    s_led2_state = (on != 0);
    gpio_set_level(LED2_GPIO, s_led2_state);
    ESP_LOGI(TAG, "LED2 set to %d", s_led2_state);
}

static void button_init(int on){
    gpio_set_level(BUTTON) ;
    gpio_set_direction(BUTTON,GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON);
}
static int button_read(){
    return gpio_get_level(BUTTON)
}

/* ===================== HTTP handlers ===================== */
/*Remember, handlers are special functions that process HTTP requests and generate responses*/


static esp_err_t api_led_get(httpd_req_t *req)
{
    /* resp is a buffer to hold the JSON response  text in this case our jason is
       either {"state":0} or {"state":1} */
    char resp[32];
    /*sprintf is a normal printf for strings, snprintf is safer because it 
    limits the number of characters written to the buffer

    Here we build the JSON string with the current LED state. 
    */
    snprintf(resp, sizeof(resp), "{\"state\":%d}", s_led_state);

    /* Tell the client (browser/Postman/etc.) that the payload is JSON.
       This sets the HTTP header: Content-Type: application/json */
    httpd_resp_set_type(req, "application/json");

    /* Send the response body to the client.
       - 'resp' is the payload
       - HTTPD_RESP_USE_STRLEN tells ESP-IDF to compute the string length automatically
         (it treats resp as a null-terminated C string). */
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    /* Return ESP_OK so the HTTP server knows the request was handled correctly. */
    return ESP_OK;
}

static esp_err_t api_led_post(httpd_req_t *req)
{
    // Basic safety: reject empty or very large payloads
    if (req->content_len <= 0 || req->content_len > 256) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Content-Length");
        return ESP_FAIL;
    }

    char buf[257] = {0}; // +1 for null terminator
    int received = httpd_req_recv(req, buf, req->content_len);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    // Minimal parse: find "state":<number>
    int state = -1;
    char *p = strstr(buf, "\"state\"");
    if (p) {
        p = strchr(p, ':');
        if (p) state = atoi(p + 1);
    }

    if (state != 0 && state != 1) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "state must be 0 or 1");
        return ESP_FAIL;
    }

    led_set(state);
    led2_set(state);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    // Readable inline HTML
    static const char *INDEX_HTML =
        "<!doctype html>\n"
        "<html>\n"
        "<head>\n"
        "  <meta charset='utf-8'>\n"
        "  <meta name='viewport' content='width=device-width, initial-scale=1'>\n"
        "  <title>ESP32-C6 LED</title>\n"
        "</head>\n"
        "<body>\n"
        "  <h2>ESP32-C6 LED Control</h2>\n"
        "  <button onclick='setLed(1)'>ON</button>\n"
        "  <button onclick='setLed(0)'>OFF</button>\n"
        "  <button onclick='setLed2(1)'>ON</button>\n"
        "  <button onclick='setLed2(0)'>OFF</button>\n"
        "  <p id='st'>State: ?</p>\n"
        "\n"
        "  <script>\n"
        "    async function refresh(){\n"
        "      const r = await fetch('/api/led');\n"
        "      const j = await r.json();\n"
        "      document.getElementById('st').innerText = 'State: ' + j.state;\n"
        "    }\n"
        "    async function setLed(v){\n"
        "      await fetch('/api/led', {\n"
        "        method: 'POST',\n"
        "        headers: {'Content-Type':'application/json'},\n"
        "        body: JSON.stringify({state:v})\n"
        "      });\n"
        "      refresh();\n"
        "    }\n"
        "    setInterval(refresh, 1000);\n"
        "    refresh();\n"
        "  </script>\n"
        "</body>\n"
        "</html>\n";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(httpd_start(&s_server, &config));
    ESP_LOGI(TAG, "HTTP server started");

    httpd_uri_t root = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t led_get = {
        .uri      = "/api/led",
        .method   = HTTP_GET,
        .handler  = api_led_get,
        .user_ctx = NULL
    };

    httpd_uri_t led_post = {
        .uri      = "/api/led",
        .method   = HTTP_POST,
        .handler  = api_led_post,
        .user_ctx = NULL
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &led_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &led_post));

    ESP_LOGI(TAG, "Routes registered: /  ,  GET/POST /api/led");
}

/* ===================== Wi-Fi STA connect + events ===================== */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START -> esp_wifi_connect()");
        ESP_ERROR_CHECK(esp_wifi_connect());
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry < MAX_RETRY) {
            s_retry++;
            ESP_LOGW(TAG, "Disconnected. Retrying (%d/%d)...", s_retry, MAX_RETRY);
            ESP_ERROR_CHECK(esp_wifi_connect());
        } else {
            ESP_LOGE(TAG, "Failed to connect after %d retries.", MAX_RETRY);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        s_retry = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        return;
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_LOGI(TAG, "Configuring Wi-Fi STA: SSID='%s'", WIFI_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ===================== app_main ===================== */

void app_main(void)
{
    ESP_LOGI(TAG, "Lab D start: Wi-Fi + HTTP + LED control.");

    // NVS init (required by Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    // Connect to Wi-Fi (STA)
    wifi_init_sta();

    // Wait for "got IP" (connected)
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(30000)
    );

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Timeout waiting for Wi-Fi connection. Check SSID/PASS and 2.4 GHz.");
        return;
    }

    // Start peripherals + HTTP
    led_init();
    http_server_start();

    ESP_LOGI(TAG, "Open: http://<ESP_IP>/ from a device on the same network.");
}
```

#### Video

<iframe width="560" height="315" src="https://www.youtube.com/embed/7MEiwLmx20I" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

---

### Excercise 2 code:
```
/*
 * LAB 3 — ESP32-C6 Wi-Fi + HTTP LED Control 
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "esp_http_server.h"

/* ===================== User config ===================== */
/*
 * LAB 3 — ESP32-C6 Wi-Fi + HTTP LED Control 
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "esp_http_server.h"

/* ===================== User config ===================== */
#define WIFI_SSID "OPPO Reno7 5G"
#define WIFI_PASS "Ar1356673"

#define LED1_GPIO  21
#define LED2_GPIO  20
#define BUTTON     22   
#define MAX_RETRY 10

// Pines puente H TB6612
#define PWMA   0   
#define AIN1   1   
#define AIN2   2   
#define STBY   3   

#define F_PWM_HZ 2000   
#define TOP 1023       
/* ===================== Globals ===================== */
static const char *TAG = "LAB_3";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static int s_retry = 0;
static int s_led_state1 = 0;            // 0=OFF, 1=ON
static int s_led_state2 = 0;            // 0=OFF, 1=ON
static int button_state = 0;            // 0=not pressed, 1=pressed
static httpd_handle_t s_server = NULL;

/* ===================== LED helpers ===================== */
static void led_init(void)
{
    gpio_reset_pin(LED1_GPIO);
    gpio_set_direction(LED1_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED2_GPIO);
    gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);

    s_led_state1 = 0;
    s_led_state2 = 0;
    gpio_set_level(LED1_GPIO, s_led_state1);
    gpio_set_level(LED2_GPIO, s_led_state2);

    ESP_LOGI(TAG, "LEDs initialized on GPIO %d and %d (state=%d)", LED1_GPIO, LED2_GPIO, s_led_state1);
}


static void led_set1(int on)
{
    s_led_state1 = (on != 0);
    gpio_set_level(LED1_GPIO, s_led_state1);
}

static void led_set2(int on)
{
    s_led_state2 = (on != 0);
    gpio_set_level(LED2_GPIO, s_led_state2);
}

static void button_init(void)
{
    gpio_reset_pin(BUTTON);
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON);
}
static int button_read(void)
{
    return gpio_get_level(BUTTON);
}
/* ===================== HTTP Handlers ===================== */


static esp_err_t root_get_handler(httpd_req_t *req)
{
    char response[2048];

    // Read REAL GPIO button
    int level = button_read();

    // If using pull-up: pressed = 0
    const char *state_str = (level == 0) ? "RELEASED" : "PRESSED";

    snprintf(response, sizeof(response),
        "<!doctype html>\n"
        "<html><head>\n"
        "  <meta charset='utf-8'>\n"
        "  <meta name='viewport' content='width=device-width, initial-scale=1'>\n"
        "  <meta http-equiv='refresh' content='1'>\n"
        "  <title>ESP32-C6 LED</title>\n"
        "</head><body>\n"
        "  <h2>ESP32-C6 LED Control (GET-only)</h2>\n"
        "  <p>Use the buttons below to send HTTP GET requests to the ESP32.</p>\n"
        "  <p>\n"
        "    <a href='/ledon1'><button>LED1 ON</button></a>\n"
        "    <a href='/ledoff1'><button>LED1 OFF</button></a>\n"
        "  </p>\n"
        "  <p>\n"
        "    <a href='/ledon2'><button>LED2 ON</button></a>\n"
        "    <a href='/ledoff2'><button>LED2 OFF</button></a>\n"
        "  </p>\n"
        "  <p>\n"
        "    <strong>External Button STATE:</strong> %s\n"
        "  </p>\n"
        "  <p>Direct URLs:</p>\n"
        "  <ul>\n"
        "    <li><code>/ledon1</code></li>\n"
        "    <li><code>/ledoff1</code></li>\n"
        "    <li><code>/ledon2</code></li>\n"
        "    <li><code>/ledoff2</code></li>\n"
        "  </ul>\n"
        "</body></html>\n",
        state_str
    );

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}


static esp_err_t ledon_get_handler1(httpd_req_t *req)
{
    // Application logic: turn LED ON
    led_set1(1);

    // Response: simple HTML confirmation page
    static const char *RESP =
        "<!doctype html><html><body>"
        "<h3>LED is now: ON</h3>"
        "<p><a href='/'><button>Back</button></a></p>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, RESP, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ledon_get_handler2(httpd_req_t *req)
{
    // Application logic: turn LED ON
    led_set2(1);
    // Response: simple HTML confirmation page
    static const char *RESP =
        "<!doctype html><html><body>"
        "<h3>LED is now: ON</h3>"
        "<p><a href='/'><button>Back</button></a></p>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, RESP, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ledoff_get_handler1(httpd_req_t *req)
{
    // Application logic: turn LED OFF
    led_set1(0);
    // Response: simple HTML confirmation page
    static const char *RESP =
        "<!doctype html><html><body>"
        "<h3>LED is now: OFF</h3>"
        "<p><a href='/'><button>Back</button></a></p>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, RESP, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ledoff_get_handler2(httpd_req_t *req)
{
    // Application logic: turn LED OFF
    led_set2(0);

    // Response: simple HTML confirmation page
    static const char *RESP =
        "<!doctype html><html><body>"
        "<h3>LED is now: OFF</h3>"
        "<p><a href='/'><button>Back</button></a></p>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, RESP, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t button_1_get_handler(httpd_req_t *req)
{
    button_state = 1;
    // Response: simple HTML confirmation page
    static const char *RESP =
        "<!doctype html><html><body>"
        "<h3>BUTTON is now: PRESSED</h3>"
        "<p><a href='/'><button>Back</button></a></p>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, RESP, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t button_0_get_handler(httpd_req_t *req)
{
    button_state = 0;
    // Response: simple HTML confirmation page
    static const char *RESP =
        "<!doctype html><html><body>"
        "<h3>BUTTON is now: RELEASED</h3>"
        "<p><a href='/'><button>Back</button></a></p>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, RESP, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* ===================== HTTP Server Start + Route Registration ===================== */

static void http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(httpd_start(&s_server, &config));
    ESP_LOGI(TAG, "HTTP server started (port %d)", config.server_port);

    httpd_uri_t root = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ledon1 = {
        .uri      = "/ledon1",
        .method   = HTTP_GET,
        .handler  = ledon_get_handler1,
        .user_ctx = NULL
    };

        httpd_uri_t ledon2 = {
        .uri      = "/ledon2",
        .method   = HTTP_GET,
        .handler  = ledon_get_handler2,
        .user_ctx = NULL
    };

    httpd_uri_t ledoff1 = {
        .uri      = "/ledoff1",
        .method   = HTTP_GET,
        .handler  = ledoff_get_handler1,
        .user_ctx = NULL
    };

    httpd_uri_t ledoff2 = {
        .uri      = "/ledoff2",
        .method   = HTTP_GET,
        .handler  = ledoff_get_handler2,
        .user_ctx = NULL
    };

    httpd_uri_t button_1 = {
        .uri      = "/button_1",
        .method   = HTTP_GET,
        .handler  = button_1_get_handler,
        .user_ctx = NULL
    };

    httpd_uri_t button_0 = {
        .uri      = "/button_0",
        .method   = HTTP_GET,
        .handler  = button_0_get_handler,
        .user_ctx = NULL
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &ledon1));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &ledoff1));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &ledon2));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &ledoff2));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &button_1));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &button_0));

    ESP_LOGI(TAG, "Routes: GET /, GET /ledon1, GET /ledoff1, GET /ledon2, GET /ledoff2, GET /button_1, GET /button_0");
}

/* ===================== Wi-Fi STA connect + events ===================== */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START -> connecting...");
        ESP_ERROR_CHECK(esp_wifi_connect());
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry < MAX_RETRY) {
            s_retry++;
            ESP_LOGW(TAG, "Disconnected. Retrying (%d/%d)...", s_retry, MAX_RETRY);
            ESP_ERROR_CHECK(esp_wifi_connect());
        } else {
            ESP_LOGE(TAG, "Failed to connect after %d retries.", MAX_RETRY);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        s_retry = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        return;
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_LOGI(TAG, "Configuring Wi-Fi STA: SSID='%s'", WIFI_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ===================== app_main ===================== */

void app_main(void)
{
    ESP_LOGI(TAG, "Lab 3 start: Wi-Fi + HTTP.");

    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    wifi_init_sta();

    // Wait for IP
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(30000)
    );

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Timeout waiting for Wi-Fi connection. Check SSID/PASS and 2.4 GHz.");
        return;
    }

    led_init();
    http_server_start();

    ESP_LOGI(TAG, "Open: http://<ESP_IP>/ ");
    ESP_LOGI(TAG, "Direct control: http://<ESP_IP>/ledon  and  http://<ESP_IP>/ledoff");
}
```
#### videos:

<iframe width="560" height="315" src="https://www.youtube.com/embed/4lUXgxR2c8A" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>




### Excercise 3 code:

```
hi

```
#### Images and videos:

---