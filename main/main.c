#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "Semaforos_GPIO";

// Pines disponibles
#define GPIO_PIN1 17
#define GPIO_PIN2 18

// Semáforos por cada pin
SemaphoreHandle_t sem_pin1;
SemaphoreHandle_t sem_pin2;

// Prototipo de función
void led_task(void *param);

typedef struct {
    int blink_time_ms;   // Tiempo entre encendidos (período)
    int total_time_ms;   // Duración total de la tarea
    const char* task_name;
} task_config_t;

void led_task(void *param)
{
    task_config_t *config = (task_config_t *) param;
    int pin = -1;

    // Intenta tomar un semáforo (cualquiera de los dos)
    while (1) {
        if (xSemaphoreTake(sem_pin1, 0) == pdTRUE) {
            pin = GPIO_PIN1;
            break;
        } else if (xSemaphoreTake(sem_pin2, 0) == pdTRUE) {
            pin = GPIO_PIN2;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Espera y reintenta
    }

    // Configurar el pin
    gpio_pad_select_gpio(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "%s usando GPIO %d", config->task_name, pin);

    int elapsed = 0;
    while (elapsed < config->total_time_ms) {
        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(config->blink_time_ms / 2));
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(config->blink_time_ms / 2));
        elapsed += config->blink_time_ms;
    }

    ESP_LOGI(TAG, "%s terminó. Liberando GPIO %d", config->task_name, pin);

    // Liberar semáforo
    if (pin == GPIO_PIN1) {
        xSemaphoreGive(sem_pin1);
    } else if (pin == GPIO_PIN2) {
        xSemaphore
