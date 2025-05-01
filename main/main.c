#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define GPIO_1 17
#define GPIO_2 18

static const char *TAG = "Semaforos";

SemaphoreHandle_t sem_gpio_1;
SemaphoreHandle_t sem_gpio_2;

int contador_t1 = 0;
int contador_t2 = 0;
int contador_t3 = 0;

void configurar_gpio(int gpio) {
    // gpio_pad_select_gpio(gpio); <- ELIMINADA por ser innecesaria en ESP-IDF 5+
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
}

void tarea_led(void *pvParameter) {
    int gpio, tarea_id;
    gpio = (int)pvParameter;

    if (gpio == GPIO_1)
        tarea_id = 1;
    else if (gpio == GPIO_2)
        tarea_id = 2;
    else
        tarea_id = 3;

    TickType_t periodo = 1000;   // Valores por defecto
    TickType_t duracion = 5000;

    switch (tarea_id) {
        case 1: periodo = 1000; duracion = 5000; break;
        case 2: periodo = 500;  duracion = 3000; break;
        case 3: periodo = 2000; duracion = 10000; break;
        default:
            ESP_LOGE(TAG, "Tarea inválida: %d", tarea_id);
            vTaskDelete(NULL);
    }

    while (1) {
        ESP_LOGI(TAG, "Tarea %d: Intentando ejecutar", tarea_id);

        BaseType_t tomado_1 = xSemaphoreTake(sem_gpio_1, 0);
        BaseType_t tomado_2 = xSemaphoreTake(sem_gpio_2, 0);

        if (tomado_1 == pdTRUE) {
            ESP_LOGI(TAG, "Tarea %d ejecutándose en GPIO %d", tarea_id, GPIO_1);
            gpio_set_level(GPIO_1, 1);
            vTaskDelay(duracion / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_1, 0);
            ESP_LOGI(TAG, "Tarea %d: Terminó ejecución, liberando GPIO %d", tarea_id, GPIO_1);
            xSemaphoreGive(sem_gpio_1);
            ESP_LOGI(TAG, "Semaforos: GPIO %d libre", GPIO_1);

            if (tarea_id == 1) contador_t1++;
            if (tarea_id == 2) contador_t2++;
            if (tarea_id == 3) contador_t3++;

        } else if (tomado_2 == pdTRUE) {
            ESP_LOGI(TAG, "Tarea %d ejecutándose en GPIO %d", tarea_id, GPIO_2);
            gpio_set_level(GPIO_2, 1);
            vTaskDelay(duracion / portTICK_PERIOD_MS);
            gpio_set_level(GPIO_2, 0);
            ESP_LOGI(TAG, "Tarea %d: Terminó ejecución, liberando GPIO %d", tarea_id, GPIO_2);
            xSemaphoreGive(sem_gpio_2);
            ESP_LOGI(TAG, "Semaforos: GPIO %d libre", GPIO_2);

            if (tarea_id == 1) contador_t1++;
            if (tarea_id == 2) contador_t2++;
            if (tarea_id == 3) contador_t3++;

        } else {
            ESP_LOGI(TAG, "Tarea %d: Ambos pines ocupados. Esperando...", tarea_id);
        }

        vTaskDelay(periodo / portTICK_PERIOD_MS);
    }
}

void monitor_contadores(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Ejecuciones -> T1: %d | T2: %d | T3: %d", contador_t1, contador_t2, contador_t3);
        vTaskDelay(pdMS_TO_TICKS(10000)); // cada 10 segundos
    }
}

void app_main(void) {
    configurar_gpio(GPIO_1);
    configurar_gpio(GPIO_2);

    sem_gpio_1 = xSemaphoreCreateBinary();
    sem_gpio_2 = xSemaphoreCreateBinary();

    xSemaphoreGive(sem_gpio_1); // disponibles al inicio
    xSemaphoreGive(sem_gpio_2);

    xTaskCreate(tarea_led, "Tarea1", 2048, (void *)GPIO_1, 1, NULL);
    xTaskCreate(tarea_led, "Tarea2", 2048, (void *)GPIO_2, 1, NULL);
    xTaskCreate(tarea_led, "Tarea3", 2048, (void *)0,       1, NULL); // usa cualquiera

    xTaskCreate(monitor_contadores, "Monitor", 2048, NULL, 1, NULL);
}
