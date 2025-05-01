#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "Semaforos";

// Pines GPIO
#define GPIO_PIN_1 17
#define GPIO_PIN_2 18

// Semáforos para los pines
SemaphoreHandle_t sem_pin1;
SemaphoreHandle_t sem_pin2;

// Estados de los pines
bool pin1_ocupado = false;
bool pin2_ocupado = false;

// Función para parpadear un LED en un pin
void blink_led(int gpio, int period_ms, int duration_ms)
{
    int cycles = duration_ms / period_ms;
    for (int i = 0; i < cycles; i++) {
        gpio_set_level(gpio, 1);
        vTaskDelay((period_ms / 2) / portTICK_PERIOD_MS);
        gpio_set_level(gpio, 0);
        vTaskDelay((period_ms / 2) / portTICK_PERIOD_MS);
    }
}

// Lógica de acceso al recurso (pin)
int obtener_pin_disponible(int tarea_id, bool *esperando_reportado)
{
    if (xSemaphoreTake(sem_pin1, 0) == pdTRUE) {
        pin1_ocupado = true;
        *esperando_reportado = false;
        return GPIO_PIN_1;
    }
    if (xSemaphoreTake(sem_pin2, 0) == pdTRUE) {
        pin2_ocupado = true;
        *esperando_reportado = false;
        return GPIO_PIN_2;
    }

    if (!(*esperando_reportado)) {
        ESP_LOGI(TAG, "Tarea %d en espera: ambos GPIOs ocupados", tarea_id);
        *esperando_reportado = true;
    }
    return -1; // Ambos ocupados
}

// Liberar recurso (pin)
void liberar_pin(int pin, int tarea_id)
{
    if (pin == GPIO_PIN_1) {
        pin1_ocupado = false;
        xSemaphoreGive(sem_pin1);
        ESP_LOGI(TAG, "Tarea %d terminó. GPIO 17 libre", tarea_id);
    } else if (pin == GPIO_PIN_2) {
        pin2_ocupado = false;
        xSemaphoreGive(sem_pin2);
        ESP_LOGI(TAG, "Tarea %d terminó. GPIO 18 libre", tarea_id);
    }
}

// Plantilla para tareas
void tarea_func(void *params)
{
    int period = ((int*)params)[0];
    int duration = ((int*)params)[1];
    int id = ((int*)params)[2];

    bool esperando_reportado = false;

    while (1) {
        int pin = -1;

        // Esperar hasta obtener un pin disponible
        while (pin == -1) {
            pin = obtener_pin_disponible(id, &esperando_reportado);
            if (pin == -1) {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }

        ESP_LOGI(TAG, "Tarea %d ejecutándose en GPIO %d", id, pin);

        blink_led(pin, period, duration);

        liberar_pin(pin, id);

        vTaskDelay(500 / portTICK_PERIOD_MS); // Espera antes de volver a intentar
    }
}

void app_main(void)
{
    // Configurar los pines como salida
    gpio_set_direction(GPIO_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_PIN_2, GPIO_MODE_OUTPUT);

    // Crear semáforos binarios
    sem_pin1 = xSemaphoreCreateBinary();
    sem_pin2 = xSemaphoreCreateBinary();

    // Inicializar semáforos como disponibles
    xSemaphoreGive(sem_pin1);
    xSemaphoreGive(sem_pin2);

    // Parámetros de las tareas [periodo_ms, duracion_ms, id]
    static int params1[] = {3000, 5000, 1};   // T = 1s, durante 5s
    static int params2[] = {2000, 3000, 2};    // T = 0.5s, durante 3s
    static int params3[] = {5000, 10000, 3};  // T = 2s, durante 10s

    // Crear tareas
    xTaskCreatePinnedToCore(tarea_func, "Tarea1", 2048, params1, 2, NULL, 1);
    xTaskCreatePinnedToCore(tarea_func, "Tarea2", 2048, params2, 2, NULL, 1);
    xTaskCreatePinnedToCore(tarea_func, "Tarea3", 2048, params3, 2, NULL, 1);
}
