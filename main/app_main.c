#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // Para esp_rom_delay_us
#include "esp_timer.h"     // Para esp_timer_get_time

#define DHT22_PIN GPIO_NUM_4   // Pino de dados do DHT22
#define LED_PIN GPIO_NUM_2     // Pino do LED 1 (para temperatura)
#define FAN_PIN GPIO_NUM_14     // Pino da ventoinha
#define LED_TIME_PIN GPIO_NUM_18 // Pino do LED 2 (para o tempo)
#define TRIG_PIN GPIO_NUM_15   // Pino TRIG do sensor ultrassônico
#define ECHO_PIN GPIO_NUM_19   // Pino ECHO do sensor ultrassônico

#define RESERVOIR_HEIGHT 12.5  // Altura do reservatório em cm
#define MAX_VOLUME 1.8         // Capacidade máxima do reservatório em litros
#define TEMP_THRESHOLD 27.0   // Temperatura limite para ligar a ventoinha

static const char *TAG = "DHT22";

// Função para atrasar o tempo em microssegundos
void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

// Função para ler a distância do sensor ultrassônico
float read_ultrasonic_distance() {
    // Garante que o TRIG esteja em nível baixo
    gpio_set_level(TRIG_PIN, 0);
    delay_us(2);
    
    // Envia um pulso de 10 us no TRIG
    gpio_set_level(TRIG_PIN, 1);
    delay_us(10);
    gpio_set_level(TRIG_PIN, 0);
    
    // Espera o início do pulso no ECHO com timeout
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0 && (esp_timer_get_time() - start_time) < 20000);  // Timeout de 20ms
    
    // Se timeout, retorna uma distância inválida
    if (gpio_get_level(ECHO_PIN) == 0) {
        ESP_LOGE(TAG, "Falha na leitura do ECHO (timeout)");
        return -1;
    }
    
    // Mede o tempo que o ECHO fica em nível alto
    start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1 && (esp_timer_get_time() - start_time) < 20000);  // Timeout de 20ms
    
    if (gpio_get_level(ECHO_PIN) == 1) {
        ESP_LOGE(TAG, "Falha no fim do pulso do ECHO (timeout)");
        return -1;
    }
    
    // Calcula a duração do pulso (em microssegundos)
    int64_t end_time = esp_timer_get_time();
    float duration = (float)(end_time - start_time);
    
    // Calcula a distância com base no tempo do pulso
    float distance_cm = (duration / 2.0) * 0.0343;  // Velocidade do som: 343 m/s
    return distance_cm;
}

// Função para calcular o volume de água no reservatório
float calculate_water_volume(float distance) {
    // Altura da água no reservatório
    float water_height = RESERVOIR_HEIGHT - distance;  
    if (water_height < 0) water_height = 0; // Não pode ser negativa
    float volume = (water_height / RESERVOIR_HEIGHT) * MAX_VOLUME; // Volume em litros
    return volume;
}

// Função simulada para ler dados do DHT22
bool read_dht22(float *temperature, float *humidity) {
    // Aqui você adicionaria o código real para ler do DHT22.
    // Simulação para teste:
    *temperature = 25.0 + (rand() % 10);  // Gera temperatura entre 25 e 35
    *humidity = 50.0 + (rand() % 20);     // Gera umidade entre 50 e 70
    return true;
}

// Função para controle da ventoinha com base na temperatura
void control_fan(float temperature) {
    if (temperature > TEMP_THRESHOLD) {
        gpio_set_level(FAN_PIN, 1);  // Liga a ventoinha
        ESP_LOGI(TAG, "Ventoinha LIGADA - Temperatura: %.2f", temperature);
    } else {
        gpio_set_level(FAN_PIN, 0);  // Desliga a ventoinha
        ESP_LOGI(TAG, "Ventoinha DESLIGADA - Temperatura: %.2f", temperature);
    }
}

void app_main(void) {
    float temperature, humidity;
    float distance, volume;

    // Configurar os LEDs, a ventoinha e o sensor ultrassônico como saída/entrada
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(FAN_PIN);
    gpio_set_direction(FAN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_TIME_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    
    printf("Hello, Wokwi!\n");

    while (1) {
        // Ler a distância do sensor ultrassônico
        distance = read_ultrasonic_distance();
        if (distance < 0) {
            ESP_LOGE(TAG, "Erro na leitura do sensor ultrassônico");
        } else {
            printf("Distância medida: %.2f cm\n", distance);
            ESP_LOGI(TAG, "Distância medida: %.2f cm", distance);

            // Calcular o volume de água no reservatório
            volume = calculate_water_volume(distance);
            ESP_LOGI(TAG, "Volume de água: %.2f L", volume);
        }

        // Ler os dados do DHT22
        if (read_dht22(&temperature, &humidity)) {
            ESP_LOGI(TAG, "Temperatura: %.2f C, Umidade: %.2f%%", temperature, humidity);
            
            // Controlar a ventoinha com base na temperatura
            control_fan(temperature);

            // Indicar a leitura de temperatura no LED
            gpio_set_level(LED_PIN, 1);  // Acende o LED
            vTaskDelay(100 / portTICK_PERIOD_MS);  // LED aceso por 100 ms
            gpio_set_level(LED_PIN, 0);  // Apaga o LED
        } else {
            ESP_LOGE(TAG, "Erro na leitura do DHT22");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Aguarda 2 segundos para a próxima leitura
    }
}
