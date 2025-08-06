#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" // Necessário para as funções de I2C do Pico SDK
#include "hardware/gpio.h" // Necessário para interrupções GPIO
#include "hardware/sync.h" // Para __wfi()
#include "opt4001.h"       // Inclui biblioteca OPT4001

// --- Definições de Hardware Específicas para PCB ---
// ATENÇÃO: Verifique se estas definições correspondem ao seu hardware.
#define I2C_APP_BAUDRATE 100 * 1000 // 100 kHz (modo padrão)
#define I2C_PORT_USED i2c0
#define SDA_PIN_USED 4
#define SCL_PIN_USED 5
#define SENSOR_I2C_ADDRESS 0x45
#define SENSOR_VARIANT OPT4001_VARIANT_SOT5X3 // PCB usa a variante SOT-5X3
#define OPT4001_INT_GPIO_PIN 13 // Pino da Pico W conectado ao pino INT do sensor

// --- Variáveis para Gerenciamento de Interrupção ---
// Variável global para sinalizar a interrupção (volátil para garantir visibilidade)
volatile bool opt4001_interrupted = false;

// --- Rotina de Tratamento de Interrupção (ISR) ---
// Esta função é chamada automaticamente pelo hardware quando o pino INT é ativado.
void opt4001_int_gpio_callback(uint gpio, uint32_t events) {
    if (gpio == OPT4001_INT_GPIO_PIN) {
        opt4001_interrupted = true;
        // Na ISR, minimize o trabalho. Apenas sinalize a flag.
    }
}

// --- Função Principal do Programa ---
int main() {
    // Inicializa a interface serial para depuração via USB
    stdio_init_all();
    // Pequeno atraso para garantir que o terminal serial esteja pronto
    sleep_ms(2000);

    printf("Iniciando aplicacao com sensor OPT4001 e interrupcoes...\n");

    // 1. Inicializa o sensor OPT4001
    // A funcao opt4001_init agora aceita todos os parametros de hardware
    if (!opt4001_init(I2C_PORT_USED, SDA_PIN_USED, SCL_PIN_USED, SENSOR_I2C_ADDRESS, SENSOR_VARIANT, I2C_APP_BAUDRATE)) {
        printf("ERRO: Falha na inicialização do OPT4001! Verifique as conexoes e definições.\n");
        while (true) { // Loop infinito em caso de falha grave
            sleep_ms(1000);
        }
    }
    printf("OPT4001 inicializado com sucesso no baudrate %u Hz!\n", I2C_APP_BAUDRATE);

    // 2. Configura o sensor para o modo de operação desejado
    if (!opt4001_set_operating_mode(OPT4001_MODE_CONTINUOUS)) {
        printf("ERRO: Falha ao definir o modo de operacao para CONTINUOUS.\n");
    }
    if (!opt4001_set_range(OPT4001_RANGE_AUTO)) {
        printf("ERRO: Falha ao definir o modo RANGE para AUTO.\n");
    }
    if (!opt4001_set_conversion_time(OPT4001_CONV_TIME_400MS)) {
        printf("ERRO: Falha ao definir o tempo de conversao para 400ms.\n");
    }
    printf("Sensor configurado: Modo Continuo, Auto-Range, Conversao 400ms.\n");
    
    // 3. Configura o LATCH para o modo TRANSPARENT HYSTERESIS (LATCH = 0)
    // Isso é essencial para o comportamento de histerese transparente que desejamos.
    uint16_t current_config_reg;
    if (opt4001_read_register(OPT4001_REG_CONFIG, &current_config_reg)) {
        current_config_reg &= ~(1 << 3); // Limpa o bit 3 (LATCH)
        if (!opt4001_write_register(OPT4001_REG_CONFIG, current_config_reg)) {
            printf("ERRO: Falha ao configurar o bit LATCH (Transparent Hysteresis Mode).\n");
        }
    } else {
        printf("ERRO: Falha ao ler reg config para configurar LATCH.\n");
    }
    printf("Sensor configurado para Transparent Hysteresis Mode (LATCH = 0).\n");

    // 4. Define os limiares de luz (thresholds) usando a função de Lux
    float desired_low_lux = 50.0f;  // Luz baixa para alarme
    float desired_high_lux = 500.0f; // Luz alta para alarme

    if (!opt4001_set_lux_thresholds(desired_low_lux, desired_high_lux)) {
        printf("ERRO: Falha ao configurar os limiares de interrupcao em Lux.\n");
    } else {
        printf("Limiares de interrupcao configurados: Baixo %.2f Lux, Alto %.2f Lux.\n",
               desired_low_lux, desired_high_lux);
    }


    // --- Lógica de Correção: Checagem Inicial do Estado das Flags ---
    opt4001_data_t initial_data;
    if (opt4001_get_data(&initial_data)) {
        if (initial_data.flag_l || initial_data.flag_h) {
            printf("\nAVISO: O sensor foi inicializado em um estado de alarme!\n");
            printf("Alarme inicial: Lux %.2f, FlagH=%d, FlagL=%d\n", initial_data.lux, initial_data.flag_h, initial_data.flag_l);
        }
    }

    // 5. Configura o pino INT do sensor (sensor-side)
    if (!opt4001_config_interrupt_pin(OPT4001_INT_POLARITY_ACTIVE_LOW,OPT4001_FAULT_COUNT_1_FAULT)) {
        printf("ERRO: Falha ao configurar o pino INT do sensor.\n");
    } else {
        printf("Pino INT do sensor configurado: Ativo Baixo, 1 Falhas para Interrupção.\n");
    }

    // 6. Configura o pino GPIO na Raspberry Pi Pico W para a interrupção
    gpio_set_dir(OPT4001_INT_GPIO_PIN, GPIO_IN); // Pino como entrada
    gpio_pull_up(OPT4001_INT_GPIO_PIN); // Habilita pull-up, pois o pino INT do sensor é open-drain

    uint32_t irq_mask = GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE;
    gpio_set_irq_enabled_with_callback(OPT4001_INT_GPIO_PIN, irq_mask, true, &opt4001_int_gpio_callback);
    printf("Interrupcao GPIO da Pico W configurada para o pino %d para ambas as bordas.\n", OPT4001_INT_GPIO_PIN);

    opt4001_data_t sensor_data;

    // --- Loop Principal ---
    while (true) {
        // O microcontrolador entra em um estado de baixa energia
        __wfi(); // Espera por uma interrupção

        // O microcontrolador foi acordado por uma interrupção!
        if (opt4001_interrupted) {
            // Limpa a flag de interrupção
            opt4001_interrupted = false;

            printf("\n--- INTERRUPÇÃO DETECTADA ---\n");

            // Imediatamente após a interrupção, leia os dados e as flags para saber a causa
            if (opt4001_get_data(&sensor_data)) {
                printf("Lux: %.2f | Exp: %d | Mant: %lu | ADC: %lu | Counter: %d\n",
                       sensor_data.lux, sensor_data.exponent, sensor_data.mantissa,
                       sensor_data.adc_codes, sensor_data.counter);

                printf("  Flags: Overload=%d, ConvReady=%d, FlagH=%d, FlagL=%d\n",
                       sensor_data.overload_flag, sensor_data.conversion_ready,
                       sensor_data.flag_h, sensor_data.flag_l);

                // Adicione sua lógica de resposta aqui:
                if (sensor_data.flag_h) {
                    printf("  ALERTA: Luz acima do limite alto!\n");
                }
                if (sensor_data.flag_l) {
                    printf("  ALERTA: Luz abaixo do limite baixo!\n");
                }
                if (sensor_data.overload_flag) {
                    printf("  AVISO: Sensor sobrecarregado!\n");
                }

            } else {
                printf("ERRO: Falha ao ler dados do sensor após interrupção.\n");
            }
            printf("--------------------------\n");
            printf("Programa retornando ao sono. Mude a luz para disparar outra interrupcao.\n");
        }
    }

    return 0;
}