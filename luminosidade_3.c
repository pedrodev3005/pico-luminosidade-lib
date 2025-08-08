#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" // Necessário para as funções de I2C do Pico SDK
#include "opt4001.h"       // Inclui biblioteca OPT4001

// --- Definições de Hardware Específicas para PCB ---
// ATENÇÃO: Verifique se estas definições correspondem ao seu hardware.
#define I2C_APP_BAUDRATE 100 * 1000 // 100 kHz (modo padrão)
#define I2C_PORT_USED i2c0
#define SDA_PIN_USED 4
#define SCL_PIN_USED 5
#define SENSOR_I2C_ADDRESS 0x45
#define SENSOR_VARIANT OPT4001_VARIANT_SOT5X3 // PCB usa a variante SOT-5X3

// --- Função Principal do Programa ---
int main() {
    // Inicializa a interface serial para depuração via USB
    stdio_init_all();
    // Pequeno atraso para garantir que o terminal serial esteja pronto
    sleep_ms(2000);

    printf("Iniciando aplicacao de leitura continua do sensor OPT4001...\n");

    // 1. Inicializa o sensor OPT4001
    if (!opt4001_init(I2C_PORT_USED, SDA_PIN_USED, SCL_PIN_USED, SENSOR_I2C_ADDRESS, SENSOR_VARIANT, I2C_APP_BAUDRATE)) {
        printf("ERRO: Falha na inicializacao do OPT4001! Verifique as conexoes e definicoes.\n");
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

    // 3. Opcional: Configurar o LATCH para o modo transparente
    uint16_t current_config_reg;
    if (opt4001_read_register(OPT4001_REG_CONFIG, &current_config_reg)) { 
        current_config_reg &= ~(1 << 3); // Limpa o bit 3 (LATCH)
        opt4001_write_register(OPT4001_REG_CONFIG, current_config_reg); 
    }

    opt4001_data_t sensor_data;

    // --- Loop Principal de Leitura de Dados (Polling) ---
    while (true) {
        // Espera pela conversao ser concluida antes de ler os dados
        if (opt4001_wait_for_conversion_complete(500)) { 
            if (opt4001_get_data(&sensor_data)) { 
                // Imprime todos os dados lidos
                printf("Lux: %.2f | Exp: %d | Mant: %lu | ADC: %lu | Counter: %d\n",
                       sensor_data.lux, sensor_data.exponent, sensor_data.mantissa,
                       sensor_data.adc_codes, sensor_data.counter);

                printf("  Flags: Overload=%d, ConvReady=%d, FlagH=%d, FlagL=%d\n",
                       sensor_data.overload_flag, sensor_data.conversion_ready,
                       sensor_data.flag_h, sensor_data.flag_l);
            } else {
                printf("ERRO: Falha ao ler dados do sensor.\n");
            }
        } else {
            printf("AVISO: Tempo limite excedido na espera pela conversão.\n");
        }
        
        sleep_ms(1000); 
    }

    return 0;
}