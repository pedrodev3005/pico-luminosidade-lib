#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" // Necessário para as funções de I2C do Pico SDK
#include "opt4001.h"       // Inclui sua biblioteca OPT4001

// Definição do baud rate I2C para a aplicação
#define I2C_APP_BAUDRATE 100 * 1000 // 100 kHz (modo padrão)

int main() {
    // Inicializa a interface serial para depuração via USB
    stdio_init_all();
    // Pequeno atraso para garantir que o terminal serial esteja pronto
    sleep_ms(2000);

    printf("Iniciando teste do sensor OPT4001...\n");

    // 1. Inicializa o sensor OPT4001
    // A função opt4001_init agora só precisa do baudrate,
    // pois os outros parâmetros de hardware são fixos em opt4001.h.
    if (!opt4001_init(I2C_APP_BAUDRATE)) {
        printf("ERRO: Falha na inicialização do OPT4001! Verifique as conexões e definições em opt4001.h.\n");
        while (true) { // Loop infinito em caso de falha grave de inicialização
            sleep_ms(1000);
        }
    }
    printf("OPT4001 inicializado com sucesso no baudrate %u Hz!\n", I2C_APP_BAUDRATE);

    // 2. Configura o sensor para o modo de operação desejado
    // Exemplo: Modo Contínuo, Auto-Range, Tempo de Conversão de 400ms 
    if (!opt4001_set_operating_mode(OPT4001_MODE_CONTINUOUS)) { 
        printf("ERRO: Falha ao definir o modo de operação para CONTINUOUS.\n");
    }
    if (!opt4001_set_range(OPT4001_RANGE_AUTO)) { 
        printf("ERRO: Falha ao definir o modo RANGE para AUTO.\n");
    }
    if (!opt4001_set_conversion_time(OPT4001_CONV_TIME_400MS)) { 
        printf("ERRO: Falha ao definir o tempo de conversão para 400ms.\n");
    }
    printf("Sensor configurado: Modo Contínuo, Auto-Range, Conversão 400ms.\n");
    

// 3. Define os limiares de luz (thresholds) usando a nova função
    float desired_low_lux = 10.0f;  // Exemplo: Limiar baixo de 10 Lux
    float desired_high_lux = 1000.0f; // Exemplo: Limiar alto de 1000 Lux

    if (!opt4001_set_lux_thresholds(desired_low_lux, desired_high_lux)) {
        printf("ERRO: Falha ao configurar os limiares de interrupção em Lux.\n");
    } else {
        printf("Limiares de interrupção configurados: Baixo %.2f Lux, Alto %.2f Lux.\n",
               desired_low_lux, desired_high_lux);
    }

    opt4001_data_t sensor_data; // Estrutura para armazenar os dados lidos do sensor

    // --- Loop Principal de Leitura de Dados ---
    while (true) {
        // No modo contínuo, você pode esperar pela flag CONVERSION_READY para saber quando um novo resultado está disponível.
        // O tempo limite deve ser maior que o tempo de conversão configurado.
        if (opt4001_wait_for_conversion_complete(500)) { // Espera até 500ms (maior que 400ms de conversão)
            // Se a conversão foi concluída, tenta obter todos os dados do sensor.
            if (opt4001_get_data(&sensor_data)) { 
                // Imprime os dados lidos
                printf("Lux: %.2f | Exp: %d | Mant: %lu | ADC: %lu | Counter: %d\n",
                       sensor_data.lux,
                       sensor_data.exponent,
                       sensor_data.mantissa,
                       sensor_data.adc_codes,
                       sensor_data.counter);

                // Opcional: Imprime o estado das flags
                printf("  Flags: Overload=%d, ConvReady=%d, FlagH=%d, FlagL=%d\n",
                       sensor_data.overload_flag,
                       sensor_data.conversion_ready,
                       sensor_data.flag_h,
                       sensor_data.flag_l);
                       // Força a limpeza das flags escrevendo 0x0000 no registrador 0x0C
                //opt4001_write_register(OPT4001_REG_INTERRUPT_FLAGS, 0x0000);
                // Em main.c, após tentar desabilitar o LATCH:
                // ...
                //printf("Sensor configurado para flags transparentes (LATCH = 0).\n");
                uint16_t debug_config_reg;
                if (opt4001_read_register(OPT4001_REG_CONFIG, &debug_config_reg)) {
                    printf("DEBUG: REG_CONFIG (0x0A) lido: 0x%04X (LATCH bit 3: %d)\n",
                        debug_config_reg, (debug_config_reg >> 3) & 0x01);
                } else {
                    printf("DEBUG: Falha ao ler REG_CONFIG para verificar LATCH.\n");
                }
                // ..

            } else {
                printf("ERRO: Falha ao ler dados do sensor.\n");
            }
        } else {
            printf("AVISO: Tempo limite excedido na espera pela conversão ou erro de leitura da flag.\n");
            // Pode tentar um reset ou reconfiguração aqui se isso acontecer com frequência.
        }

        sleep_ms(1000); // Pequeno atraso entre as leituras para não sobrecarregar o terminal
    }

    return 0; // O programa nunca deve chegar aqui em um sistema embarcado típico
}