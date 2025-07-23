#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" // Necessário para as funções de I2C do Pico SDK
#include "opt4001.h"       // Inclui sua biblioteca OPT4001
#include "hardware/gpio.h"
#include "hardware/sync.h"

// Definição do baud rate I2C para a aplicação
#define I2C_APP_BAUDRATE 100 * 1000 // 100 kHz (modo padrão)

#define OPT4001_INT_GPIO_PIN 13

// Variável global para sinalizar a interrupção (volátil para garantir visibilidade)
volatile bool opt4001_interrupted = false;

// Rotina de Tratamento de Interrupção (ISR)
void opt4001_int_gpio_callback(uint gpio, uint32_t events) {
    if (gpio == OPT4001_INT_GPIO_PIN) {
        opt4001_interrupted = true;
        // Na ISR, minimize o trabalho. Apenas sinalize e lide com o resto no loop principal.
        // A leitura do registrador 0x0C para limpar a interrupção pode ser feita aqui,
        // mas é mais seguro fazê-la no loop principal se for uma operação I2C complexa.
        // O datasheet diz que a leitura do 0x0C limpa a flag, mas isso deve ser feito
        // quando a condição que causou a interrupção não está mais presente (para flags latchadas)
        // ou a cada leitura (para flags transparentes).

        // Se o pino INT do sensor for Active-Low e você o conectou a um pull-up,
        // a interrupção pode ser configurada para GPIO_IRQ_EDGE_FALL (borda de descida).
        // Se for Active-High, então GPIO_IRQ_EDGE_RISE (borda de subida).
        // O padrão do OPT4001 é Active-Low.
    }
}


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

    // 5. Configura o pino INT do sensor (sensor-side)
    // Polarity: OPT4001_INT_POLARITY_ACTIVE_LOW (padrão do sensor)
    // Fault Count: OPT4001_FAULT_COUNT_2_FAULTS (para 2 falhas consecutivas) 
    if (!opt4001_config_interrupt_pin(OPT4001_INT_POLARITY_ACTIVE_LOW, OPT4001_FAULT_COUNT_2_FAULTS)) {
        printf("ERRO: Falha ao configurar o pino INT do sensor.\n");
    } else {
        printf("Pino INT do sensor configurado: Ativo Baixo, 2 Falhas para Interrupção.\n");
    }

    // 6. Configura o pino GPIO na Raspberry Pi Pico W para a interrupção
    gpio_set_dir(OPT4001_INT_GPIO_PIN, GPIO_IN); // Configura o pino como entrada
    gpio_pull_up(OPT4001_INT_GPIO_PIN); // Habilita pull-up, pois o pino INT do sensor é open-drain 

    // Configura a interrupção GPIO na borda de descida (se Active-Low)
    gpio_set_irq_enabled_with_callback(OPT4001_INT_GPIO_PIN, GPIO_IRQ_EDGE_FALL, true, &opt4001_int_gpio_callback);
    printf("Interrupção GPIO da Pico W configurada para o pino %d.\n", OPT4001_INT_GPIO_PIN);

    opt4001_data_t sensor_data;

        // --- Loop Principal ---
    while (true) {
        // O microcontrolador pode dormir para economizar energia
        // (WFI = Wait For Interrupt)
        __wfi(); // Espera por uma interrupção

        // A interrupção ocorreu!
        if (opt4001_interrupted) {
            opt4001_interrupted = false; // Limpa a flag de interrupção

            printf("\n--- INTERRUPÇÃO DETECTADA ---\n");

            // Imediatamente após a interrupção, leia os dados e as flags para saber a causa
            if (opt4001_get_data(&sensor_data)) {
                printf("Lux: %.2f | Exp: %d | Mant: %lu | ADC: %lu | Counter: %d\n",
                       sensor_data.lux, sensor_data.exponent, sensor_data.mantissa,
                       sensor_data.adc_codes, sensor_data.counter);

                printf("  Flags: Overload=%d, ConvReady=%d, FlagH=%d, FlagL=%d\n",
                       sensor_data.overload_flag, sensor_data.conversion_ready,
                       sensor_data.flag_h, sensor_data.flag_l);

                // No modo Transparent Hysteresis, a flag será 1 se a condição de alarme
                // ainda for verdadeira. Se a medição estiver na zona de histerese,
                // a flag permanecerá como estava quando foi definida.

                // Você pode adicionar lógica aqui para responder à interrupção:
                if (sensor_data.flag_h) {
                    printf("  ALERTA: Luz acima do limite alto!\n");
                    // Implemente sua ação para luz alta aqui
                }
                if (sensor_data.flag_l) {
                    printf("  ALERTA: Luz abaixo do limite baixo!\n");
                    // Implemente sua ação para luz baixa aqui
                }
                if (sensor_data.overload_flag) {
                    printf("  AVISO: Sensor sobrecarregado!\n");
                    // Implemente sua ação para sobrecarga aqui
                }

                // Não é necessário um opt4001_write_register(0x0C, 0x0000) explícito aqui
                // para limpar os flags H/L no modo transparente, a menos que você queira
                // forçar um reset (se for o caso de não se mover sozinho na histerese).
                // A histerese fará com que o estado seja mantido ou atualizado na próxima
                // medição, dependendo da nova luz.

            } else {
                printf("ERRO: Falha ao ler dados do sensor após interrupção.\n");
            }

            printf("--------------------------\n");
        }
        // O sleep_ms(1000) não é necessário aqui, pois o __wfi() já gerencia o tempo
        // dormindo até a próxima interrupção. Se não houver interrupções, o sistema
        // ficará em __wfi() indefinidamente.
        // Se você quiser leituras periódicas mesmo sem interrupção (ex: a cada 5s),
        // use um timer em vez de sleep_ms e configure a interrupção do timer.
    }

    return 0; // O programa nunca deve chegar aqui em um sistema embarcado típico
}