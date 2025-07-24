// test_opt4001.c
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"    // Para interrupções GPIO
#include "hardware/sync.h"    // Para __wfi()
#include "opt4001.h"          

// Definição do baud rate I2C para o teste
#define I2C_TEST_BAUDRATE 100 * 1000 // 100 kHz (modo padrão)

// Pino GPIO da Raspberry Pi Pico W conectado ao pino INT do OPT4001
// Ajuste este pino para o que você usa para o pino INT do sensor.
#define OPT4001_TEST_INT_GPIO_PIN 13 // Verifique sua conexão física na PCB (pode ser 12 ou outro)

// Variável global para sinalizar a interrupção (volátil para garantir visibilidade)
volatile bool opt4001_test_interrupted = false;

// Rotina de Tratamento de Interrupção (ISR) para o teste
void opt4001_test_int_gpio_callback(uint gpio, uint32_t events) {
    if (gpio == OPT4001_TEST_INT_GPIO_PIN) {
        opt4001_test_interrupted = true;
        // Na ISR, minimize o trabalho. Apenas sinalize.
    }
}

// --- Função auxiliar para rodar um teste e imprimir o resultado ---
static void run_test(const char* test_name, bool (*test_func)(void)) {
    printf("\n--- Teste: %s ---\n", test_name);
    if (test_func()) {
        printf("RESULTADO: SUCESSO\n");
    } else {
        printf("RESULTADO: FALHA\n");
    }
    printf("---------------------------\n");
    sleep_ms(100); // Pequeno delay entre testes
}

// --- Funções de Teste Individuais ---

bool test_1_init_and_device_id() {
    printf("1. Testando opt4001_init()...\n");
    if (!opt4001_init(I2C_TEST_BAUDRATE)) {
        printf("   Falha na inicializacao I2C ou Device ID nao lido.\n");
        return false;
    }
    printf("   opt4001_init() executado com sucesso. Verifique a saida do Device ID (deve ser 0x0121).\n");
    return true;
}

bool test_2_read_write_basic_register_access() {
    printf("2. Testando opt4001_write_register() e opt4001_read_register()...\n");
    uint16_t test_value = 0xABCD;
    uint16_t read_value = 0;

    // Tentar escrever e ler de um registrador nao-configuravel que nao interfira
    // O registrador 0x0C (INTERRUPT_FLAGS) e de leitura, mas uma escrita nele limpa flags.
    // Vamos usar um registrador de limiar (0x08 ou 0x09) que é R/W.
    // Primeiro, leia o valor atual para restaurar depois
    uint16_t original_threshold_l_exp;
    if (!opt4001_read_register(OPT4001_REG_THRESHOLD_L_EXPONENT, &original_threshold_l_exp)) {
        printf("   Falha ao ler REG_THRESHOLD_L_EXPONENT original.\n");
        return false;
    }

    printf("   Escrevendo 0x%04X no REG_THRESHOLD_L_EXPONENT (0x%02X)...\n", test_value, OPT4001_REG_THRESHOLD_L_EXPONENT);
    if (!opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, test_value)) {
        printf("   Falha na escrita.\n");
        return false;
    }
    sleep_ms(10); // Pequeno delay

    printf("   Lendo do REG_THRESHOLD_L_EXPONENT (0x%02X)...\n", OPT4001_REG_THRESHOLD_L_EXPONENT);
    if (!opt4001_read_register(OPT4001_REG_THRESHOLD_L_EXPONENT, &read_value)) {
        printf("   Falha na leitura.\n");
        // Tentar restaurar mesmo que a leitura falhe
        opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, original_threshold_l_exp);
        return false;
    }

    printf("   Valor lido: 0x%04X\n", read_value);

    // Restaurar o valor original
    if (!opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, original_threshold_l_exp)) {
        printf("   AVISO: Falha ao restaurar REG_THRESHOLD_L_EXPONENT original.\n");
    }

    if (read_value == test_value) {
        return true;
    } else {
        printf("   Erro: Valor escrito (0x%04X) nao corresponde ao lido (0x%04X).\n", test_value, read_value);
        return false;
    }
}


bool test_3_operating_mode_and_range() {
    printf("3. Testando opt4001_set_operating_mode() e opt4001_set_range()...\n");
    uint16_t config_reg_value;

    // 3.1 Testar Power-down
    if (!opt4001_set_operating_mode(OPT4001_MODE_POWER_DOWN)) return false;
    sleep_ms(10);
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &config_reg_value)) return false;
    if (((config_reg_value >> 4) & 0x03) != OPT4001_MODE_POWER_DOWN) {
        printf("   Falha: Modo Power-down nao configurado (lido: 0x%04X).\n", config_reg_value);
        return false;
    }
    printf("   Modo Power-down configurado e verificado.\n");

    // 3.2 Testar Continuous e Auto-Range
    if (!opt4001_set_operating_mode(OPT4001_MODE_CONTINUOUS)) return false;
    if (!opt4001_set_range(OPT4001_RANGE_AUTO)) return false;
    sleep_ms(10);
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &config_reg_value)) return false;
    if (((config_reg_value >> 4) & 0x03) != OPT4001_MODE_CONTINUOUS ||
        ((config_reg_value >> 10) & 0x0F) != OPT4001_RANGE_AUTO) {
        printf("   Falha: Modo Continuo ou Auto-Range nao configurado (lido: 0x%04X).\n", config_reg_value);
        return false;
    }
    printf("   Modo Continuo e Auto-Range configurados e verificados.\n");
    return true;
}

bool test_4_conversion_time() {
    printf("4. Testando opt4001_set_conversion_time()...\n");
    uint16_t config_reg_value;
    opt4001_conversion_time_t test_conv_time = OPT4001_CONV_TIME_100MS;

    if (!opt4001_set_conversion_time(test_conv_time)) return false;
    sleep_ms(10);
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &config_reg_value)) return false;

    if (((config_reg_value >> 6) & 0x0F) != test_conv_time) {
        printf("   Falha: Tempo de conversao nao configurado (lido: 0x%04X, esperado: 0x%X).\n", config_reg_value, test_conv_time);
        return false;
    }
    printf("   Tempo de conversao para 100ms configurado e verificado.\n");
    return true;
}

bool test_5_get_data_and_lux_calculation() {
    printf("5. Testando opt4001_get_data() e calculo de Lux...\n");
    opt4001_data_t test_data;
    
    // Assegura que o sensor esta ativo para gerar dados
    opt4001_set_operating_mode(OPT4001_MODE_CONTINUOUS);
    opt4001_set_range(OPT4001_RANGE_AUTO);
    opt4001_set_conversion_time(OPT4001_CONV_TIME_400MS); // Tempo suficiente para uma conversao

    sleep_ms(500); // Esperar pelo menos um ciclo de conversao

    if (!opt4001_get_data(&test_data)) {
        printf("   Falha ao obter dados do sensor.\n");
        return false;
    }
    printf("   Leitura de dados bem-sucedida:\n");
    printf("    Lux: %.2f\n", test_data.lux);
    printf("    Exp: %d, Mant: %lu, ADC: %lu\n", test_data.exponent, test_data.mantissa, test_data.adc_codes);
    printf("    Flags: Overload=%d, ConvReady=%d, FlagH=%d, FlagL=%d\n",
           test_data.overload_flag, test_data.conversion_ready, test_data.flag_h, test_data.flag_l);
    printf("    Counter: %d, CRC: %d\n", test_data.counter, test_data.crc);

    // Validação básica do Lux (assumindo alguma luz no ambiente)
    if (test_data.lux <= 0 && !test_data.overload_flag) { // Se nao esta sobrecarregado, Lux > 0
        printf("   AVISO: Lux lido e zero ou negativo em ambiente supostamente iluminado.\n");
        // Nao e uma falha critica da funcao, mas um aviso sobre o ambiente/sensor.
        // Depende de quao rigoroso voce quer ser.
    }
    return true;
}

bool test_6_wait_for_conversion() {
    printf("6. Testando opt4001_wait_for_conversion_complete()...\n");
    // Coloca em modo One-Shot para testar a espera
    if (!opt4001_set_operating_mode(OPT4001_MODE_ONE_SHOT)) return false;
    if (!opt4001_set_conversion_time(OPT4001_CONV_TIME_100MS)) return false;
    sleep_ms(100); // Dar tempo para o sensor entrar em one-shot/power-down

    printf("   Acionando uma conversao One-Shot e esperando (max 200ms)...\n");
    // Aciona a conversao (escrevendo 0b01 no OPERATING_MODE para Forced Auto-Range One-Shot)
    // Para one-shot, o bit OPERATING_MODE reseta para 0 apos a conversao 
    uint16_t config_reg;
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &config_reg)) return false;
    config_reg = (config_reg & ~((0x3) << 4)) | (OPT4001_MODE_FORCE_AUTO_ONE_SHOT << 4);
    if (!opt4001_write_register(OPT4001_REG_CONFIG, config_reg)) return false;

    if (!opt4001_wait_for_conversion_complete(200)) { // Tempo limite ligeiramente maior que o de conversao
        printf("   Falha: Conversao nao concluida a tempo.\n");
        return false;
    }
    printf("   Sucesso: Conversao concluida a tempo.\n");
    return true;
}

bool test_7_set_lux_thresholds_and_get_flags() {
    printf("7. Testando opt4001_set_lux_thresholds() e opt4001_get_flags()...\n");
    bool ov, cr, fh, fl;

    // Assegura modo transparente para ver as flags mudarem
    uint16_t current_config_reg;
    if (opt4001_read_register(OPT4001_REG_CONFIG, &current_config_reg)) {
        current_config_reg &= ~(1 << 3); // Limpa o bit 3 (LATCH) para Transparent Hysteresis
        if (!opt4001_write_register(OPT4001_REG_CONFIG, current_config_reg)) {
            printf("   Falha ao configurar LATCH para transparente.\n");
            return false;
        }
    } else {
        printf("   Falha ao ler REG_CONFIG para configurar LATCH.\n");
        return false;
    }
    printf("   LATCH configurado para modo transparente (0).\n");

    // Define limiares de teste: Baixo (50 Lux), Alto (500 Lux)
    float low_test_lux = 50.0f;
    float high_test_lux = 500.0f;
    if (!opt4001_set_lux_thresholds(low_test_lux, high_test_lux)) {
        printf("   Falha ao definir limiares de teste.\n");
        return false;
    }
    printf("   Limiares de teste configurados: Baixo %.2f Lux, Alto %.2f Lux.\n", low_test_lux, high_test_lux);

    printf("   OBSERVE: Altere a luz e observe as flags. Comece com luz entre %.2f e %.2f Lux.\n", low_test_lux, high_test_lux);
    printf("   Luz abaixo de %.2f Lux deve ativar FlagL.\n", low_test_lux);
    printf("   Luz acima de %.2f Lux deve ativar FlagH.\n", high_test_lux);
    printf("   Verificando flags por 5s (10 leituras):\n");

    opt4001_data_t test_data; // Para garantir que o sensor esta lendo e as flags estao sendo geradas
    for (int i = 0; i < 10; ++i) {
        if (!opt4001_get_data(&test_data)) { // Garante que novas medicoes estao ocorrendo
            printf("   Falha ao obter dados na iteracao %d. Pulos nas medicoes.\n", i + 1);
        }

        if (opt4001_get_flags(&ov, &cr, &fh, &fl)) {
            printf("   Flags (iter %d, Lux: %.2f): Overload=%d, ConvReady=%d, FlagH=%d, FlagL=%d\n", i + 1, test_data.lux, ov, cr, fh, fl);
        } else {
            printf("   Falha ao obter flags na iteracao %d.\n", i + 1);
            return false;
        }
        sleep_ms(500); // Espera para a proxima conversao
    }
    printf("   Teste de flags concluido. Verificacao visual NECESSARIA (mude a luz manualmente).\n");
    return true; // Sucesso na execucao, validacao visual
}


bool test_8_interrupt_handling() {
    printf("8. Testando tratamento de interrupcao com __wfi()...\n");
    opt4001_test_interrupted = false; // Reset before test

    // Assegura modo transparente para que as flags mudem de estado
    uint16_t config_reg_val;
    if (opt4001_read_register(OPT4001_REG_CONFIG, &config_reg_val)) {
        config_reg_val &= ~(1 << 3); // LATCH = 0
        if (!opt4001_write_register(OPT4001_REG_CONFIG, config_reg_val)) {
            printf("   Falha ao configurar LATCH para transparente.\n"); return false;
        }
    } else { printf("   Falha ao ler REG_CONFIG.\n"); return false; }
    printf("   LATCH configurado para modo transparente (0).\n");


    // Define limiares de teste: Baixo (50 Lux), Alto (500 Lux)
    float low_lux_int = 50.0f;
    float high_lux_int = 500.0f;
    if (!opt4001_set_lux_thresholds(low_lux_int, high_lux_int)) {
        printf("   Falha ao definir limiares para interrupcao.\n"); return false;
    }
    printf("   Limiares para interrupcao configurados: Baixo %.2f Lux, Alto %.2f Lux.\n", low_lux_int, high_lux_int);

    // Configura o pino INT do sensor: Ativo Baixo, 2 Falhas para Interrupção
    if (!opt4001_config_interrupt_pin(OPT4001_INT_POLARITY_ACTIVE_LOW, OPT4001_FAULT_COUNT_2_FAULTS)) {
        printf("   Falha ao configurar o pino INT do sensor.\n"); return false;
    }
    printf("   Pino INT do sensor configurado: Ativo Baixo, 2 Falhas para Interrupcao.\n");

    // Configura o pino GPIO da Pico W para a interrupção
    gpio_set_dir(OPT4001_TEST_INT_GPIO_PIN, GPIO_IN);
    gpio_pull_up(OPT4001_TEST_INT_GPIO_PIN);
    gpio_set_irq_enabled_with_callback(OPT4001_TEST_INT_GPIO_PIN, GPIO_IRQ_EDGE_FALL, true, &opt4001_test_int_gpio_callback);
    printf("   Interrupcao GPIO da Pico W configurada para o pino %d.\n", OPT4001_TEST_INT_GPIO_PIN);

    printf("   OBSERVE: Entre em loop __wfi(). Force a luz ABAIXO de %.2f Lux ou ACIMA de %.2f Lux para disparar a interrupcao.\n", low_lux_int, high_lux_int);
    printf("   O Pico ira dormir e acordar somente com interrupcao do sensor.\n");

    opt4001_data_t test_data;
    for(int i = 0; i < 3; ++i) { // Testa algumas interrupcoes
        printf("   Dormindo... (ciclo %d)\n", i+1);
        opt4001_test_interrupted = false; // Reset antes de dormir
        __wfi(); // Espera por interrupcao

        if (opt4001_test_interrupted) {
            printf("\n   --- INTERRUPCAO DETECTADA ---\n");
            if (opt4001_get_data(&test_data)) {
                printf("   Lux: %.2f\n", test_data.lux);
                printf("   Flags: Overload=%d, ConvReady=%d, FlagH=%d, FlagL=%d\n",
                       test_data.overload_flag, test_data.conversion_ready,
                       test_data.flag_h, test_data.flag_l);
                if (test_data.flag_h) printf("   ALERTA: Luz acima do limite alto!\n");
                if (test_data.flag_l) printf("   ALERTA: Luz abaixo do limite baixo!\n");
            } else {
                printf("   Falha ao ler dados do sensor apos interrupcao.\n");
            }
            printf("   ------------------------------\n");
        } else {
            printf("   Acordou, mas sem interrupcao do sensor (pode ter sido outra interrupcao ou timeout).\n");
        }
        sleep_ms(500); // Pequeno delay antes de dormir novamente, para evitar spam
    }
    printf("   Teste de interrupcao concluido. Verificacao visual NECESSARIA.\n");
    return true;
}


// --- Main Function for Tests ---
int main() {
    stdio_init_all();
    sleep_ms(2000); // Dar tempo para o terminal abrir

    printf("\n--- Iniciando Suite de Testes da Biblioteca OPT4001 ---\n");

    // Executa os testes em sequencia
    run_test("1. Inicializacao e Device ID", test_1_init_and_device_id);
    // Se a inicializacao falhar, e provavel que os proximos testes de I2C tambem falhem.
    
    run_test("2. Leitura/Escrita de Registrador (Acesso Direto)", test_2_read_write_basic_register_access);
    run_test("3. Modo de Operacao e Faixa", test_3_operating_mode_and_range);
    run_test("4. Tempo de Conversao", test_4_conversion_time);
    run_test("5. Obter Dados e Calculo Lux", test_5_get_data_and_lux_calculation);
    run_test("6. Esperar por Conversao", test_6_wait_for_conversion);
    run_test("7. Setar Limiares Lux e Obter Flags", test_7_set_lux_thresholds_and_get_flags);
    run_test("8. Tratamento de Interrupcao (__wfi)", test_8_interrupt_handling);


    printf("\n--- Suite de Testes da Biblioteca OPT4001 Concluida ---\n");

    // Loop infinito para manter o programa rodando e permitir revisao da saida
    while (true) {
        __wfi(); // Dorme para economizar energia
    }

    return 0;
}
