// test_opt4001.c
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "opt4001.h"

// --- Definições de Hardware para o Teste ---
#define I2C_TEST_BAUDRATE 100 * 1000 // 100 kHz (modo padrão)
#define I2C_PORT_USED i2c0
#define SDA_PIN_USED 4
#define SCL_PIN_USED 5
#define SENSOR_I2C_ADDRESS 0x45
#define SENSOR_VARIANT OPT4001_VARIANT_SOT5X3
#define OPT4001_TEST_INT_GPIO_PIN 13

// --- Variáveis para Gerenciamento de Interrupção no Teste ---
volatile bool opt4001_test_interrupted = false;

void opt4001_test_int_gpio_callback(uint gpio, uint32_t events) {
    if (gpio == OPT4001_TEST_INT_GPIO_PIN) {
        opt4001_test_interrupted = true;
    }
}

// --- Função auxiliar para rodar um teste e imprimir o resultado ---
static void run_test(const char* test_name, bool (*test_func)(void)) {
    printf("\n--- TESTE: %s ---\n", test_name);
    if (test_func()) {
        printf("RESULTADO: SUCESSO\n");
    } else {
        printf("RESULTADO: FALHA\n");
    }
    printf("---------------------------\n");
    sleep_ms(100);
}

// --- Funções de Teste Individuais ---

bool test_1_init_and_reset() {
    printf("1.1: Testando opt4001_init()...\n");
    if (!opt4001_init(I2C_PORT_USED, SDA_PIN_USED, SCL_PIN_USED, SENSOR_I2C_ADDRESS, SENSOR_VARIANT, I2C_TEST_BAUDRATE)) {
        printf("   Falha na inicializacao I2C. Verifique hardware.\n");
        return false;
    }
    printf("   opt4001_init() executado com sucesso.\n");

}

bool test_2_low_level_register_access() {
    printf("2.1: Testando opt4001_read_register() e opt4001_write_register()...\n");
    uint16_t test_value = 0xABCD;
    uint16_t read_value = 0;
    uint16_t original_reg_08;

    if (!opt4001_read_register(OPT4001_REG_THRESHOLD_L_EXPONENT, &original_reg_08)) {
        printf("   Falha ao ler REG_THRESHOLD_L_EXPONENT original.\n");
        return false;
    }
    printf("   Escrevendo 0x%04X no REG_THRESHOLD_L_EXPONENT (0x08)...\n", test_value);
    if (!opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, test_value)) {
        printf("   Falha na escrita.\n");
        return false;
    }
    sleep_ms(10);
    if (!opt4001_read_register(OPT4001_REG_THRESHOLD_L_EXPONENT, &read_value)) {
        printf("   Falha na leitura.\n");
        opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, original_reg_08);
        return false;
    }

    if (read_value != test_value) {
        printf("   Erro: Valor escrito (0x%04X) nao corresponde ao lido (0x%04X).\n", test_value, read_value);
        opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, original_reg_08);
        return false;
    }
    opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, original_reg_08); // Restaurar o valor original
    printf("   Acesso de baixo nível verificado. Valor lido corresponde ao escrito.\n");
    return true;
}

bool test_3_mode_and_range_config() {
    printf("3.1: Testando opt4001_set_operating_mode()...\n");
    if (!opt4001_set_operating_mode(OPT4001_MODE_CONTINUOUS)) return false;
    uint16_t config_reg;
    opt4001_read_register(OPT4001_REG_CONFIG, &config_reg);
    if (((config_reg >> 4) & 0x03) != OPT4001_MODE_CONTINUOUS) {
        printf("   Falha: Modo Continuo nao configurado.\n");
        return false;
    }

    printf("3.2: Testando opt4001_set_range()...\n");
    if (!opt4001_set_range(OPT4001_RANGE_AUTO)) return false;
    opt4001_read_register(OPT4001_REG_CONFIG, &config_reg);
    if (((config_reg >> 10) & 0x0F) != OPT4001_RANGE_AUTO) {
        printf("   Falha: Auto-Range nao configurado.\n");
        return false;
    }
    return true;
}

bool test_4_conversion_time_config() {
    printf("4: Testando opt4001_set_conversion_time()...\n");
    opt4001_conversion_time_t test_time = OPT4001_CONV_TIME_100MS;
    if (!opt4001_set_conversion_time(test_time)) return false;
    uint16_t config_reg;
    opt4001_read_register(OPT4001_REG_CONFIG, &config_reg);
    if (((config_reg >> 6) & 0x0F) != test_time) {
        printf("   Falha: Tempo de conversao nao configurado (lido: %d, esperado: %d).\n", (config_reg >> 6) & 0x0F, test_time);
        return false;
    }
    return true;
}

bool test_5_get_data_and_lux_calculation() {
    printf("5: Testando opt4001_get_data()...\n");
    opt4001_data_t test_data;
    opt4001_set_operating_mode(OPT4001_MODE_CONTINUOUS);
    opt4001_set_range(OPT4001_RANGE_AUTO);
    opt4001_set_conversion_time(OPT4001_CONV_TIME_400MS);
    sleep_ms(500); // Esperar pelo menos uma conversao
    if (!opt4001_get_data(&test_data)) {
        printf("   Falha ao obter dados do sensor.\n");
        return false;
    }
    printf("   Lux lido: %.2f\n", test_data.lux);
    if (test_data.lux < 0) { // Simples sanity check
        printf("   Erro: Lux lido deve ser positivo.\n");
        return false;
    }
    return true;
}

bool test_6_wait_for_conversion() {
    printf("6: Testando opt4001_wait_for_conversion_complete()...\n");
    if (!opt4001_set_operating_mode(OPT4001_MODE_ONE_SHOT)) return false;
    if (!opt4001_set_conversion_time(OPT4001_CONV_TIME_100MS)) return false;
    sleep_ms(100); // Dar tempo para o sensor entrar em power-down
    printf("   Disparando conversao one-shot e esperando...\n");
    uint16_t config_reg;
    opt4001_read_register(OPT4001_REG_CONFIG, &config_reg);
    config_reg = (config_reg & ~((0x3) << 4)) | (OPT4001_MODE_FORCE_AUTO_ONE_SHOT << 4);
    if (!opt4001_write_register(OPT4001_REG_CONFIG, config_reg)) return false;
    if (!opt4001_wait_for_conversion_complete(200)) {
        printf("   Falha: Conversao nao concluida a tempo.\n");
        return false;
    }
    return true;
}

bool test_7_lux_threshold_config() {
    printf("7: Testando opt4001_set_lux_thresholds()...\n");
    float low_lux = 10.0f;
    float high_lux = 1000.0f;
    if (!opt4001_set_lux_thresholds(low_lux, high_lux)) {
        printf("   Falha ao configurar limiares.\n");
        return false;
    }
    printf("   Limiares para interrupcao configurados: Baixo %.2f Lux, Alto %.2f Lux.\n", low_lux, high_lux);
    
    // Teste de validacao (verifique a saida das funcoes internas, por exemplo, o valor cru)
    uint16_t low_reg_val, high_reg_val;
    opt4001_read_register(OPT4001_REG_THRESHOLD_L_EXPONENT, &low_reg_val);
    opt4001_read_register(OPT4001_REG_THRESHOLD_H_EXPONENT, &high_reg_val);
    printf("   Valores brutos dos registradores: Baixo=0x%04X, Alto=0x%04X\n", low_reg_val, high_reg_val);
    // Verificacao manual seria necessaria aqui para confirmar que os valores brutos estao corretos.
    return true;
}

bool test_8_interrupt_pin_config() {
    printf("8: Testando opt4001_config_interrupt_pin()...\n");
    opt4001_int_polarity_t pol = OPT4001_INT_POLARITY_ACTIVE_LOW;
    opt4001_fault_count_t fc = OPT4001_FAULT_COUNT_2_FAULTS;
    if (!opt4001_config_interrupt_pin(pol, fc)) return false;
    
    uint16_t config_reg_0A, config_reg_0B;
    opt4001_read_register(OPT4001_REG_CONFIG, &config_reg_0A);
    opt4001_read_register(OPT4001_REG_I2C_CONFIG, &config_reg_0B);
    
    if (((config_reg_0A >> 2) & 0x01) != pol ||
        (config_reg_0A & 0x03) != fc ||
        ((config_reg_0B >> 4) & 0x01) != 1 || // INT_DIR deve ser 1 (Output)
        ((config_reg_0B >> 2) & 0x03) != 0) { // INT_CFG deve ser 0
        printf("   Falha: Configuracao do pino INT incorreta.\n");
        return false;
    }
    return true;
}

bool test_9_get_flags() {
    printf("9: Testando opt4001_get_flags()...\n");
    bool ov, cr, fh, fl;
    if (!opt4001_get_flags(&ov, &cr, &fh, &fl)) {
        printf("   Falha ao ler flags.\n");
        return false;
    }
    printf("   Flags lidas: Overload=%d, ConvReady=%d, FlagH=%d, FlagL=%d\n", ov, cr, fh, fl);
    return true;
}

// --- Main Function for Tests ---
int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n--- INICIANDO SUITE DE TESTES DA BIBLIOTECA OPT4001 ---\n");

    // Sequencia de testes
    run_test("1: Inicializacao e Reset", test_1_init_and_reset);
    run_test("2: Acesso a Registrador (Read/Write)", test_2_low_level_register_access);
    run_test("3: Config. de Modo e Faixa", test_3_mode_and_range_config);
    run_test("4: Config. de Tempo de Conversao", test_4_conversion_time_config);
    run_test("5: Obter Dados e Calculo Lux", test_5_get_data_and_lux_calculation);
    run_test("6: Esperar por Conversao (One-Shot)", test_6_wait_for_conversion);
    run_test("7: Config. Limiares de Lux", test_7_lux_threshold_config);
    run_test("8: Config. Pino de Interrupcao", test_8_interrupt_pin_config);
    run_test("9: Leitura de Flags", test_9_get_flags);

    printf("\n--- SUITE DE TESTES CONCLUIDA ---\n");
    printf("Verifique a saida para confirmar a correcao dos valores.\n");
    
    // Loop infinito para manter o programa rodando e permitir revisao da saida
    while (true) {
        __wfi();
    }

    return 0;
}