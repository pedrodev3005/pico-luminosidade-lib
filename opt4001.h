#ifndef OPT4001_H
#define OPT4001_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>

// --- Novos Enums para Configuração de Interrupção ---
typedef enum {
    OPT4001_INT_POLARITY_ACTIVE_LOW  = 0,
    OPT4001_INT_POLARITY_ACTIVE_HIGH = 1
} opt4001_int_polarity_t;

typedef enum {
    OPT4001_FAULT_COUNT_1_FAULT = 0, // 1 medição falha 
    OPT4001_FAULT_COUNT_2_FAULTS = 1, // 2 medições falhas consecutivas 
    OPT4001_FAULT_COUNT_4_FAULTS = 2, // 4 medições falhas consecutivas 
    OPT4001_FAULT_COUNT_8_FAULTS = 3  // 8 medições falhas consecutivas 
} opt4001_fault_count_t;


// --- Estrutura para os Limiares Calculados ---
typedef struct {
    uint8_t  exponent;
    uint16_t result; // Corresponde à Mantissa do limiar (12 bits)
    bool     valid;  // Indica se o cálculo foi bem-sucedido e está dentro dos limites
} opt4001_threshold_value_t;

// --- Mapeamento de registradores ---
#define OPT4001_REG_RESULT_MSB          0x00 // EXPONENT e RESULT_MSB [cite: 1085-1086]
#define OPT4001_REG_RESULT_LSB          0x01 // RESULT_LSB, COUNTER e CRC [cite: 1085-1086]
#define OPT4001_REG_CONFIG              0x0A // RANGE, CONVERSION_TIME, OPERATING_MODE, LATCH, INT_POL, FAULT_COUNT [cite: 1085-1086]
#define OPT4001_REG_INTERRUPT_FLAGS     0x0C // OVERLOAD_FLAG, CONVERSION_READY_FLAG, FLAG_H, FLAG_L [cite: 1085-1086]
#define OPT4001_REG_DEVICE_ID           0x11 // DIDL e DIDH (identificação do sensor) [cite: 1085-1086]
#define OPT4001_REG_THRESHOLD_L_EXPONENT 0x8 // 
#define OPT4001_REG_THRESHOLD_H_EXPONENT 0x9 // 
#define OPT4001_REG_I2C_CONFIG          0x0B // INT_DIR, INT_CFG, I2C_BURST, etc. [cite: 1085-1086]


// Constantes de cálculo de Lux
#define OPT4001_LUX_FACTOR_SOT5X3       437.5E-6f // para SOT-5X3 
#define OPT4001_LUX_FACTOR_PICOSTAR     312.5E-6f // para PicoStar™ 

// --- Enum para a variante do sensor ---
typedef enum {
    OPT4001_VARIANT_PICOSTAR,
    OPT4001_VARIANT_SOT5X3
} opt4001_variant_t;


// --- Enum para Configurações ---
typedef enum {
    OPT4001_MODE_POWER_DOWN         = 0b00, // Desligamento, menor consumo 
    OPT4001_MODE_FORCE_AUTO_ONE_SHOT= 0b01, // Disparo único com auto-range forçado 
    OPT4001_MODE_ONE_SHOT           = 0b10, // Disparo único (usa range anterior para auto-range) 
    OPT4001_MODE_CONTINUOUS         = 0b11  // Medição contínua 
} opt4001_operating_mode_t;

// Valores de configuração para CONVERSION_TIME 
typedef enum {
    OPT4001_CONV_TIME_600US  = 0x0,
    OPT4001_CONV_TIME_1MS    = 0x1,
    OPT4001_CONV_TIME_1_8MS  = 0x2,
    OPT4001_CONV_TIME_3_4MS  = 0x3,
    OPT4001_CONV_TIME_6_5MS  = 0x4,
    OPT4001_CONV_TIME_12_7MS = 0x5,
    OPT4001_CONV_TIME_25MS   = 0x6,
    OPT4001_CONV_TIME_50MS   = 0x7,
    OPT4001_CONV_TIME_100MS  = 0x8,
    OPT4001_CONV_TIME_200MS  = 0x9,
    OPT4001_CONV_TIME_400MS  = 0xA,
    OPT4001_CONV_TIME_800MS  = 0xB
} opt4001_conversion_time_t;

// Valores de configuração para RANGE 
typedef enum {
    OPT4001_RANGE_MANUAL_0      = 0x0,
    OPT4001_RANGE_MANUAL_1      = 0x1,
    // ... até 0x8
    OPT4001_RANGE_MANUAL_8      = 0x8,
    OPT4001_RANGE_AUTO          = 0xC
} opt4001_range_t;


// --- Estrutura para os Dados de Leitura do Sensor ---
typedef struct {
    uint16_t raw_result_msb; // Conteúdo bruto do registrador 0x00 
    uint16_t raw_result_lsb; // Conteúdo bruto do registrador 0x01 
    float lux;               // Valor da iluminação em Lux
    uint8_t exponent;        // Expoente da medição (4 bits, bits 15-12 de 0x00)
    uint32_t mantissa;       // Mantissa da medição (20 bits)
    uint32_t adc_codes;      // Valor linearizado ADC (até 28 bits)
    uint8_t counter;         // Contador de amostras (4 bits, bits 7-4 de 0x01)
    uint8_t crc;             // CRC (4 bits, bits 3-0 de 0x01)
    bool overload_flag;      // Flag de sobrecarga (bit 3 de 0x0C)
    bool conversion_ready;   // Flag de conversão concluída (bit 2 de 0x0C)
    bool flag_h;             // Flag de limite alto (bit 1 de 0x0C)
    bool flag_l;             // Flag de limite baixo (bit 0 de 0x0C)
} opt4001_data_t;


// --- Funções da API Pública da Biblioteca ---

/**
 * @brief Inicializa a comunicação com o sensor OPT4001.
 * Esta função é agora configurável para qualquer pino GPIO e endereço I2C.
 * @param i2c_port Instância I2C (i2c0 ou i2c1).
 * @param sda_pin Pino GPIO para SDA.
 * @param scl_pin Pino GPIO para SCL.
 * @param i2c_address Endereço I2C do sensor.
 * @param variant Variante do sensor (PicoStar ou SOT-5X3).
 * @param baudrate Taxa de baud do I2C.
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool opt4001_init(i2c_inst_t *i2c_port, uint8_t sda_pin, uint8_t scl_pin, uint8_t i2c_address, opt4001_variant_t variant, uint baudrate);


/**
 * @brief Define o tempo de conversão do sensor.
 * @param conversion_time_reg_value Valor de opt4001_conversion_time_t.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_conversion_time(opt4001_conversion_time_t conversion_time_reg_value);

/**
 * @brief Define o modo de operação do sensor.
 * @param mode Valor de opt4001_operating_mode_t.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_operating_mode(opt4001_operating_mode_t mode);

/**
 * @brief Define a faixa de escala do sensor (ganho) .
 * @param range_reg_value Valor de opt4001_range_t.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_range(opt4001_range_t range_reg_value);

/**
 * @brief Lê todos os dados de resultado e status do sensor e os armazena na estrutura .
 * @param data Ponteiro para a estrutura opt4001_data_t para armazenar os dados lidos.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool opt4001_get_data(opt4001_data_t *data);

/**
 * @brief Espera a conversão do sensor ser concluída, fazendo polling no CONVERSION_READY_FLAG .
 * @param timeout_ms Tempo limite em milissegundos para esperar a conversão.
 * @return true se a conversão for concluída dentro do tempo limite, false caso contrário.
 */
bool opt4001_wait_for_conversion_complete(uint32_t timeout_ms);

/**
 * @brief Obtém o estado das flags de interrupção (overload, conv_ready, flag_h, flag_l) .
 * @param overload Ponteiro para armazenar o estado da flag de sobrecarga.
 * @param conv_ready Ponteiro para armazenar o estado da flag de conversão concluída.
 * @param flag_h Ponteiro para armazenar o estado da flag de limite alto.
 * @param flag_l Ponteiro para armazenar o estado da flag de limite baixo.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool opt4001_get_flags(bool *overload, bool *conv_ready, bool *flag_h, bool *flag_l);

/**
 * @brief Define os limites de limiar (thresholds) para o sensor, passando valores de Lux diretamente.
 * @param low_lux O valor de Lux para o limiar baixo.
 * @param high_lux O valor de Lux para o limiar alto.
 * @return true se a configuração for bem-sucedida e os valores de Lux forem válidos, false caso contrário.
 */
bool opt4001_set_lux_thresholds(float low_lux, float high_lux);

/**
 * @brief Configura o comportamento do pino de interrupção (INT) do sensor.
 * @param polarity Polaridade do pino INT (Ativo Baixo/Alto).
 * @param fault_count Número de falhas consecutivas para disparar a interrupção.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_config_interrupt_pin(opt4001_int_polarity_t polarity, opt4001_fault_count_t fault_count);

// --- Funções Auxiliares de Baixo Nível (Públicas para Teste/Depuração) ---

/**
 * @brief Lê um registrador de 16 bits do sensor via I2C.
 * EXPONDO PARA FINS DE TESTE E DEPURACAO.
 * @param reg_address Endereço do registrador a ser lido.
 * @param value Ponteiro para armazenar o valor lido.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool opt4001_read_register(uint8_t reg_address, uint16_t *value);

/**
 * @brief Escreve um valor de 16 bits em um registrador do sensor via I2C.
 * EXPONDO PARA FINS DE TESTE E DEPURACAO.
 * @param reg_address Endereço do registrador a ser escrito.
 * @param value Valor a ser escrito.
 * @return true se a escrita for bem-sucedida, false caso contrário.
 */
bool opt4001_write_register(uint8_t reg_address, uint16_t value);


#endif // OPT4001_H