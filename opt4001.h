#ifndef OPT4001_H
#define OPT4001_H


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Definições de hardware fixas para a PCB em questão
#define OPT4001_I2C_PORT i2c0
#define OPT4001_SCL_PIN 5     // Pino SCL fixo 
#define OPT4001_SDA_PIN 4     // Pino SDA fixo pode ser que seja o 8 (verificar)

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


// Endereço I2C do sensor OPT4001
#define OPT4001_ADDRESS 0x45

// Mapeamento de registradores
#define OPT4001_REG_RESULT_MSB      0x00 // EXPONENT e RESULT_MSB
#define OPT4001_REG_RESULT_LSB      0x01 // RESULT_LSB, COUNTER e CRC
#define OPT4001_REG_CONFIG          0x0A // RANGE, CONVERSION_TIME, OPERATING_MODE, LATCH, INT_POL, FAULT_COUNT
#define OPT4001_REG_INTERRUPT_FLAGS 0x0C // OVERLOAD_FLAG, CONVERSION_READY_FLAG, FLAG_H, FLAG_L
#define OPT4001_REG_DEVICE_ID       0x11 // DIDL e DIDH (identificação do sensor)
#define OPT4001_REG_THRESHOLD_L_EXPONENT 0x8
#define OPT4001_REG_THRESHOLD_H_EXPONENT 0x9
#define OPT4001_REG_I2C_CONFIG          0x0B // INT_DIR, INT_CFG, I2C_BURST, etc.


//Constantes de cálculo de Lux
#define OPT4001_LUX_FACTOR_SOT5X3   437.5E-6f // para SOT-5X3

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
    OPT4001_RANGE_459LUX    = 0x0, // 459 lux 
    OPT4001_RANGE_918LUX    = 0x1, // 918 lux 
    OPT4001_RANGE_1K8LUX    = 0x2, // 1.8 klux 
    OPT4001_RANGE_3K7LUX    = 0x3, // 3.7 klux 
    OPT4001_RANGE_7K3LUX    = 0x4, // 7.3 klux 
    OPT4001_RANGE_14K7LUX   = 0x5, // 14.7 klux 
    OPT4001_RANGE_29K4LUX   = 0x6, // 29.4 klux 
    OPT4001_RANGE_58K7LUX   = 0x7, // 58.7 klux 
    OPT4001_RANGE_117K4LUX  = 0x8, // 117.4 klux 
    OPT4001_RANGE_AUTO      = 0xC  // Auto-Range (recomendado)
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


// Funções da API da biblioteca
bool opt4001_init(uint baudrate);
bool opt4001_set_conversion_time(uint8_t conversion_time_reg_value); // De 0 a 11
bool opt4001_set_operating_mode(uint8_t mode); // 0: Power-down, 1: Forced auto-range One-shot, 2: One-shot, 3: Continuous
bool opt4001_set_range(uint8_t range_reg_value); // De 0 a 8 para manual, 0xC para auto-range
bool opt4001_get_data(opt4001_data_t *data); // Lê todos os registradores de resultado e status
float opt4001_calculate_lux(uint32_t adc_codes); // Função interna para calcular lux a partir de ADC_CODES (vai virar interna)
bool opt4001_read_register(uint8_t reg_address, uint16_t *value); // vai virar interna
bool opt4001_write_register(uint8_t reg_address, uint16_t value); // vai virar interna
bool opt4001_wait_for_conversion_complete(uint32_t timeout_ms); // Útil para modo one-shot
bool opt4001_set_thresholds(uint8_t low_exp, uint16_t low_res, uint8_t high_exp, uint16_t high_res); // vai virar interna
bool opt4001_get_flags(bool *overload, bool *conv_ready, bool *flag_h, bool *flag_l);
bool opt4001_set_lux_thresholds(float low_lux, float high_lux);
opt4001_threshold_value_t opt4001_lux_to_threshold_reg_values(float desired_lux);// vai virar interna
bool opt4001_config_interrupt_pin(opt4001_int_polarity_t polarity, opt4001_fault_count_t fault_count);

#endif // OPT4001_H