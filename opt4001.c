#include "opt4001.h"
#include <stdio.h> // Para printf de depuração
#include <math.h>  // Para roundf

// Variáveis estáticas globais que serão preenchidas na função de inicialização
static i2c_inst_t *_i2c_instance;
static uint8_t _i2c_addr;
static opt4001_variant_t _sensor_variant;

// --- Funções Auxiliares Internas (static) ---

/**
 * @brief Função auxiliar interna para calcular o valor de lux a partir dos ADC_CODES.
 * @param adc_codes O valor linearizado de ADC_CODES.
 * @return O valor em Lux.
 */
static float opt4001_calculate_lux(uint32_t adc_codes) {
    // Seleciona o fator de lux com base na variante do sensor armazenada
    if (_sensor_variant == OPT4001_VARIANT_PICOSTAR) {
        return (float)adc_codes * OPT4001_LUX_FACTOR_PICOSTAR;
    } else { // OPT4001_VARIANT_SOT5X3
        return (float)adc_codes * OPT4001_LUX_FACTOR_SOT5X3;
    }
}

/**
 * @brief Função auxiliar interna que converte um valor de Lux desejado para os valores de Expoente e Resultado (Mantissa)
 * apropriados para os registradores de limiar. Tornada 'static' para encapsulamento.
 * @param desired_lux O valor de Lux que se deseja definir para o limiar.
 * @return Uma estrutura opt4001_threshold_value_t contendo o expoente, resultado e uma flag de validade.
 */
static opt4001_threshold_value_t opt4001_lux_to_threshold_reg_values(float desired_lux) {
    opt4001_threshold_value_t threshold_val = {0}; // Inicializa todos os campos como 0
    threshold_val.valid = false; // Assume inválido até ser calculado com sucesso

    if (desired_lux <= 0) {
        printf("OPT4001_lux_to_threshold_reg_values: Lux desejado deve ser positivo.\n");
        return threshold_val;
    }

    float lux_factor;
    // Seleciona o fator de lux com base na variante do sensor armazenada
    if (_sensor_variant == OPT4001_VARIANT_PICOSTAR) {
        lux_factor = OPT4001_LUX_FACTOR_PICOSTAR;
    } else { // OPT4001_VARIANT_SOT5X3
        lux_factor = OPT4001_LUX_FACTOR_SOT5X3;
    }

    // 1. Converta o Lux desejado para ADC_CODES_Desejado
    float adc_codes_float = desired_lux / lux_factor;

    // 2. Determine o Expoente do Limiar (EXPONENT_LIMIAR)
    // O Expoente vai de 0 a 8 .
    // A Mantissa do limiar (result) é de 12 bits (0 a 4095, ou 0xFFF) 
    // Formula interna de ADC_CODES_TH = RES_LIMIAR << (8 + EXPONENT_LIMIAR) 
    // Então, RES_LIMIAR = ADC_CODES_Desejado / (1 << (8 + EXPONENT_LIMIAR))
    // Precisamos encontrar EXPONENT_LIMIAR tal que RES_LIMIAR caiba em 12 bits.

    uint8_t exp_limiar = 0;
    uint32_t res_limiar_candidate; // Usar uint32_t para evitar overflow temporário

    // Itera pelos possíveis expoentes (0 a 8) 
    for (exp_limiar = 0; exp_limiar <= 8; exp_limiar++) {
        // Calcular o valor do deslocamento (2^(8+EXPONENT))
        uint32_t shift_value = 1 << (8 + exp_limiar);
        
        // Calcular a Mantissa_Candidata
        res_limiar_candidate = (uint32_t)roundf(adc_codes_float / (float)shift_value);

        // Se a Mantissa_Candidata cabe em 12 bits (0 a 4095)
        if (res_limiar_candidate >= 0x0 && res_limiar_candidate <= 0xFFF) { // 0xFFF é 4095
            threshold_val.exponent = exp_limiar;
            threshold_val.result = (uint16_t)res_limiar_candidate;
            threshold_val.valid = true;
            return threshold_val;
        }
    }
    
    // Se nenhum expoente encontrou uma Mantissa_Candidata válida dentro da faixa de 12 bits
    printf("OPT4001_lux_to_threshold_reg_values: Nao foi possivel encontrar expoente/resultado validos para Lux %.2f\n", desired_lux);
    return threshold_val;
}

/**
 * @brief Função auxiliar interna para definir os limites de limiar (thresholds) brutos no sensor.
 * Tornada 'static' para encapsulamento. Usada por opt4001_set_lux_thresholds.
 * @param low_exp Expoente do limiar baixo.
 * @param low_res Resultado/Mantissa (12 bits) do limiar baixo.
 * @param high_exp Expoente do limiar alto.
 * @param high_res Resultado/Mantissa (12 bits) do limiar alto.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
static bool opt4001_set_thresholds(uint8_t low_exp, uint16_t low_res, uint8_t high_exp, uint16_t high_res) {
    // Combina o expoente (4 bits) e o resultado/mantissa (12 bits) para formar o valor de 16 bits para o registrador.
    // reg_val_08: Bits 15-12 = THRESHOLD_L_EXPONENT; Bits 11-0 = THRESHOLD_L_RESULT 
    uint16_t low_threshold_reg = (low_exp << 12) | (low_res & 0x0FFF);
    // reg_val_09: Bits 15-12 = THRESHOLD_H_EXPONENT; Bits 11-0 = THRESHOLD_H_RESULT 
    uint16_t high_threshold_reg = (high_exp << 12) | (high_res & 0x0FFF);

    // Escreve os valores nos registradores de limiar baixo e alto.
    if (!opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, low_threshold_reg)) {
        return false;
    }
    if (!opt4001_write_register(OPT4001_REG_THRESHOLD_H_EXPONENT, high_threshold_reg)) {
        return false;
    }
    return true;
}


// --- Funções Auxiliares de Baixo Nível (Públicas para Teste/Depuração) ---

/**
 * @brief Lê um registrador de 16 bits do sensor via I2C.
 * @param reg_address Endereço do registrador a ser lido.
 * @param value Ponteiro para armazenar o valor lido.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool opt4001_read_register(uint8_t reg_address, uint16_t *value) {
    uint8_t rx_data[2]; // Buffer para 2 bytes de dados lidos
    int ret;

    // 1. Envia o endereço do registrador que queremos ler .
    ret = i2c_write_blocking(_i2c_instance, _i2c_addr, &reg_address, 1, true);
    if (ret != 1) { // Verifica se o endereço do registrador foi enviado com sucesso (1 byte).
        return false;
    }

    // 2. Lê os 2 bytes de dados do registrador .
    ret = i2c_read_blocking(_i2c_instance, _i2c_addr, rx_data, 2, false);
    if (ret != 2) { // Verifica se 2 bytes foram lidos com sucesso.
        return false;
    }

    // 3. Reconstrói o valor de 16 bits a partir dos 2 bytes lidos.
    *value = (rx_data[0] << 8) | rx_data[1];
    return true;
}

/**
 * @brief Escreve um valor de 16 bits em um registrador do sensor via I2C.
 * @param reg_address Endereço do registrador a ser escrito.
 * @param value Valor de 16 bits a ser escrito.
 * @return true se a escrita for bem-sucedida, false caso contrário.
 */
bool opt4001_write_register(uint8_t reg_address, uint16_t value) {
    uint8_t tx_data[3]; // Buffer para o endereço do registrador e os 2 bytes de dados.
    int ret;

    // Preenche o buffer de transmissão.
    tx_data[0] = reg_address;                  // Byte 0: Endereço do registrador 
    tx_data[1] = (uint8_t)(value >> 8);        // Byte 1: MSB do valor de 16 bits 
    tx_data[2] = (uint8_t)(value & 0xFF);      // Byte 2: LSB do valor de 16 bits 

    // Realiza a escrita bloqueante de 3 bytes (endereço + MSB + LSB) no sensor.
    ret = i2c_write_blocking(_i2c_instance, _i2c_addr, tx_data, 3, false);
    if (ret != 3) { // Verifica se 3 bytes foram enviados com sucesso.
        return false;
    }
    return true;
}


// --- Funções da API Pública Principal ---

/**
 * @brief Inicializa a comunicação com o sensor OPT4001.
 * @param i2c_port Instância I2C (i2c0 ou i2c1).
 * @param sda_pin Pino GPIO para SDA.
 * @param scl_pin Pino GPIO para SCL.
 * @param i2c_address Endereço I2C do sensor.
 * @param variant Variante do sensor (PicoStar ou SOT-5X3).
 * @param baudrate Taxa de baud do I2C.
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool opt4001_init(i2c_inst_t *i2c_port, uint8_t sda_pin, uint8_t scl_pin, uint8_t i2c_address, opt4001_variant_t variant, uint baudrate) {
    // Armazena os parâmetros de hardware nas variáveis estáticas
    _i2c_instance = i2c_port;
    _i2c_addr = i2c_address;
    _sensor_variant = variant;

    // Inicializa o periférico I2C na Raspberry Pi Pico W
    i2c_init(_i2c_instance, baudrate);
    // Configura as funções dos pinos GPIO para I2C
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    // Habilita os resistores de pull-up internos nos pinos I2C 
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Pequeno atraso para o sensor inicializar após a energização
    sleep_ms(100);

    // Tenta ler o Device ID para verificar a comunicação com o sensor 
    uint16_t device_id;
    if (!opt4001_read_register(OPT4001_REG_DEVICE_ID, &device_id)) {
        printf("OPT4001: Falha ao ler Device ID. Verifique conexoes e endereco I2C.\n");
        return false;
    }
    printf("OPT4001: Device ID lido: 0x%04X\n", device_id);

    // Configura o sensor para o modo operacional padrão de Power-down
    uint16_t config_reg_value = 0;
    if (!opt4001_write_register(OPT4001_REG_CONFIG, config_reg_value)) {
        printf("OPT4001: Falha ao configurar o modo inicial (Power-down).\n");
        return false;
    }
    printf("OPT4001: Configurado para modo Power-down inicialmente.\n");

    return true;
}


/**
 * @brief Define o tempo de conversão do sensor.
 * @param conversion_time_reg_value Valor de opt4001_conversion_time_t.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_conversion_time(opt4001_conversion_time_t conversion_time_reg_value) {
    uint16_t current_config;
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &current_config)) {
        return false;
    }
    current_config = (current_config & ~((0xF) << 6)) | (conversion_time_reg_value << 6);
    return opt4001_write_register(OPT4001_REG_CONFIG, current_config);
}

/**
 * @brief Define o modo de operação do sensor.
 * @param mode Valor de opt4001_operating_mode_t.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_operating_mode(opt4001_operating_mode_t mode) {
    uint16_t current_config;
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &current_config)) {
        return false;
    }
    current_config = (current_config & ~((0x3) << 4)) | (mode << 4);
    return opt4001_write_register(OPT4001_REG_CONFIG, current_config);
}

/**
 * @brief Define a faixa de escala do sensor (ganho) .
 * @param range_reg_value Valor de opt4001_range_t.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_range(opt4001_range_t range_reg_value) {
    uint16_t current_config;
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &current_config)) {
        return false;
    }
    current_config = (current_config & ~((0xF) << 10)) | (range_reg_value << 10);
    return opt4001_write_register(OPT4001_REG_CONFIG, current_config);
}

/**
 * @brief Lê todos os dados de resultado e status do sensor e os armazena na estrutura [cite: 2026-2027].
 * @param data Ponteiro para a estrutura opt4001_data_t para armazenar os dados lidos.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool opt4001_get_data(opt4001_data_t *data) {
    uint16_t reg_val_00, reg_val_01, reg_val_0C;

    if (!opt4001_read_register(OPT4001_REG_RESULT_MSB, &reg_val_00)) return false; 
    if (!opt4001_read_register(OPT4001_REG_RESULT_LSB, &reg_val_01)) return false; 
    if (!opt4001_read_register(OPT4001_REG_INTERRUPT_FLAGS, &reg_val_0C)) return false; 

    data->raw_result_msb = reg_val_00;
    data->raw_result_lsb = reg_val_01;

    data->exponent = (reg_val_00 >> 12) & 0x0F;
    data->mantissa = ((reg_val_00 & 0x0FFF) << 8) | ((reg_val_01 >> 8) & 0xFF);
    data->adc_codes = data->mantissa << data->exponent;
    data->lux = opt4001_calculate_lux(data->adc_codes);
    data->counter = (reg_val_01 >> 4) & 0x0F;
    data->crc = reg_val_01 & 0x0F;

    data->overload_flag = (reg_val_0C >> 3) & 0x01;
    data->conversion_ready = (reg_val_0C >> 2) & 0x01;
    data->flag_h = (reg_val_0C >> 1) & 0x01;
    data->flag_l = reg_val_0C & 0x01;

    return true;
}

/**
 * @brief Espera a conversão do sensor ser concluída, fazendo polling no CONVERSION_READY_FLAG [cite: 2021-2023].
 * @param timeout_ms Tempo limite em milissegundos para esperar a conversão.
 * @return true se a conversão for concluída dentro do tempo limite, false caso contrário.
 */
bool opt4001_wait_for_conversion_complete(uint32_t timeout_ms) {
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint16_t flags_reg;

    while (to_ms_since_boot(get_absolute_time()) - start_time < timeout_ms) {
        if (!opt4001_read_register(OPT4001_REG_INTERRUPT_FLAGS, &flags_reg)) {
            return false;
        }
        if ((flags_reg >> 2) & 0x01) {
            return true;
        }
        sleep_ms(1);
    }
    return false;
}

/**
 * @brief Obtém o estado das flags de interrupção (overload, conv_ready, flag_h, flag_l) [cite: 2596-2601].
 * @param overload Ponteiro para armazenar o estado da flag de sobrecarga.
 * @param conv_ready Ponteiro para armazenar o estado da flag de conversão concluída.
 * @param flag_h Ponteiro para armazenar o estado da flag de limite alto.
 * @param flag_l Ponteiro para armazenar o estado da flag de limite baixo.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool opt4001_get_flags(bool *overload, bool *conv_ready, bool *flag_h, bool *flag_l) {
    uint16_t flags_reg;
    if (!opt4001_read_register(OPT4001_REG_INTERRUPT_FLAGS, &flags_reg)) {
        return false;
    }
    *overload = (flags_reg >> 3) & 0x01;
    *conv_ready = (flags_reg >> 2) & 0x01;
    *flag_h = (flags_reg >> 1) & 0x01;
    *flag_l = flags_reg & 0x01;
    return true;
}

/**
 * @brief Define os limites de limiar (thresholds) para o sensor, passando valores de Lux diretamente.
 * @param low_lux O valor de Lux para o limiar baixo.
 * @param high_lux O valor de Lux para o limiar alto.
 * @return true se a configuração for bem-sucedida e os valores de Lux forem válidos, false caso contrário.
 */
bool opt4001_set_lux_thresholds(float low_lux, float high_lux) {
    opt4001_threshold_value_t low_thresh_val = opt4001_lux_to_threshold_reg_values(low_lux);
    if (!low_thresh_val.valid) {
        printf("OPT4001_set_lux_thresholds: Calculo do limiar baixo (%f Lux) falhou.\n", low_lux);
        return false;
    }

    opt4001_threshold_value_t high_thresh_val = opt4001_lux_to_threshold_reg_values(high_lux);
    if (!high_thresh_val.valid) {
        printf("OPT4001_set_lux_thresholds: Calculo do limiar alto (%f Lux) falhou.\n", high_lux);
        return false;
    }

    return opt4001_set_thresholds(
        low_thresh_val.exponent,
        low_thresh_val.result,
        high_thresh_val.exponent,
        high_thresh_val.result
    );
}

/**
 * @brief Configura o comportamento do pino de interrupção (INT) do sensor 
 * @param polarity Polaridade do pino INT (Ativo Baixo/Alto).
 * @param fault_count Número de falhas consecutivas para disparar a interrupção.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_config_interrupt_pin(opt4001_int_polarity_t polarity, opt4001_fault_count_t fault_count) {
    uint16_t config_reg_0A;
    uint16_t config_reg_0B;

    if (!opt4001_read_register(OPT4001_REG_CONFIG, &config_reg_0A)) {
        return false;
    }
    if (!opt4001_read_register(OPT4001_REG_I2C_CONFIG, &config_reg_0B)) {
        return false;
    }

    // Configurar REG_CONFIG (0x0A): INT_POL (bit 2) e FAULT_COUNT (bits 1-0) 
    config_reg_0A &= ~((0x01 << 2) | (0x03 << 0));
    config_reg_0A |= (polarity << 2) | (fault_count << 0);

    // Configurar REG_I2C_CONFIG (0x0B): INT_DIR (bit 4) e INT_CFG (bits 3-2) 
    config_reg_0B &= ~((0x01 << 4) | (0x03 << 2));
    config_reg_0B |= (1 << 4) | (0 << 2);

    if (!opt4001_write_register(OPT4001_REG_CONFIG, config_reg_0A)) {
        return false;
    }
    if (!opt4001_write_register(OPT4001_REG_I2C_CONFIG, config_reg_0B)) {
        return false;
    }

    return true;
}