#include "opt4001.h"
#include <stdio.h>
#include <math.h>

// Variáveis estáticas para armazenar referências internas,
static i2c_inst_t *_i2c_instance = OPT4001_I2C_PORT;
static uint8_t _i2c_addr =  OPT4001_ADDRESS;


/**
 * @brief Configura o comportamento do pino de interrupção (INT) do sensor.
 * Define a polaridade e o número de falhas consecutivas necessárias.
 * Assume INT_DIR = 1 (Output) e INT_CFG = 0 (Table 8-1 behavior) para a interrupção baseada em limiares.
 * @param polarity Polaridade do pino INT (Ativo Baixo/Alto).
 * @param fault_count Número de falhas consecutivas para disparar a interrupção.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_config_interrupt_pin(opt4001_int_polarity_t polarity, opt4001_fault_count_t fault_count) {
    uint16_t config_reg_0A; // OPT4001_REG_CONFIG
    uint16_t config_reg_0B; // OPT4001_REG_I2C_CONFIG

    // 1. Ler os registradores de configuração atuais
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &config_reg_0A)) {
        return false;
    }
    if (!opt4001_read_register(OPT4001_REG_I2C_CONFIG, &config_reg_0B)) {
        return false;
    }

    // 2. Configurar REG_CONFIG (0x0A): INT_POL (bit 2) e FAULT_COUNT (bits 1-0) 
    // Limpa os bits existentes
    config_reg_0A &= ~((0x01 << 2) | (0x03 << 0)); // Limpa INT_POL e FAULT_COUNT
    // Define os novos valores
    config_reg_0A |= (polarity << 2) | (fault_count << 0);

    // 3. Configurar REG_I2C_CONFIG (0x0B): INT_DIR (bit 4) e INT_CFG (bits 3-2)
    // Para interrupção baseada em limiares: INT_DIR = 1 (Output), INT_CFG = 0 
    // Limpa os bits existentes
    config_reg_0B &= ~((0x01 << 4) | (0x03 << 2)); // Limpa INT_DIR e INT_CFG
    // Define os novos valores: INT_DIR = 1 (Output), INT_CFG = 0 
    config_reg_0B |= (1 << 4) | (0 << 2); // (1 << 4) sets INT_DIR to Output; (0 << 2) sets INT_CFG to 0

    // 4. Escrever os valores de volta nos registradores
    if (!opt4001_write_register(OPT4001_REG_CONFIG, config_reg_0A)) {
        return false;
    }
    if (!opt4001_write_register(OPT4001_REG_I2C_CONFIG, config_reg_0B)) {
        return false;
    }

    return true;
}


/**
 * @brief Define os limites de limiar (thresholds) para o sensor.
 * Esta função calcula os valores de expoente e resultado (mantissa) para os registradores
 * de limiar automaticamente a partir dos valores de Lux desejados.
 * @param low_lux O valor de Lux para o limiar baixo.
 * @param high_lux O valor de Lux para o limiar alto.
 * @return true se a configuração for bem-sucedida e os valores de Lux forem válidos, false caso contrário.
 */
bool opt4001_set_lux_thresholds(float low_lux, float high_lux) {
    opt4001_threshold_value_t low_thresh_val = opt4001_lux_to_threshold_reg_values(low_lux);
    if (!low_thresh_val.valid) {
        printf("OPT4001_set_lux_thresholds: Cálculo do limiar baixo (%f Lux) falhou.\n", low_lux);
        return false;
    }

    opt4001_threshold_value_t high_thresh_val = opt4001_lux_to_threshold_reg_values(high_lux);
    if (!high_thresh_val.valid) {
        printf("OPT4001_set_lux_thresholds: Cálculo do limiar alto (%f Lux) falhou.\n", high_lux);
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
 * @brief Converte um valor de Lux desejado para os valores de Expoente e Resultado (Mantissa)
 * apropriados para os registradores de limiar (THRESHOLD_L/H_EXPONENT e THRESHOLD_L/H_RESULT).
 * @param desired_lux O valor de Lux que se deseja definir para o limiar.
 * @return Uma estrutura opt4001_threshold_value_t contendo o expoente, resultado e uma flag de validade.
 */
opt4001_threshold_value_t opt4001_lux_to_threshold_reg_values(float desired_lux) {
    opt4001_threshold_value_t threshold_val = {0}; // Inicializa todos os campos como 0
    threshold_val.valid = false; // Assume inválido até ser calculado com sucesso

    if (desired_lux <= 0) {
        printf("OPT4001_lux_to_threshold_reg_values: Lux desejado deve ser positivo.\n");
        return threshold_val;
    }

    float lux_factor;
    lux_factor = OPT4001_LUX_FACTOR_SOT5X3;
  

    // 1. Converta o Lux desejado para ADC_CODES_Desejado
    float adc_codes_float = desired_lux / lux_factor;

    // 2. Determina o Expoente do Limiar (EXPONENT_LIMIAR)
    // O Expoente vai de 0 a 8.
    // A Mantissa do limiar é de 12 bits (0 a 4095, ou 0xFFF).
    // Formula interna de ADC_CODES_TH = RES_LIMIAR << (8 + EXPONENT_LIMIAR)
    // Então, RES_LIMIAR = ADC_CODES_Desejado / (1 << (8 + EXPONENT_LIMIAR))
    // Precisamos encontrar EXPONENT_LIMIAR tal que RES_LIMIAR caiba em 12 bits.

    uint8_t exp_limiar = 0;
    uint32_t res_limiar_candidate; // Usar uint32_t para evitar overflow temporário

    // Iterar pelos possíveis expoentes (0 a 8) 
    for (exp_limiar = 0; exp_limiar <= 8; exp_limiar++) { 
        // Calcular o valor do deslocamento (2^(8+EXPONENT))
        uint32_t shift_value = 1 << (8 + exp_limiar);
        
        // Calcular a Mantissa_Candidata
        res_limiar_candidate = (uint32_t)(adc_codes_float / (float)shift_value);

        // Se a Mantissa_Candidata cabe em 12 bits (0 a 4095)
        if (res_limiar_candidate <= 0xFFF) { // 0xFFF é 4095
            threshold_val.exponent = exp_limiar;
            threshold_val.result = (uint16_t)res_limiar_candidate;
            threshold_val.valid = true;
            return threshold_val;
        }
    }
    
    // Se nenhum expoente encontrou uma Mantissa_Candidata válida dentro da faixa de 12 bits
    // Isso pode acontecer se o Lux desejado for muito alto ou muito baixo para ser representado
    // com a precisão de 12 bits para a Mantissa.
    printf("OPT4001_lux_to_threshold_reg_values: Não foi possível encontrar expoente/resultado válidos para Lux %.2f\n", desired_lux);
    return threshold_val;
}


/**
 * @brief Inicializa o sensor OPT4001 com o baud rate especificado.
 * Os outros parâmetros de hardware (porta I2C, pinos, endereço e variante)
 * são fixos de acordo com as definições em opt4001.h.
 * @param baudrate Taxa de baud do I2C.
 * @return true se a inicialização for bem-sucedida, false caso contrário.
 */
bool opt4001_init(uint baudrate) {
    i2c_init(_i2c_instance, baudrate);
    gpio_set_function(OPT4001_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OPT4001_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OPT4001_SDA_PIN);
    gpio_pull_up(OPT4001_SCL_PIN);

    // Pequeno atraso para o sensor inicializar após a energização
    sleep_ms(100);

    // Tentar ler o Device ID para verificar a comunicação (registrador 0x11)
    uint16_t device_id;
    if (!opt4001_read_register(OPT4001_REG_DEVICE_ID, &device_id)) {
        printf("OPT4001: Falha ao ler Device ID.\n");
        return false;
    }
    printf("OPT4001: Device ID lido: 0x%04X\n", device_id);

    // Configurar o sensor para o modo operacional padrão (ex: auto-range, contínuo)
    uint16_t config_reg_value = 0; // Inicia em power-down 

    // Se você quiser iniciar em um modo específico, por exemplo, Contínuo com Auto-Range:
    // config_reg_value = (0xC << 10) | (0xB << 6) | (0x3 << 4); // Auto-range, 800ms, Continuous

    if (!opt4001_write_register(OPT4001_REG_CONFIG, config_reg_value)) {
        printf("OPT4001: Falha ao configurar o modo inicial.\n");
        return false;
    }

    return true;
}

/**
 * @brief Lê um registrador de 16 bits do sensor via I2C.
 * @param reg_address Endereço do registrador a ser lido.
 * @param value Ponteiro para armazenar o valor lido.
 * @param ret armazenar o valor de retorno da chamada da função I²C.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool opt4001_read_register(uint8_t reg_address, uint16_t *value) {
    uint8_t rx_data[2]; //vetor com MSB E LSB 
    int ret = i2c_write_blocking(_i2c_instance, _i2c_addr, &reg_address, 1, true); //enviamos um byte (endereço do registrador)
    if (ret != 1) { //esperamos receber a confirmação que um byte foi escrito com sucesso. Caso contrário, retornamos false (printf util para depuração)
        // printf("OPT4001: Erro na escrita do endereço do registrador (%d)\n", ret);
        return false;
    }
    ret = i2c_read_blocking(_i2c_instance, _i2c_addr, rx_data, 2, false);
    if (ret != 2) {
        // printf("OPT4001: Erro na leitura dos dados do registrador (%d)\n", ret);
        return false;
    }
    *value = (rx_data[0] << 8) | rx_data[1];
    return true;
}

/**
 * @brief Escreve um valor de 16 bits em um registrador do sensor via I2C.
 * @param reg_address Endereço do registrador a ser escrito.
 * @param value Valor a ser escrito.
 * @param ret armazenar o valor de retorno da chamada da função I²C.
 * @return true se a escrita for bem-sucedida, false caso contrário.
 */
bool opt4001_write_register(uint8_t reg_address, uint16_t value) {
    uint8_t tx_data[3];
    tx_data[0] = reg_address;
    tx_data[1] = (uint8_t)(value >> 8);   // MSB
    tx_data[2] = (uint8_t)(value & 0xFF); // LSB
    int ret = i2c_write_blocking(_i2c_instance, _i2c_addr, tx_data, 3, false);
    if (ret != 3) {
        // printf("OPT4001: Erro na escrita do registrador (%d)\n", ret);
        return false;
    }
    return true;
}

/**
 * @brief Define o tempo de conversão do sensor.
 * @param conversion_time_reg_value Valor do registrador para o tempo de conversão (0 a 11).
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_conversion_time(uint8_t conversion_time_reg_value) {
    uint16_t current_config;
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &current_config)) {
        return false;
    }
    current_config = (current_config & ~((0xF) << 6)) | (conversion_time_reg_value << 6); // Limpa e define os bits de CONVERSION_TIME (bits 9-6)
    return opt4001_write_register(OPT4001_REG_CONFIG, current_config);
}

/**
 * @brief Define o modo de operação do sensor.
 * @param mode Modo de operação (0: Power-down, 1: Forced auto-range One-shot, 2: One-shot, 3: Continuous).
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_operating_mode(uint8_t mode) {
    uint16_t current_config;
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &current_config)) {
        return false;
    }
    current_config = (current_config & ~((0x3) << 4)) | (mode << 4); // Limpa e define os bits de OPERATING_MODE (bits 5-4)
    return opt4001_write_register(OPT4001_REG_CONFIG, current_config);
}

/**
 * @brief Define a faixa de escala do sensor.
 * @param range_reg_value Valor do registrador para a faixa (0 a 8 para manual, 0xC para auto-range).
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_range(uint8_t range_reg_value) {
    uint16_t current_config;
    if (!opt4001_read_register(OPT4001_REG_CONFIG, &current_config)) {
        return false;
    }
    current_config = (current_config & ~((0xF) << 10)) | (range_reg_value << 10); // Limpa e define os bits de RANGE (bits 13-10)
    return opt4001_write_register(OPT4001_REG_CONFIG, current_config);
}

/**
 * @brief Espera a conversão do sensor ser concluída.
 * @param timeout_ms Tempo limite em milissegundos.
 * @return true se a conversão for concluída dentro do tempo limite, false caso contrário.
 */
bool opt4001_wait_for_conversion_complete(uint32_t timeout_ms) {
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint16_t flags_reg;

    while (to_ms_since_boot(get_absolute_time()) - start_time < timeout_ms) {
        if (!opt4001_read_register(OPT4001_REG_INTERRUPT_FLAGS, &flags_reg)) {
            return false;
        }
        if ((flags_reg >> 2) & 0x01) { // CONVERSION_READY_FLAG é o bit 2
            return true;
        }
        sleep_ms(1); // Pequeno atraso para evitar polling excessivo
    }
    return false;
}

/**
 * @brief Calcula o valor de lux a partir dos ADC_CODES.
 * @param adc_codes O valor linearizado de ADC_CODES.
 * @param variant A variante do sensor (PicoStar ou SOT5X3).
 * @return O valor em Lux.
 */
float opt4001_calculate_lux(uint32_t adc_codes) {
        return (float)adc_codes * OPT4001_LUX_FACTOR_SOT5X3;
}


/**
 * @brief Lê todos os dados de resultado e status do sensor.
 * @param data Ponteiro para a estrutura opt4001_data_t para armazenar os dados lidos.
 * @return true se a leitura for bem-sucedida, false caso contrário.
 */
bool opt4001_get_data(opt4001_data_t *data) {
    uint16_t reg_val_00, reg_val_01, reg_val_0C;

    // Lê os registradores de resultado e status
    if (!opt4001_read_register(OPT4001_REG_RESULT_MSB, &reg_val_00)) return false;
    if (!opt4001_read_register(OPT4001_REG_RESULT_LSB, &reg_val_01)) return false;
    if (!opt4001_read_register(OPT4001_REG_INTERRUPT_FLAGS, &reg_val_0C)) return false;

    data->raw_result_msb = reg_val_00;
    data->raw_result_lsb = reg_val_01;

    // Extrai EXPONENT (bits 15-12 de 0x00)
    data->exponent = (reg_val_00 >> 12) & 0x0F;

    // Extrai MANTISSA: RESULT_MSB (bits 11-0 de 0x00) e RESULT_LSB (bits 15-8 de 0x01)
    data->mantissa = ((reg_val_00 & 0x0FFF) << 8) | ((reg_val_01 >> 8) & 0xFF);

    // Calcula ADC_CODES
    data->adc_codes = data->mantissa << data->exponent;

    // Calcula Lux
    data->lux = opt4001_calculate_lux(data->adc_codes);

    // Extrai COUNTER (bits 7-4 de 0x01)
    data->counter = (reg_val_01 >> 4) & 0x0F; 

    // Extrai CRC (bits 3-0 de 0x01)
    data->crc = reg_val_01 & 0x0F; 

    // Extrai flags (de 0x0C)
    data->overload_flag = (reg_val_0C >> 3) & 0x01; 
    data->conversion_ready = (reg_val_0C >> 2) & 0x01; 
    data->flag_h = (reg_val_0C >> 1) & 0x01; 
    data->flag_l = reg_val_0C & 0x01; 

    // O datasheet (seção 8.6.1.13) indica que CONVERSION_READY_FLAG é limpa
    // quando o registrador 0x0C é lido ou escrito com qualquer valor não-zero.
    // A simples leitura acima já deve limpar, mas se quiser garantir a limpeza explícita
    // de todas as flags após a leitura (seção 8.4.2), você pode adicionar:
    // opt4001_write_register(OPT4001_REG_INTERRUPT_FLAGS, 0x0000); // Limpa todas as flags
    // Isso pode ser uma decisão de design dependendo de como você quer gerenciar as flags.

    return true;
}

/**
 * @brief Define os limites de limiar (thresholds) para o sensor.
 * @param low_exp Expoente do limiar baixo.
 * @param low_res Resultado do limiar baixo.
 * @param high_exp Expoente do limiar alto.
 * @param high_res Resultado do limiar alto.
 * @return true se a configuração for bem-sucedida, false caso contrário.
 */
bool opt4001_set_thresholds(uint8_t low_exp, uint16_t low_res, uint8_t high_exp, uint16_t high_res) {
    uint16_t low_threshold_reg = (low_exp << 12) | (low_res & 0x0FFF); 
    uint16_t high_threshold_reg = (high_exp << 12) | (high_res & 0x0FFF);

    if (!opt4001_write_register(OPT4001_REG_THRESHOLD_L_EXPONENT, low_threshold_reg)) {
        return false;
    }
    if (!opt4001_write_register(OPT4001_REG_THRESHOLD_H_EXPONENT, high_threshold_reg)) {
        return false;
    }
    return true;
}

/**
 * @brief Obtém o estado das flags de interrupção.
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