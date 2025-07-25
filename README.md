
-----

# Biblioteca OPT4001 para Raspberry Pi Pico W

Esta é uma biblioteca em C para o sensor de luz ambiente digital Texas Instruments OPT4001, desenvolvida para ser utilizada com a placa Raspberry Pi Pico W e seu SDK. A biblioteca permite configurar o sensor, ler dados de iluminância (Lux), e utilizar recursos avançados como detecção de limiares (thresholds) e interrupções.

## Índice

1.  [Visão Geral](#visão-geral)
2.  [Funcionalidades](#funcionalidades)
3.  [Hardware e Conexões](#hardware-e-conexões)
4.  [Estrutura do Projeto](#estrutura-do-projeto)
5.  [Configuração do Ambiente de Desenvolvimento](#configuração-do-ambiente-de-desenvolvimento)
6.  [Como Usar a Biblioteca](#como-usar-a-biblioteca)
    * [Inicialização](#inicialização)
    * [Configuração Básica](#configuração-básica)
    * [Leitura de Dados](#leitura-de-dados)
    * [Configuração de Limiares (Thresholds)](#configuração-de-limiares-thresholds)
    * [Uso de Interrupções](#uso-de-interrupções)
    * [Funções de Baixo Nível](#funções-de-baixo-nível)
7.  [Suite de Testes](#suite-de-testes)

-----

## Visão Geral

O OPT4001 é um sensor de luz para digital (luxímetro de chip único) com alta precisão e velocidade, que se comunica via interface I²C. Ele é otimizado para corresponder à resposta do olho humano à luz visível, com excelente rejeição de infravermelho (IV). Esta biblioteca abstrai a complexidade da comunicação I²C e da interpretação dos dados brutos do sensor, fornecendo uma API simples e fácil de usar.

## Funcionalidades

  * Inicialização da comunicação I²C com o sensor.
  * Modos de operação configuráveis: Power-down, One-shot (disparo único) e Contínuo.
  * Tempos de conversão selecionáveis.
  * Seleção automática de faixa de luz (Auto-Range).
  * Leitura de dados de iluminância em Lux.
  * Extração de `EXPONENT`, `MANTISSA`, `ADC_CODES`, `COUNTER` e `CRC` dos dados brutos.
  * Detecção de sobrecarga do sensor (`overload_flag`).
  * Configuração e uso de limiares de luz (thresholds) em valores de Lux diretamente.
  * Flags de status para limites alto (`FLAG_H`) e baixo (`FLAG_L`), e conversão concluída (`CONVERSION_READY`).
  * Suporte a interrupções via pino `INT` (somente variante SOT-5X3) para operação baseada em eventos e economia de energia.
  * Acesso direto a funções de leitura e escrita de registradores para depuração avançada.

## Hardware e Conexões

Esta biblioteca é configurada para a variante **OPT4001 SOT-5X3** e as seguintes conexões padrão na Raspberry Pi Pico W:

  * **Sensor I²C Address:** `0x45` (indicando que o pino `ADDR` do OPT4001 está conectado ao VDD ou SCL na PCB).
  * **Pino SDA (Pico W):** `GPIO 4`
  * **Pino SCL (Pico W):** `GPIO 5`
  * **Pino INT (Pico W):** `GPIO 13` (Conectado ao pino `INT` do OPT4001).

**Importante:** Certifique-se de que as linhas I²C (SDA e SCL) possuam **resistores de pull-up externos** (tipicamente 4.7kΩ ou 10kΩ) conectados à linha de alimentação VDD (3.3V). O pino `INT` do OPT4001 é *open-drain* e também requer um resistor de pull-up externo para funcionar corretamente.

## Estrutura do Projeto

O projeto segue uma estrutura padrão do SDK da Raspberry Pi Pico:

```
.
├── CMakeLists.txt              # Configuração de compilação do CMake
├── opt4001.h                   # Arquivo de cabeçalho da biblioteca OPT4001
├── opt4001.c                   # Implementação da biblioteca OPT4001
├── luminosidade_3.c            # Exemplo de aplicação principal (para uso final)
├── test_opt4001.c              # Suite de testes para a biblioteca
└── README.md                   # Este arquivo
```

## Como Usar a Biblioteca

### Inicialização

A primeira função a ser chamada para inicializar o sensor.

```c
#include "opt4001.h"

// ... em main()
if (!opt4001_init(100 * 1000)) { // Inicializa I2C a 100 kHz
    printf("ERRO: Falha na inicializacao do OPT4001!\n");
    while(true); // Trava se falhar
}
printf("Sensor OPT4001 inicializado com sucesso.\n");
```

### Configuração Básica

Defina o modo de operação, faixa de luz e tempo de conversão.

```c
// ... após opt4001_init()
opt4001_set_operating_mode(OPT4001_MODE_CONTINUOUS); // Modo contínuo
opt4001_set_range(OPT4001_RANGE_AUTO);               // Auto-ajuste de faixa
opt4001_set_conversion_time(OPT4001_CONV_TIME_400MS); // Tempo de conversão de 400ms
printf("Sensor configurado para medicões contínuas e auto-range.\n");
```

### Leitura de Dados

Obtenha todos os dados processados do sensor.

```c
opt4001_data_t sensor_data;
if (opt4001_get_data(&sensor_data)) {
    printf("Lux: %.2f\n", sensor_data.lux);
    printf("Overload: %d, FlagH: %d, FlagL: %d\n", sensor_data.overload_flag, sensor_data.flag_h, sensor_data.flag_l);
} else {
    printf("Falha ao ler dados do sensor.\n");
}
```

### Configuração de Limiares (Thresholds)

Defina os limites de luz que acionarão as flags `FLAG_H` e `FLAG_L`.

```c
float low_threshold_lux = 10.0f;  // Luz muito baixa
float high_threshold_lux = 1000.0f; // Luz muito alta
if (!opt4001_set_lux_thresholds(low_threshold_lux, high_threshold_lux)) {
    printf("ERRO: Falha ao configurar os limiares de Lux.\n");
} else {
    printf("Limiares setados para Baixo: %.2f Lux, Alto: %.2f Lux.\n", low_threshold_lux, high_threshold_lux);
}
```

### Uso de Interrupções

Configure o pino `INT` do sensor e a interrupção GPIO na Pico W para acordar o microcontrolador em eventos de luz.

```c
#include "hardware/gpio.h"
#include "hardware/sync.h" // Para __wfi()

// ... (definição de OPT4001_INT_GPIO_PIN e opt4001_interrupted, opt4001_int_gpio_callback)

// ... em main()
// Configurar o LATCH para Transparent Hysteresis (recomendado para interrupções dinâmicas)
uint16_t config_reg_val;
if (opt4001_read_register(OPT4001_REG_CONFIG, &config_reg_val)) {
    config_reg_val &= ~(1 << 3); // Limpa o bit 3 (LATCH)
    opt4001_write_register(OPT4001_REG_CONFIG, config_reg_val);
}

// Configura o pino INT do sensor (sensor-side)
opt4001_config_interrupt_pin(OPT4001_INT_POLARITY_ACTIVE_LOW, OPT4001_FAULT_COUNT_2_FAULTS);

// Configura o pino GPIO da Pico W para a interrupção
gpio_set_dir(OPT4001_INT_GPIO_PIN, GPIO_IN);
gpio_pull_up(OPT4001_INT_GPIO_PIN); // Importante para open-drain
gpio_set_irq_enabled_with_callback(OPT4001_INT_GPIO_PIN, GPIO_IRQ_EDGE_FALL, true, &opt4001_int_gpio_callback);

// Loop principal com economia de energia
while (true) {
    __wfi(); // Espera por uma interrupção

    if (opt4001_interrupted) {
        opt4001_interrupted = false; // Limpa a flag

        opt4001_data_t event_data;
        if (opt4001_get_data(&event_data)) {
            printf("Interrupcao! Lux: %.2f, FlagH: %d, FlagL: %d\n", event_data.lux, event_data.flag_h, event_data.flag_l);
            // Sua lógica de resposta à interrupção aqui
        }
    }
}
```

### Funções de Baixo Nível

Estas funções são expostas para depuração e testes avançados.

  * **`bool opt4001_read_register(uint8_t reg_address, uint16_t *value)`**: Lê o valor de 16 bits de um registrador específico.
  * **`bool opt4001_write_register(uint8_t reg_address, uint16_t value)`**: Escreve um valor de 16 bits em um registrador específico.

## Suite de Testes

O projeto inclui um arquivo `test_opt4001.c` com uma suite de testes que cobre a inicialização, configuração de modos, leitura de dados, limiares e interrupções.
-----


-----
