# Biblioteca OPT4001 para Raspberry Pi Pico W

Esta é uma biblioteca em C para o sensor de luz ambiente digital Texas Instruments OPT4001, desenvolvida para ser utilizada com a placa Raspberry Pi Pico W e seu SDK. A biblioteca permite configurar o sensor, ler dados de iluminância (Lux), e utilizar recursos avançados como detecção de limiares (thresholds) e interrupções.

## Índice

1.  [Visão Geral](#visao-geral)
2.  [Funcionalidades](#funcionalidades)
3.  [Estrutura do Projeto](#estrutura-do-projeto)
4.  [Como Usar a Biblioteca](#como-usar-a-biblioteca)
      * [Inicialização](#inicialização)
      * [Configuração Básica](#configuração)
      * [Leitura de Dados](#leitura-de-dados)
      * [Configuração de Limiares (Thresholds)](#configuração-de-limiares-thresholds)
      * [Uso de Interrupções](#uso-de-interrupção)
      * [Funções de Baixo Nível](#funções-de-baixo-nível)

-----

## Visão Geral

O OPT4001 é um sensor de luz para digital (luxímetro de chip único) com alta precisão e velocidade, que se comunica via interface I²C. Ele é otimizado para corresponder à resposta do olho humano, com excelente rejeição de infravermelho (IV). Esta biblioteca abstrai a complexidade da comunicação I²C e da interpretação dos dados brutos do sensor, fornecendo uma API simples e fácil de usar.

## Funcionalidades

  * Inicialização da comunicação I²C com o sensor.
  * Modos de operação configuráveis: Power-down, One-shot (disparo único) e Contínuo.
  * Tempos de conversão selecionáveis.
  * Seleção automática de faixa de luz (Auto-Range).
  * Leitura de dados de iluminância em Lux.
  * Extração de `EXPONENT`, `MANTISSA`, `ADC_CODES`, `COUNTER`.
  * Detecção de sobrecarga do sensor (`overload_flag`).
  * Configuração e uso de limiares de luz (thresholds) em valores de Lux diretamente.
  * Flags de status para limites alto (`FLAG_H`) e baixo (`FLAG_L`), e conversão concluída (`CONVERSION_READY`).
  * Suporte a interrupções via pino `INT` (somente variante SOT-5X3) para operação baseada em eventos e economia de energia.
  * Acesso direto a funções de leitura e escrita de registradores para depuração avançada.

## Estrutura do Projeto

O projeto segue uma estrutura padrão do SDK da Raspberry Pi Pico. Para demonstrar o uso da biblioteca, foram criados diferentes arquivos de aplicação:

```
.
├── CMakeLists.txt                  # Configuração de compilação do CMake
├── opt4001.h                       # Arquivo de cabeçalho da biblioteca OPT4001
├── opt4001.c                       # Implementação da biblioteca OPT4001
├── luminosidade_3.c                # Exemplo de aplicação principal (leitura contínua)
├── interrupcao_histeris_teste.c    # Exemplo de uso de interrupção no modo Hysteresis
├── interrupcao_latched_teste.c     # Exemplo de uso de interrupção no modo Latched
├── test_opt4001.c                  # Suite de testes para a biblioteca
└── README.md                       # Este arquivo
```


## Como Usar a Biblioteca

### Inicialização

A primeira função a ser chamada para inicializar o sensor.

```c
#include "opt4001.h"

// ... em main()
if (!opt4001_init(I2C_PORT_USED, SDA_PIN_USED, SCL_PIN_USED, SENSOR_I2C_ADDRESS, SENSOR_VARIANT, 100 * 1000)) {
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

A biblioteca suporta interrupções de hardware, oferecendo dois modos de operação. Verifique os arquivos de exemplo para a implementação completa:

  * **`interrupcao_histeris_teste.c`**: Demonstra o modo de **histerese transparente** (`LATCH = 0`). Neste modo, as flags de alarme são atualizadas a cada conversão, e o pino de interrupção (`INT`) só muda de estado quando a medição cruza um dos limiares.
  * **`interrupcao_latched_teste.c`**: Demonstra o modo **latched** (`LATCH = 1`). Neste modo, uma vez que uma flag de alarme é ativada, ela permanece travada (`latched`) e o pino de interrupção fica no estado ativo, até que o registrador de flags seja limpo manualmente.

### Funções de Baixo Nível

Estas funções são expostas para depuração e testes avançados.

  * **`bool opt4001_read_register(uint8_t reg_address, uint16_t *value)`**: Lê o valor de 16 bits de um registrador específico.
  * **`bool opt4001_write_register(uint8_t reg_address, uint16_t value)`**: Escreve um valor de 16 bits em um registrador específico.
