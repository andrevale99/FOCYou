# Resumo das bibliotecas do firmware

Firmware de controle de motor BLDC para o **STM32F411 (Black Pill)**, compilado em bare-metal (CMSIS + FreeRTOS, sem HAL).
Hardware atual: módulo **SimpleFOC Mini (DRV8313)**, com controle de velocidade por **six-step**. O objetivo final é o FOC.

> Não estão neste resumo `Drivers/` (CMSIS da ARM/ST) e `FreeRTOS/` (kernel), que são código de terceiros.

## Organização

```
firmware/
├── lib/                  # bibliotecas de aplicação (controle, potência, interface)
│   ├── drv8313/          # driver do CI DRV8313 (SimpleFOC Mini)
│   ├── inverter/         # inversor trifásico com PWM complementar no TIM1
│   ├── sixstep/          # lógica de comutação six-step (trapezoidal 120°)
│   ├── foc/              # transformadas de Clarke e Park
│   ├── pid/              # controlador PID com anti-windup
│   ├── as5600/           # encoder magnético AS5600 (I2C)
│   └── lcd16x2/          # display HD44780 em modo de 4 bits
├── src/
│   ├── drivers/          # camada de acesso a registradores (HAL própria)
│   │   ├── driver_err.h  # códigos de erro comuns
│   │   ├── gpio/  rcc/  uart/  i2c/  lcd/
│   ├── main.c            # aplicação e tasks do FreeRTOS
│   └── FreeRTOSConfig.h, startup, syscalls, system_stm32f4xx.c
├── makefile              # arm-none-eabi-gcc; alvos all, compile, flash, clean
└── STM32F411XE_FLASH.ld
```

O `makefile` adiciona automaticamente ao include path todo subdiretório de `lib/` e `src/drivers/` que tenha `.c`/`.h`, e compila todos os `.c` encontrados. Uma biblioteca nova só precisa de uma pasta própria. O gravador padrão é o `st-flash`; com `FLASHER=openocd` usa o OpenOCD.

As bibliotecas seguem o mesmo padrão:
- um **handle** (`xxx_t`) preenchido pelo `init` a partir de uma **struct de configuração**;
- um **enum de erro** próprio (0 = sucesso, negativos = erro);
- validação de ponteiros e de inicialização em todas as funções públicas;
- duty cycle **normalizado (0.0 a 1.0)**, com saturação de valores fora da faixa e de NaN.

---

## Bibliotecas de aplicação (`lib/`)

### `drv8313`: driver do DRV8313 (SimpleFOC Mini)

**Uso:** é o driver de potência atual do projeto.

O DRV8313 tem três meias pontes com lógica interna de dead-time. Cada entrada `INx` recebe **um PWM por fase**, sem pares complementares. No SimpleFOC Mini, os três `ENx` estão ligados a **um único pino EN**.

| Item | Detalhe |
|---|---|
| Timer | Qualquer um de TIM1 a TIM5; IN1/IN2/IN3 = CH1/CH2/CH3 do mesmo timer |
| PWM | Modo PWM1 com preload; edge- ou center-aligned (`center_aligned`) |
| Base de tempo | PSC/ARR calculados a partir de `timer_clock_hz` e `pwm_frequency_hz` |
| TIM1 | A biblioteca seta `MOE` automaticamente |
| Pinos de controle | `en` (obrigatório); `n_fault` (entrada com pull-up), `n_reset` e `n_sleep` são opcionais (`port = NULL` quando não usados) |
| `delay_ms` | Callback obrigatório só quando `n_reset` ou `n_sleep` é usado |

**API**

| Função | Descrição |
|---|---|
| `drv8313_init` | Configura pinos e timer. Termina com EN baixo, duty 0% e o PWM rodando |
| `drv8313_deinit` | Desabilita o driver, para o timer e desliga o clock dele |
| `drv8313_enable` / `drv8313_disable` | Controlam o pino EN. `enable` falha se o driver estiver em sleep ou com nFAULT ativo |
| `drv8313_sleep` / `drv8313_wake` | Controlam nSLEEP; `wake` espera `DRV8313_WAKE_DELAY_MS` |
| `drv8313_reset` | Gera um pulso em nRESET (e desabilita as saídas antes dele) |
| `drv8313_get_fault` | Lê nFAULT (ativo em nível baixo) |
| `drv8313_set_duty_cycle` | Atualiza os três duty cycles (`drv8313_duty_t {a, b, c}`); o valor vale no próximo update event |
| `drv8313_is_enabled` | Informa o estado do EN |

Erros: `DRV8313_OK`, `ERR_INVALID_ARG`, `ERR_NOT_INITIALIZED`, `ERR_INVALID_TIMER`, `ERR_OUT_OF_RANGE`, `ERR_NO_DELAY`, `ERR_GPIO`, `ERR_NOT_CONFIGURED`, `ERR_FAULT`, `ERR_ASLEEP`.

---

### `inverter`: inversor trifásico com o TIM1 (timer advanced)

**Uso:** é a biblioteca usada hoje no `main.c`. Serve para pontes com **6 entradas** (high/low separados), que não é o caso do DRV8313.

Gera três pares de PWM complementares (CHx/CHxN) com dead-time por hardware, proteção por entrada de break (BKIN), dispara o ADC injetado pelo CH4 e chama callbacks a partir das ISRs.

| Grupo | Funções |
|---|---|
| Ciclo de vida | `inverter_init`, `inverter_deinit`, `inverter_start` (CEN), `inverter_stop` |
| Saídas (MOE) | `inverter_set_state`, `inverter_get_state` |
| Frequência e PWM | `inverter_set_frequency` (recalcula PSC/ARR), `inverter_get_frequency`, `inverter_set_alignment`, `inverter_set_repetition_counter` |
| Dead-time | `inverter_set_deadtime` (valor bruto do DTG), `inverter_set_deadtime_ns` (escolhe a codificação do DTG) |
| Duty | `inverter_set_duty_cycle` (contagens, 0..ARR), `inverter_set_duty_cycle_normalized` (0.0 a 1.0), `inverter_get_duty_cycle` |
| Proteção | `inverter_config_break`, `inverter_get_fault`, `inverter_clear_fault`, `inverter_set_idle_state` (OISx/OSSR/OSSI), `inverter_lock_config` (LOCK do BDTR, irreversível até reset) |
| ADC e interrupções | `inverter_config_adc_trigger` (CCR4 → JEXTSEL = TIM1_CC4), `inverter_set_callbacks`, `inverter_enable_update_irq`, `inverter_enable_break_irq`, `inverter_irq_update_handler`, `inverter_irq_break_handler` |
| Controle por fase | `inverter_set_channel_enable` (CCxE/CCxNE individuais), `inverter_set_polarity`, `inverter_set_output_mode`, `inverter_force_output` |

Observações:
- a frequência é f = fclk / (2·(PSC+1)·ARR) em center-aligned e f = fclk / ((PSC+1)·(ARR+1)) em edge-aligned;
- `inverter_set_frequency` não reescala os duty cycles; eles precisam ser reaplicados depois;
- o handler de break desliga o BIE para não disparar de novo. Depois de `inverter_clear_fault`, é preciso reabilitar o BIE.

---

### `sixstep`: comutação six-step

**Uso:** faz a comutação do controle de velocidade atual.

É lógica pura, sem acesso a hardware: converte o setor (0 a 5) e um comando de tensão `u` nos três duty cycles. A aplicação lê a posição (Hall ou malha aberta) e envia o resultado ao driver de potência (`drv8313` ou `inverter`).

| Setor | +  | −  | Desligada |
|---|---|---|---|
| 0 | A | B | C |
| 1 | A | C | B |
| 2 | B | C | A |
| 3 | B | A | C |
| 4 | C | A | B |
| 5 | C | B | A |

O vetor de tensão avança 60° a cada setor.

- **Duty:** a fase + recebe `0.5 + u/2`, a fase − recebe `0.5 − u/2` e a fase desligada fica em `0.5`. No SimpleFOC Mini os três EN estão ligados juntos, então a fase desligada não pode ficar em alta impedância e fica no ponto médio (a mesma técnica do modo `Trapezoid_120` da biblioteca SimpleFOC).
- **Sinal de `u`:** o comando vai de −1.0 a 1.0, com saturação e NaN tratado como 0. Um `u` negativo troca as fases + e −, o que inverte o torque. Assim a saída do `pid` (com `saturation = 1.0`) pode ir direto para `u`.
- **`sixstep_output_t`:** contém `a`, `b`, `c` e `floating`, a fase desligada no setor. Esse campo serve para quando houver ENs independentes ou para o `inverter` (`inverter_set_channel_enable`).

**API**

| Função | Descrição |
|---|---|
| `sixstep_commutate(sector, u, &out)` | Calcula os duty cycles do setor |
| `sixstep_next_sector(sector, dir)` | Retorna o próximo setor (`SIXSTEP_DIR_FORWARD` ou `SIXSTEP_DIR_REVERSE`), para malha aberta |
| `sixstep_init(&ss, hall_by_sector, lead)` | Monta a tabela que converte o código Hall no setor a aplicar |
| `sixstep_hall_to_sector(&ss, hall, &sector)` | Converte o código Hall (bit0 = A, bit1 = B, bit2 = C) no setor |

**Calibração dos Hall:** com o motor parado, aplique cada setor com um `u` baixo, espere o rotor alinhar e anote o código Hall lido. Os 6 códigos formam o `hall_by_sector`. O `lead` (avanço do vetor em setores) é ajustado na prática: teste 1 e 2 e fique com o que der menor corrente, ou maior velocidade, para o mesmo `u`.

Erros: `SIXSTEP_OK`, `ERR_INVALID_ARG`, `ERR_NOT_INITIALIZED` e `ERR_INVALID_HALL` (código 000, 111 ou fora da tabela; nesse caso as saídas devem ser desligadas).

---

### `foc`: transformadas de referencial

Funções matemáticas puras, sem estado e sem acesso a hardware.

| Tipo | Campos |
|---|---|
| `foc_uvw_coord_t` | `u, v, w` (trifásico) |
| `foc_ab_coord_t` | `alpha, beta` (estacionário) |
| `foc_dq_coord_t` | `d, q` (síncrono) |

| Função | Fórmula |
|---|---|
| `foc_clarke_transform` | α = u; β = (v − w)/√3 (invariante em amplitude, supõe u + v + w = 0) |
| `foc_inverse_clarke_transform` | u = α; v = −α/2 + (√3/2)β; w = −α/2 − (√3/2)β |
| `foc_park_transform(theta_e, ...)` | d = α·cosθ + β·sinθ; q = −α·sinθ + β·cosθ |
| `foc_inverse_park_transform(theta_e, ...)` | α = d·cosθ − q·sinθ; β = d·sinθ + q·cosθ |

Ainda faltam a modulação (SPWM/SVPWM), a estimativa de ângulo e as malhas de corrente.

---

### `pid`: controlador PID

Controlador PID de tempo discreto, em `float`.

- **struct `pid`**: ganhos `kp`, `ki`, `kd`; `saturation` (limite simétrico da saída e do integrador); estado interno `P`, `I`, `D` e `output`.
- **`pid_control(pid, setpoint, input, dt)`**:
  - derivada calculada sobre a medida, o que evita o *derivative kick* quando o setpoint muda;
  - na primeira amostra o termo D é zero;
  - anti-windup por integração condicional: o integrador não avança quando a saída satura no mesmo sentido do erro, e ele também é limitado a ±`saturation`.
- **`pid_clear`**: zera o estado e mantém os ganhos e a saturação.

O resultado fica em `pid.output`, na faixa [−saturation, +saturation]. No six-step, com `saturation = 1.0`, essa saída vai direto para o `u` de `sixstep_commutate`.

---

### `as5600`: encoder magnético AS5600

**Uso:** leitura da posição e da velocidade do rotor, um dos blocos que faltavam para o six-step e o FOC.

Sensor Hall de 12 bits (4096 contagens por volta), endereço I2C fixo **0x36**. Usa o driver `i2c`: a aplicação inicializa o barramento e passa o ponteiro do `i2c_t` na configuração, o que permite compartilhar o barramento. O sentido de contagem é definido pelo pino **DIR** do sensor (GND = cresce no sentido horário, VDD = anti-horário).

| Item | Detalhe |
|---|---|
| Configuração | `i2c` (barramento já inicializado) e `delay_ms` (obrigatório só para os comandos de OTP) |
| `init` | Só verifica o ACK em 0x36; não escreve nenhum registrador |
| Erro de I2C | Volta como `AS5600_ERR_I2C`; o código do driver fica em `as5600_get_i2c_error` |

**API**

| Grupo | Funções |
|---|---|
| Ciclo de vida | `as5600_init`, `as5600_get_i2c_error` |
| Ângulo | `as5600_read_raw_angle` (sem escala, indicado para controle de motor), `as5600_read_angle` (escala ZPOS..MPOS/MANG) |
| Diagnóstico | `as5600_read_status` (MD/ML/MH), `as5600_check_magnet`, `as5600_read_agc`, `as5600_read_magnitude` |
| Configuração (RAM) | `as5600_get_conf` / `as5600_set_conf` (PM, HYST, OUTS, PWMF, SF, FTH, WD), `as5600_get/set_zpos`, `_mpos`, `_mang` |
| OTP (irreversível) | `as5600_get_burn_count` (ZMCO), `as5600_burn_angle` (ZPOS/MPOS, até 3 vezes), `as5600_burn_setting` (MANG/CONF, uma vez) |
| Posição e velocidade | `as5600_update(h, dt_s)`, `as5600_reset_tracking`, `as5600_get_angle_rad`, `as5600_get_position_rad`, `as5600_get_turns`, `as5600_get_velocity_rad_s` |
| Conversões | `as5600_counts_to_rad`, `as5600_counts_to_deg` (`static inline`) |

Observações:
- `as5600_update` lê o RAW ANGLE, conta as voltas pelo menor caminho entre duas leituras e calcula a velocidade sem filtro. O rotor precisa girar menos de meia volta entre chamadas (a 1 kHz, até 30.000 rpm). A resolução da velocidade a 1 kHz é de 1,53 rad/s, então convém filtrar na aplicação.
- Os comandos de OTP conferem as pré-condições do datasheet (MD = 1 e ZMCO < 3 para o BURN_ANGLE; ZMCO = 0 para o BURN_SETTING), recarregam o OTP (0x01, 0x11, 0x10) e comparam com o valor escrito. Em 3,3 V, a gravação exige alimentação de 3,3 V a 3,5 V e 10 µF em VDD3V3.
- Para o FOC, o ângulo elétrico é `pares_de_polos · ângulo_mecânico − offset`; o offset vem do alinhamento do rotor e ainda não está implementado.

Erros: `AS5600_OK`, `ERR_INVALID_ARG`, `ERR_NOT_INITIALIZED`, `ERR_I2C`, `ERR_NOT_FOUND`, `ERR_NO_MAGNET`, `ERR_MAGNET_TOO_WEAK`, `ERR_MAGNET_TOO_STRONG`, `ERR_OUT_OF_RANGE`, `ERR_BURN_LIMIT`, `ERR_NO_DELAY`, `ERR_VERIFY`.

---

### `lcd16x2`: display HD44780

Driver de LCD 16x2 em modo de 4 bits, independente de hardware. O acesso aos pinos é feito por callbacks.

- **`lcd16x2_handle_t`**: callbacks `write(uint8_t)` para D4 a D7, EN e RS, mais `delay_ms`.
- **API**: `lcd16x2_init_4bits(handle, init_func)`, `lcd16x2_send_cmd`, `lcd16x2_send_data`, `lcd16x2_write_string(handle, str, size)`.
- **Macros** de comando do HD44780: `CLEAR_DISPLAY`, `RETURN_HOME`, `SECOND_LINE`, `SET_DDRAM`, `DISPLAY_ON` etc.

No `main.c`, `delay_ms` usa `vTaskDelay`, então o LCD só pode ser inicializado com o scheduler rodando.

---

## Camada de drivers (`src/drivers/`)

É a HAL própria do projeto, que acessa os registradores via CMSIS. Todas as funções retornam `driver_err_t`.

### `driver_err.h`
Enum de erro único para todos os drivers, com faixas por módulo: 0 = OK; −1..−9 genéricos; −10..−19 GPIO; −20..−29 RCC; −30..−39 reservado para UART.

### `gpio/driver_gpio`
| Função | Registrador |
|---|---|
| `driver_gpio_enable_clock` | RCC->AHB1ENR (GPIOA a E, H) |
| `driver_gpio_set_mode` | MODER (entrada, saída, AF, analógico) |
| `driver_gpio_set_output_type` | OTYPER (push-pull ou open-drain) |
| `driver_gpio_set_speed` | OSPEEDR |
| `driver_gpio_set_pull` | PUPDR |
| `driver_gpio_set_alternate_function` | MODER + AFR |
| `driver_gpio_write_pin` | BSRR (atômico) |
| `driver_gpio_read_pin` | IDR (retorna o nível, não um erro) |
| `driver_gpio_toggle_pin` | ODR (read-modify-write, não atômico) |

### `rcc/rcc`
Configura o clock do sistema: `rcc_init(rcc_config_t*)` (fonte, PLL e prescalers AHB/APB1/APB2, e atualiza `SystemCoreClock`), `rcc_set_system_clock_source`, `rcc_configure_pll` (M, N, P, com f = src/M·N/P; usa o HSE como entrada), `rcc_set_apbx_divider`, `rcc_set_ahb_divider` e `rcc_get_clock_status`.
O `main.c` ainda não usa esse driver; o sistema roda no **HSI de 16 MHz** (`HSE_VALUE` = 25 MHz no makefile).

### `uart/uart`
UART por polling (USART1, USART2 ou USART6), com 8N1, sem controle de fluxo e oversampling de 16: `uart_init(uart_config_t*)`, `uart_send_char`, `uart_send_string`. A transmissão bloqueia até terminar, então não deve ser chamada de ISR.

### `lcd/driver_lcd16x2.h`
Ligação entre a biblioteca `lcd16x2` e o hardware: pinos D4 a D7, EN e RS em **PA0 a PA5**. Tem `driver_lcd16x2_init()` e as funções `write_d4` ... `write_rs` (`static inline`), que são passadas como callbacks.

---

## Como as peças se conectam hoje (`main.c`)

- O **TIM1** roda via `inverter` a 10 kHz center-aligned, com pinos high em PA8 a PA10 e low em PB13 a PB15, dead-time de 500 ns e BKIN em PA6.
- O **CC4 do TIM1** dispara o ADC1 injetado (PA7, PB0, PB1). A `ADC_IRQHandler` lê as correntes e aplica o duty. A malha de controle ainda é um placeholder com 50% fixo.
- Há três tasks no FreeRTOS: `blink` (LED em PC13), `lcd` (estado, duty, frequência e correntes a cada 1 s) e `inv` (rearma o inversor depois de uma falha de break).

## Pontos de atenção para o six-step com o SimpleFOC Mini

1. **O `main.c` usa `inverter`, mas o hardware atual é o DRV8313.** Para o SimpleFOC Mini, a aplicação precisa trocar para `drv8313`: um timer com CH1 a CH3 nos IN1 a IN3, mais o EN. O break, o dead-time e os pinos complementares do `inverter` não se aplicam ao DRV8313, que já tem dead-time interno.
2. **Não dá para deixar uma fase flutuando no SimpleFOC Mini.** O six-step clássico deixa uma fase em alta impedância a cada passo, mas na placa os três EN estão ligados juntos. O `sixstep` contorna isso deixando a fase desligada em 50%. Para uma fase flutuando de verdade, seria preciso separar os pinos EN1, EN2 e EN3 no hardware. Isso também afeta o BEMF sensorless, que precisa da fase flutuante.
3. **Os blocos do six-step ainda não estão ligados no `main.c`**: a posição e a velocidade (`as5600`), a comutação (`sixstep`), a malha de velocidade (`pid`) e o driver de potência (`drv8313`) já existem. Com o `as5600`, o setor sai do ângulo elétrico (`pares_de_polos · ângulo − offset`, dividido em 6 faixas de 60°) em vez dos Hall.
4. **Os pinos do exemplo do `drv8313.h` conflitam com o `main.c`.** O exemplo usa TIM3 em PA6/PA7/PB0, que no `main.c` são o BKIN e entradas do ADC. Os pinos precisam ser revistos quando o `drv8313` for integrado. O LCD também ocupa PA0 a PA5.
5. **Clock:** o sistema roda no HSI de 16 MHz. Se o PLL for ligado via `rcc_init`, `timer_clock_hz` e as frequências de PWM e do ADC terão que ser recalculadas.
