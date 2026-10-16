# Firmware FOCYou

Firmware de controle de motor BLDC para o **STM32F411 (Black Pill)**, compilado em bare-metal (CMSIS + FreeRTOS, sem HAL).
Hardware atual: módulo **SimpleFOC Mini (DRV8313)** e encoder magnético **AS5600**. O controle de velocidade é feito por **six-step**: partida em rampa (malha aberta) e depois malha fechada com PID. O objetivo final é o FOC.

> Não estão neste resumo `Drivers/` (CMSIS da ARM/ST) e `FreeRTOS/` (kernel), que são código de terceiros.

## Organização

```
firmware/
├── include/
│   └── env.h                 # pinos, clocks, timers e parâmetros do motor (só para src/)
├── lib/                      # bibliotecas (sem acesso ao env.h)
│   ├── drv8313/              # driver do CI DRV8313 (SimpleFOC Mini)
│   ├── inverter/             # inversor trifásico com PWM complementar no TIM1
│   ├── sixstep/              # lógica de comutação six-step (trapezoidal 120°)
│   ├── foc/                  # transformadas de Clarke e Park
│   ├── pid/                  # controlador PID com anti-windup
│   ├── as5600/               # encoder magnético AS5600 (I2C)
│   ├── lcd16x2/              # display HD44780 em modo de 4 bits
│   └── system_foc/           # tasks do FreeRTOS que ligam as bibliotecas
│       ├── task_as5600/      # leitura do encoder a cada interrupção do timer
│       ├── task_drv8313/     # partida, comutação e controle de velocidade
│       └── task_queue.h      # reservado para filas entre tasks (vazio)
├── src/
│   ├── drivers/              # camada de acesso a registradores (HAL própria)
│   │   ├── driver_err.h      # códigos de erro comuns
│   │   ├── gpio/  rcc/  uart/  i2c/  gptim/  lcd/
│   ├── main.c                # inicialização, ISRs e criação das tasks
│   └── FreeRTOSConfig.h, startup, syscalls, system_stm32f4xx.c
├── pdfs/                     # documentação de referência
├── makefile                  # arm-none-eabi-gcc; alvos all, compile, flash, clean
└── STM32F411XE_FLASH.ld
```

O `makefile` adiciona automaticamente ao include path todo subdiretório de `lib/` (até dois níveis, como `lib/system_foc/task_as5600/`) e de `src/drivers/` que tenha `.c`/`.h`, além de `include/`, e compila todos os `.c` encontrados. Uma biblioteca nova só precisa de uma pasta própria. O gravador padrão é o `st-flash`; com `FLASHER=openocd` usa o OpenOCD.

As bibliotecas e drivers seguem o mesmo padrão:
- um **handle** (`xxx_t`) preenchido pelo `init` a partir de uma **struct de configuração**;
- um **enum de erro** próprio (0 = sucesso, negativos = erro);
- validação de ponteiros e de inicialização em todas as funções públicas;
- duty cycle **normalizado (0.0 a 1.0)**, com saturação de valores fora da faixa e de NaN.

As tasks de `lib/system_foc/` seguem outro padrão:
- um `task_xxx_create(...)` que recebe os periféricos já inicializados pelo `main.c` e uma **função de log** (`NULL` desativa os logs);
- um `TaskHandle_t task_xxx_handle` público, usado para os task notify;
- callbacks com a assinatura de `gptim_callback_t`, para serem registradas direto nos timers.

---

## Bibliotecas de aplicação (`lib/`)

### `drv8313`: driver do DRV8313 (SimpleFOC Mini)

**Uso:** é o driver de potência do projeto, controlado pela `task_drv8313`.

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

**Uso:** não é usada hoje no `main.c`. Serve para pontes com **6 entradas** (high/low separados), que não é o caso do DRV8313.

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
- o handler de break desliga o BIE para não disparar de novo. Depois de `inverter_clear_fault`, é preciso reabilitar o BIE;
- o vetor de update do TIM1 é compartilhado com o TIM10, que hoje é o timer do AS5600. Se o `inverter` voltar a ser usado, o `TIM1_UP_TIM10_IRQHandler` precisa chamar os dois handlers.

---

### `sixstep`: comutação six-step

**Uso:** é usada pela `task_drv8313` nas duas fases do controle: na rampa, o setor avança com `sixstep_next_sector`; na malha fechada, o setor sai do ângulo do encoder.

É lógica pura, sem acesso a hardware: converte o setor (0 a 5) e um comando de tensão `u` nos três duty cycles. A aplicação escolhe o setor (malha aberta, Hall ou encoder) e envia o resultado ao driver de potência (`drv8313` ou `inverter`).

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
- **Sinal de `u`:** o comando vai de −1.0 a 1.0, com saturação e NaN tratado como 0. Um `u` negativo troca as fases + e −, o que inverte o torque.
- **`sixstep_output_t`:** contém `a`, `b`, `c` e `floating`, a fase desligada no setor. Esse campo serve para quando houver ENs independentes ou para o `inverter` (`inverter_set_channel_enable`).

**API**

| Função | Descrição |
|---|---|
| `sixstep_commutate(sector, u, &out)` | Calcula os duty cycles do setor |
| `sixstep_next_sector(sector, dir)` | Retorna o próximo setor (`SIXSTEP_DIR_FORWARD` ou `SIXSTEP_DIR_REVERSE`), para malha aberta |
| `sixstep_init(&ss, hall_by_sector, lead)` | Monta a tabela que converte o código Hall no setor a aplicar |
| `sixstep_hall_to_sector(&ss, hall, &sector)` | Converte o código Hall (bit0 = A, bit1 = B, bit2 = C) no setor |

**Calibração dos Hall** (não usada hoje, o projeto usa o AS5600): com o motor parado, aplique cada setor com um `u` baixo, espere o rotor alinhar e anote o código Hall lido. Os 6 códigos formam o `hall_by_sector`. O `lead` (avanço do vetor em setores) é ajustado na prática: teste 1 e 2 e fique com o que der menor corrente, ou maior velocidade, para o mesmo `u`.

Erros: `SIXSTEP_OK`, `ERR_INVALID_ARG`, `ERR_NOT_INITIALIZED` e `ERR_INVALID_HALL` (código 000, 111 ou fora da tabela; nesse caso as saídas devem ser desligadas).

---

### `foc`: transformadas de referencial

Funções matemáticas puras, sem estado e sem acesso a hardware. Ainda não são usadas.

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

Ainda faltam a modulação (SPWM/SVPWM) e as malhas de corrente. O ângulo elétrico já existe: é o mesmo calibrado pela `task_drv8313`.

---

### `pid`: controlador PID

Controlador PID de tempo discreto, em `float`. É a malha de velocidade da `task_drv8313`.

- **struct `pid`**: ganhos `kp`, `ki`, `kd`; `saturation` (limite simétrico da saída e do integrador); estado interno `P`, `I`, `D` e `output`.
- **`pid_control(pid, setpoint, input, dt)`**:
  - derivada calculada sobre a medida, o que evita o *derivative kick* quando o setpoint muda;
  - na primeira amostra o termo D é zero;
  - anti-windup por integração condicional: o integrador não avança quando a saída satura no mesmo sentido do erro, e ele também é limitado a ±`saturation`.
- **`pid_clear`**: zera o estado e mantém os ganhos e a saturação.

O resultado fica em `pid.output`, na faixa [−saturation, +saturation]. Na `task_drv8313`, `saturation = u_max` e a saída é somada ao feedforward V/f: `u = V/f(setpoint) + output`.

---

### `as5600`: encoder magnético AS5600

**Uso:** posição e velocidade do rotor, lidas pela `task_as5600`.

Sensor Hall de 12 bits (4096 contagens por volta). Endereço I2C **0x36** (AS5600) ou **0x40** (padrão do AS5600L), escolhido em `as5600_config_t::address` (0 = 0x36). Usa o driver `i2c`: a aplicação inicializa o barramento e passa o ponteiro do `i2c_t` na configuração, o que permite compartilhar o barramento. O sentido de contagem é definido pelo pino **DIR** do sensor (GND = cresce no sentido horário, VDD = anti-horário); a `task_drv8313` descobre esse sentido sozinha na calibração.

| Item | Detalhe |
|---|---|
| Configuração | `i2c` (barramento já inicializado), `address` e `delay_ms` (obrigatório só para os comandos de OTP) |
| `init` | Só verifica o ACK no endereço; não escreve nenhum registrador |
| Erro de I2C | Volta como `AS5600_ERR_I2C`; o código do driver fica em `as5600_get_i2c_error` |

**API**

| Grupo | Funções |
|---|---|
| Ciclo de vida | `as5600_init`, `as5600_get_i2c_error` |
| Ângulo | `as5600_read_raw_angle` (sem escala, indicado para controle de motor), `as5600_read_angle` (escala ZPOS..MPOS/MANG) |
| Diagnóstico | `as5600_read_status` (MD/ML/MH), `as5600_check_magnet`, `as5600_read_agc`, `as5600_read_magnitude` |
| Configuração (RAM) | `as5600_get_conf` / `as5600_set_conf` (PM, HYST, OUTS, PWMF, SF, FTH, WD), `as5600_get/set_zpos`, `_mpos`, `_mang` |
| OTP (irreversível) | `as5600_get_burn_count` (ZMCO), `as5600_burn_angle` (ZPOS/MPOS, até 3 vezes), `as5600_burn_setting` (MANG/CONF, uma vez) |
| Posição e velocidade | `as5600_update(h, dt_s)`, `as5600_reset_tracking`, `as5600_get_raw`, `as5600_get_angle_rad`, `as5600_get_position_rad`, `as5600_get_turns`, `as5600_get_velocity_rad_s` |
| Conversões | `as5600_counts_to_rad`, `as5600_counts_to_deg` (`static inline`) |

Observações:
- `as5600_update` lê o RAW ANGLE, conta as voltas pelo menor caminho entre duas leituras e calcula a velocidade sem filtro. O rotor precisa girar menos de meia volta entre chamadas (a 1 kHz, até 30.000 rpm). A resolução da velocidade a 1 kHz é de 1,53 rad/s (cerca de 15 rpm), por isso a `task_drv8313` filtra a velocidade.
- Os comandos de OTP conferem as pré-condições do datasheet (MD = 1 e ZMCO < 3 para o BURN_ANGLE; ZMCO = 0 para o BURN_SETTING), recarregam o OTP (0x01, 0x11, 0x10) e comparam com o valor escrito. Em 3,3 V, a gravação exige alimentação de 3,3 V a 3,5 V e 10 µF em VDD3V3. Não foram validados com o AS5600L.

Erros: `AS5600_OK`, `ERR_INVALID_ARG`, `ERR_NOT_INITIALIZED`, `ERR_I2C`, `ERR_NOT_FOUND`, `ERR_NO_MAGNET`, `ERR_MAGNET_TOO_WEAK`, `ERR_MAGNET_TOO_STRONG`, `ERR_OUT_OF_RANGE`, `ERR_BURN_LIMIT`, `ERR_NO_DELAY`, `ERR_VERIFY`.

---

### `lcd16x2`: display HD44780

Driver de LCD 16x2 em modo de 4 bits, independente de hardware. O acesso aos pinos é feito por callbacks. Não é usado hoje no `main.c`.

- **`lcd16x2_handle_t`**: callbacks `write(uint8_t)` para D4 a D7, EN e RS, mais `delay_ms`.
- **API**: `lcd16x2_init_4bits(handle, init_func)`, `lcd16x2_send_cmd`, `lcd16x2_send_data`, `lcd16x2_write_string(handle, str, size)`.
- **Macros** de comando do HD44780: `CLEAR_DISPLAY`, `RETURN_HOME`, `SECOND_LINE`, `SET_DDRAM`, `DISPLAY_ON` etc.

Se `delay_ms` usar `vTaskDelay`, o LCD só pode ser inicializado com o scheduler rodando.

---

## Tasks do sistema (`lib/system_foc/`)

### `task_as5600`: leitura do encoder

Detecta o encoder e o lê a cada interrupção de um timer (`gptim`).

1. Espera o power-up do sensor (10 ms) e procura o encoder em **0x36** e depois em **0x40**. Sem nenhum dos dois, fica presa gerando o log de erro a cada 1 s.
2. Grava o CONF (sempre ligado, sem histerese, filtro `SF_2X`, `FTH_6LSB`) e confere o ímã (log do AGC ou do problema).
3. A cada notificação do timer, chama `as5600_update` e, se a leitura deu certo, chama o **callback de amostra** com `(raw, velocidade em rad/s, dt)`.

- **Notificações acumuladas:** se a task atrasar, as notificações acumuladas valem por uma leitura só, mas o `dt` passado ao callback conta todas.
- **Erros de I2C:** com `BUSY` ou `TIMEOUT`, chama `i2c_bus_recover`. O log mostra o primeiro erro e depois um resumo a cada 1 s.
- **Timer parado:** se passar 1 s sem notificação, a task avisa no log.

| Função | Descrição |
|---|---|
| `task_as5600_create(&i2c, period_us, log)` | Cria a task; `period_us` é o período do timer que a notifica |
| `task_as5600_notify_from_isr` | Callback do timer (`vTaskNotifyGiveFromISR`) |
| `task_as5600_set_sample_callback(cb)` | Registra o callback chamado após cada leitura, no contexto da task |
| `task_as5600_get_raw` / `get_velocity_rpm` / `is_ready` | Última leitura |

### `task_drv8313`: partida e controle de velocidade

O trabalho é dividido entre uma ISR e a task:
- **ISR de comutação** (`task_drv8313_commutate_from_isr`, registrada num timer `gptim`): escolhe o setor e aplica os duty cycles. Não usa a API do FreeRTOS, por isso pode ter prioridade acima do kernel, com menos atraso na troca de setor.
- **Task:** cuida da máquina de estados, da rampa, do PID, do nFAULT e dos comandos, todos recebidos por task notify.

```
IDLE --start--> ALIGN --2 · align_ms--> RAMP --rpm = ramp_end_rpm--> RUN
  ^                                                                   |
  +---------------------------- stop ---------------------------------+
(nFAULT ativo, erro do driver ou encoder sem amostras -> FAULT; clear_fault -> IDLE)
```

| Estado | O que acontece |
|---|---|
| ALIGN | Prende o rotor no setor 0 e depois no setor 1, `align_ms` cada. A diferença entre as duas leituras do encoder dá o zero elétrico, o sentido de contagem e confere os pares de polos (esperado: 4096 / (6 · pares de polos) contagens) |
| RAMP | Partida em malha aberta de `start_rpm` até `ramp_end_rpm` a `ramp_rpm_per_s`. A ISR avança um setor por interrupção; o período do timer é `10⁷ / (rpm · pares de polos)` µs. Tensão V/f: `u = u_offset + u_per_rpm · rpm`, limitada a `u_max` |
| RUN (malha fechada) | A ISR roda a `comm_rate_hz`, estima o ângulo elétrico entre amostras (`ângulo + velocidade · tempo`) e aplica o setor 90° elétricos à frente do rotor (−90° no sentido reverso). A task acorda a cada amostra do encoder e roda o PID |
| RUN (malha aberta) | Com `closed_loop = false`, sem encoder ou com calibração suspeita: continua como na rampa, seguindo o alvo |

**Malha fechada:**
- **PID logo após cada leitura:** o callback de amostra faz `xTaskNotify(EVT_SAMPLE)`, e a task do motor (prioridade 3, acima da do encoder) acorda logo depois de cada leitura para rodar o PID.
- **Tensão aplicada:** `u = V/f(setpoint) + saída do PID`, limitada a `u_max`.
- **Setpoint:** segue o alvo com a mesma rampa (`ramp_rpm_per_s`).
- **Velocidade medida:** passa por um filtro passa-baixa de `speed_filter_ms`.
- **Troca de malha:** na passagem da rampa para a malha fechada, o integrador começa com o valor que mantém a tensão atual, para não haver salto.
- **Falha do encoder:** sem amostra por `TASK_DRV8313_ENCODER_TIMEOUT_MS` (20 ms), a task vai para FAULT.

| Função | Descrição |
|---|---|
| `task_drv8313_create(&drv, &tim, &motor, log)` | Recebe o driver e o timer de comutação já inicializados (timer parado) |
| `task_drv8313_start` / `stop` / `clear_fault` | Comandos por task notify. `stop` desliga o EN (o motor para por inércia) |
| `task_drv8313_set_target_rpm` | Novo alvo, limitado entre `start_rpm` e `max_rpm` |
| `task_drv8313_set_pid_gains` | Troca os ganhos em funcionamento (sintonia) |
| `task_drv8313_set_direction` | Sentido de giro; só em IDLE |
| `task_drv8313_get_state` / `state_str` / `is_closed_loop` | Estado |
| `task_drv8313_get_rpm` / `get_speed_rpm` / `get_target_rpm` / `get_u` | Setpoint, velocidade medida (filtrada), alvo e tensão |
| `task_drv8313_encoder_sample` | Callback de amostra para a `task_as5600` |
| `task_drv8313_commutate_from_isr` | Callback do timer de comutação |

Os logs saem com o prefixo `[drv8313]`, entre eles a calibração, as mudanças de estado e, na malha fechada, uma linha por segundo com setpoint, velocidade medida e tensão. Os números são inteiros porque o `printf` da newlib-nano não formata `float`.

### `task_queue.h`

Reservado para as filas entre tasks. Está vazio.

---

## Camada de drivers (`src/drivers/`)

É a HAL própria do projeto, que acessa os registradores via CMSIS. Todas as funções retornam `driver_err_t`.

### `driver_err.h`
Enum de erro único para todos os drivers, com faixas por módulo: 0 = OK; −1..−9 genéricos; −10..−19 GPIO; −20..−29 RCC; −30..−39 reservado para UART; −40..−49 I2C; −50..−59 timers.

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

### `i2c/i2c`
I2C mestre por polling, com timeout em todas as esperas (I2C1, I2C2 ou I2C3; endereços de 7 bits; 100 kHz ou 400 kHz).

| Função | Descrição |
|---|---|
| `i2c_init(&h, &cfg)` / `i2c_deinit` | Pinos em open-drain com AF; calcula FREQ, CCR (arredondado para cima) e TRISE |
| `i2c_write` / `i2c_read` | Transação simples |
| `i2c_mem_write` / `i2c_mem_read` | Acesso a registrador (endereço de 1 ou 2 bytes), com repeated start na leitura |
| `i2c_is_device_ready` | Verifica o ACK no endereço, com várias tentativas |
| `i2c_bus_recover` | Até 9 pulsos em SCL e STOP manual por GPIO, depois reseta e reconfigura o periférico |

- **Leitura:** segue as sequências do RM0383 para 1, 2 e 3 ou mais bytes, com os trechos críticos protegidos contra interrupções.
- **Erros:** NACK, BERR e timeout geram STOP para liberar o barramento.
- **Hardware:** precisa de pull-up externo (2,2 kΩ a 4,7 kΩ). Com o PCLK1 de 16 MHz, o fast mode fica em cerca de 381 kHz.
- **Uso:** bloqueia durante a transação, então use só de tasks, não de ISR.

### `gptim/gptim`
Base de tempo periódica com os timers general purpose: gera uma interrupção de update a cada período e chama um callback.

| Timer | Barramento | ARR | Vetor |
|---|---|---|---|
| TIM2, TIM5 | APB1 | 32 bits | Próprio |
| TIM3, TIM4 | APB1 | 16 bits | Próprio |
| TIM9 / TIM10 / TIM11 | APB2 | 16 bits | Compartilhado com o TIM1 (break / update / trigger-commutation) |

| Função | Descrição |
|---|---|
| `gptim_init(&h, &cfg)` / `gptim_deinit` | Clock, PSC/ARR, interrupção de update e prioridade no NVIC. PSC é escolhido para dar o período exato quando possível (ex.: 16 MHz e 10 ms → PSC = 3, ARR = 39999) |
| `gptim_start` / `gptim_stop` / `gptim_is_running` | Controle do contador |
| `gptim_set_period_us` | Muda o período; com o timer rodando, vale no próximo update |
| `gptim_irq_handler(&h)` | Limpa o UIF e chama o callback; a aplicação chama de dentro do handler do vetor |

O driver não define os handlers de interrupção, porque alguns vetores são compartilhados com o TIM1. Para chamar a API `FromISR` do FreeRTOS no callback, a prioridade precisa ser numericamente ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` (5). Um timer usado aqui não pode ser usado ao mesmo tempo para PWM.

### `lcd/driver_lcd16x2.h`
Ligação entre a biblioteca `lcd16x2` e o hardware: pinos D4 a D7, EN e RS em **PA0 a PA5**. Tem `driver_lcd16x2_init()` e as funções `write_d4` ... `write_rs` (`static inline`), que são passadas como callbacks.

---

## Configuração (`include/env.h`)

Concentra tudo que depende da placa e do motor. Só o `src/` pode incluí-lo; as bibliotecas recebem esses valores pelas structs de configuração.

| Grupo | Valores atuais |
|---|---|
| Clocks | HSI 16 MHz, APB1 = APB2 = 16 MHz |
| I2C1 | SCL = PB6, SDA = PB7 (AF4), fast mode |
| USART1 (log) | TX = PA9, RX = PA10 (AF7), 115200 8N1 |
| Timer do AS5600 | TIM10, 1 ms, prioridade 6 (`TIM1_UP_TIM10_IRQHandler`) |
| DRV8313 | PWM no TIM3 a 20 kHz center-aligned: IN1 = PA6, IN2 = PA7, IN3 = PB0 (AF2); EN = PB12; nFAULT = PB13 |
| Timer de comutação | TIM5 (32 bits), prioridade 2 (`TIM5_IRQHandler`) |
| Motor e rampa | Pares de polos, alinhamento, `START_RPM`, `RAMP_END_RPM`, `TARGET_RPM`, `MAX_RPM`, rampa, V/f (`U_OFFSET`, `U_PER_RPM`, `U_MAX`), `AUTOSTART` |
| Controle de velocidade | `CLOSED_LOOP`, `KP`, `KI`, `KD` (em u por rpm), `SPEED_FILTER_MS`, `COMM_RATE_HZ` (10 kHz) |

Para trocar um timer, basta mudar o `env.h`: o nome do handler (`ENV_xxx_TIM_IRQHandler`) também vem dele.

---

## Como as peças se conectam (`main.c`)

**Inicialização**, nesta ordem (qualquer falha para em `init_error()`, com log quando a UART já existe):
1. `SystemCoreClockUpdate`, UART (log) e o mutex do log.
2. I2C1, `task_as5600_create` e o timer do AS5600 (TIM10), já rodando.
3. `drv8313_init`, o timer de comutação (TIM5, parado) e `task_drv8313_create`.
4. `task_as5600_set_sample_callback(task_drv8313_encoder_sample)`: cada leitura do encoder acorda a task do motor.
5. Com `ENV_MOTOR_AUTOSTART`, `task_drv8313_start()` (o comando fica pendente até a task rodar) e então `vTaskStartScheduler()`.

**Log:** `log_write` envia pela USART1. Com o scheduler rodando, usa um mutex para as mensagens das tasks não se misturarem. Cada origem usa um prefixo: `[main]`, `[as5600]` ou `[drv8313]`.

**Fluxo de dados em funcionamento:**

```
TIM10 (1 kHz, prio 6) --notify--> task_as5600 (prio 2) --I2C--> AS5600
                                       |
                                       | callback de amostra (raw, velocidade, dt)
                                       v
                          task_drv8313_encoder_sample
                            | atualiza ângulo/velocidade da ISR
                            | xTaskNotify(EVT_SAMPLE)
                            v
                       task_drv8313 (prio 3): PID -> u

TIM5 (10 kHz, prio 2) --> ISR de comutação: ângulo estimado -> setor -> sixstep -> drv8313 (TIM3, 20 kHz)
```

| Contexto | Prioridade | Função |
|---|---|---|
| ISR do TIM5 | 2 (acima do kernel) | Comutação: setor e duty cycles |
| ISR do TIM10 | 6 | Notifica a `task_as5600` |
| `task_drv8313` | 3 | Estados, rampa, PID e nFAULT |
| `task_as5600` | 2 | Leitura do encoder |

## Pontos de atenção

1. **Não dá para deixar uma fase flutuando no SimpleFOC Mini.** Na placa os três EN estão ligados juntos; o `sixstep` deixa a fase desligada em 50%. Para uma fase flutuando de verdade (necessária também para o BEMF sensorless), seria preciso separar os pinos EN1, EN2 e EN3 no hardware.
2. **Carga da CPU a 16 MHz.** A ISR de comutação a 10 kHz e a leitura do I2C a 1 kHz, que espera o barramento por cerca de 130 µs a cada leitura, ocupam uma parte grande do processador. Ligar o PLL (até 100 MHz) com o `rcc_init` dá folga; nesse caso, ajuste os clocks no `env.h`.
3. **Os pinos do DRV8313 e os parâmetros do motor são sugestões.** Confira as ligações do SimpleFOC Mini e ajuste os pares de polos, a tensão V/f e os ganhos do PID para o seu motor. Pares de polos errados fazem a calibração ser recusada (o motor fica em malha aberta).
4. **O motor parte sozinho ao ligar** com `ENV_MOTOR_AUTOSTART` = 1. Para os primeiros testes, use uma fonte com limite de corrente.
5. **Conflitos de pinos e timers.**
   - A USART1 em PA9/PA10 ocupa os pinos do CH2/CH3 do TIM1. Se o `inverter` voltar a ser usado, mude o log para a USART6 (PA11/PA12, AF8).
   - O TIM10 divide o vetor de interrupção com o update do TIM1.
   - O LCD, se voltar a ser usado, ocupa PA0 a PA5.
6. **Velocidade máxima.** Com o encoder lido a cada 1 ms, o rotor precisa girar menos de meia volta entre leituras (até 30.000 rpm). Na prática, quem limita é a precisão da estimativa de ângulo entre amostras, que piora com a velocidade.
