/**
 * @brief Defines de utilizados pelo projeto
 *
 * @note São utilizadas para realizar as configurações
 * iniciais dos perifericos.
 *
 * @warning Nao utilizar nas bibliotecas desenvolvidas (lib/)
 */

#ifndef INCLUDE_H
#define INCLUDE_H

/*
 * ---------- Clocks: PLL a partir do HSE (cristal de 25 MHz do Black Pill) ----------
 * SYSCLK = HSE / M · N / P = 25 MHz / 25 · 200 / 4 = 50 MHz
 * (entrada do VCO = 1 MHz, saída do VCO = 200 MHz). AHB, APB1 e APB2 sem
 * prescaler: o APB1 tem limite de 50 MHz no STM32F411.
 * Com prescaler 1 no APB, o clock dos timers é igual ao PCLK.
 */
#define ENV_SYSCLK_HZ 50000000U
#define ENV_PLL_M     25U
#define ENV_PLL_N     200U
#define ENV_PLL_P     RCC_PLL_P_DIV_4
#define ENV_PCLK1_HZ  ENV_SYSCLK_HZ
#define ENV_PCLK2_HZ  ENV_SYSCLK_HZ

/* ---------- I2C1: SCL = PB6, SDA = PB7 (AF4) ---------- */
#define ENV_I2C            I2C1
#define ENV_I2C_SPEED      I2C_SPEED_FAST
#define ENV_I2C_SCL_PORT   GPIOB
#define ENV_I2C_SCL_PIN    6U
#define ENV_I2C_SDA_PORT   GPIOB
#define ENV_I2C_SDA_PIN    7U
#define ENV_I2C_AF         4U

/*
 * ---------- USART1: TX = PA9, RX = PA10 (AF7) ----------
 * 921600 baud: o CSV de 1 ms (~45 bytes por linha) usa cerca de metade da banda.
 * No PC: stty -F /dev/ttyUSB0 921600 raw -echo && cat /dev/ttyUSB0 > log.csv
 */
#define ENV_UART           USART1
#define ENV_UART_BAUDRATE  921600U  /* BRR = 54 a 50 MHz: erro de 0,5% */
#define ENV_UART_PORT      GPIOA
#define ENV_UART_TX_PIN    9U
#define ENV_UART_RX_PIN    10U
#define ENV_UART_AF        7U
#define ENV_LOG_PERIOD_MS  1U         /* uma linha do CSV por ms */

/*
 * ---------- AS5600: fonte do ângulo ----------
 * 0: RAW ANGLE pelo I2C1 (~150 µs por leitura, amostragem de 1 kHz).
 * 1: saída analógica OUT pelo ADC1 (~11 µs por leitura, amostragem de 5 kHz).
 */
#define ENV_AS5600_USE_ADC         0

/*
 * ---------- AS5600: saída analógica (OUT) no ADC1 (só com ENV_AS5600_USE_ADC = 1) ----------
 * OUT em PA1 (ADC1_IN1). Com o AS5600 e o STM32 em 3,3 V a leitura é
 * ratiométrica: 0..4095 do ADC = 0..360°. Ajuste ADC_MIN/ADC_MAX se a saída
 * não chegar aos extremos (veja task_as5600_get_adc() girando o ímã devagar).
 */
#define ENV_AS5600_ADC_PORT        GPIOA
#define ENV_AS5600_ADC_PIN         1U
#define ENV_AS5600_ADC_SAMPLE      ADC_SAMPLE_56 /* ADCCLK = 25 MHz: (56 + 12) / 25 MHz = 2,7 µs por conversão */
#define ENV_AS5600_ADC_OVERSAMPLE  4U            /* conversões por amostra (média) */
#define ENV_AS5600_ADC_MIN         0U
#define ENV_AS5600_ADC_MAX         4095U
#define ENV_AS5600_USE_I2C         1             /* fonte ADC: 1 configura o sensor e confere o ímã pelo I2C */

/*
 * ---------- Timer da leitura do AS5600 (general purpose) ----------
 * Ao trocar o timer, ajuste também o clock (APB1: TIM2..5; APB2: TIM9..11)
 * e o nome do handler da tabela de vetores.
 */
#define ENV_AS5600_TIM              TIM10
#define ENV_AS5600_TIM_CLOCK_HZ     ENV_PCLK2_HZ /* APB2 sem prescaler: clock do timer = PCLK2 */
#if ENV_AS5600_USE_ADC
#define ENV_AS5600_TIM_PERIOD_US    200U         /* 5 kHz (o AS5600 atualiza a cada ~150 µs) */
#else
#define ENV_AS5600_TIM_PERIOD_US    1000U        /* 1 kHz: uma leitura I2C ocupa ~150 µs */
#endif
#define ENV_AS5600_VEL_WINDOW_US    1000U        /* janela da velocidade: 15 rpm por contagem */
#define ENV_AS5600_TIM_IRQ_PRIORITY 6U           /* >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) */
#define ENV_AS5600_TIM_IRQHandler   TIM1_UP_TIM10_IRQHandler /* compartilhado com o update do TIM1 */

/*
 * ---------- DRV8313 (SimpleFOC Mini) ----------
 * PWM no TIM3: IN1 = PA6 (CH1), IN2 = PA7 (CH2), IN3 = PB0 (CH3), AF2.
 * Confira as ligações: os pinos de controle abaixo são sugestões.
 */
#define ENV_DRV_TIM             TIM3
#define ENV_DRV_TIM_CLOCK_HZ    ENV_PCLK1_HZ /* APB1 sem prescaler */
#define ENV_DRV_PWM_HZ          20000U       /* 20 kHz: ARR = 1250 com 50 MHz */
#define ENV_DRV_IN1_PORT        GPIOA
#define ENV_DRV_IN1_PIN         6U
#define ENV_DRV_IN2_PORT        GPIOA
#define ENV_DRV_IN2_PIN         7U
#define ENV_DRV_IN3_PORT        GPIOB
#define ENV_DRV_IN3_PIN         0U
#define ENV_DRV_IN_AF           2U
#define ENV_DRV_EN_PORT         GPIOB
#define ENV_DRV_EN_PIN          12U
#define ENV_DRV_NFAULT_PORT     GPIOB        /* NULL se não ligado */
#define ENV_DRV_NFAULT_PIN      13U

/*
 * ---------- Timer de comutação do six-step ----------
 * TIM5 (32 bits): cobre de passos lentos na partida a altas rotações.
 * A ISR não usa a API do FreeRTOS, então pode ficar acima do limite (5).
 */
#define ENV_COMM_TIM              TIM5
#define ENV_COMM_TIM_CLOCK_HZ     ENV_PCLK1_HZ
#define ENV_COMM_TIM_IRQ_PRIORITY 2U
#define ENV_COMM_TIM_IRQHandler   TIM5_IRQHandler

/*
 * ---------- Motor e partida em rampa (malha aberta) ----------
 * Valores iniciais: ajuste para o seu motor (tensão alta demais aquece o
 * motor e aciona a proteção do DRV8313; baixa demais perde o sincronismo).
 */
#define ENV_MOTOR_POLE_PAIRS      7U
#define ENV_MOTOR_ALIGN_U         0.15f
#define ENV_MOTOR_ALIGN_MS        500U        /* cada um dos 2 passos do alinhamento */
#define ENV_MOTOR_START_RPM       60.0f
#define ENV_MOTOR_RAMP_END_RPM    500.0f      /* fim da rampa: entra na malha fechada */
#define ENV_MOTOR_TARGET_RPM      1150.0f
#define ENV_MOTOR_MAX_RPM         2500.0f     /* < 30000 / período do AS5600 em ms (meia volta por amostra) */
#define ENV_MOTOR_RAMP_RPM_PER_S  300.0f
#define ENV_MOTOR_U_OFFSET        0.10f
#define ENV_MOTOR_U_PER_RPM       0.0002f
#define ENV_MOTOR_U_MAX           0.50f
#define ENV_MOTOR_AUTOSTART       1           /* 1: parte sozinho ao ligar */

/*
 * ---------- Controle de velocidade (PID + AS5600) ----------
 * Como em simulation_bldc_velocidade: PID em pu de velocidade (base =
 * ENV_MOTOR_TARGET_RPM) e saída = tensão normalizada, limitada a [0, U_MAX].
 * Com KP = 5 e alvo de 1700 rpm, um erro de 17 rpm (0,01 pu) soma 0,05 à tensão.
 * Para converter ganhos em u/rpm: ganho_pu = ganho_u_por_rpm · TARGET_RPM.
 */
#define ENV_MOTOR_CLOSED_LOOP     1           /* 0: fica em malha aberta após a rampa */
#define ENV_MOTOR_KP              5.0f        /* u por pu (simulação: Kp = 5) */
#define ENV_MOTOR_KI              1.0f        /* u por pu·s (simulação: Ki = 1) */
#define ENV_MOTOR_KD              2.0f
#define ENV_MOTOR_SPEED_FILTER_MS 5.0f        /* filtro passa-baixa sobre a velocidade da janela */
#define ENV_MOTOR_COMM_RATE_HZ    10000U      /* ISR de comutação na malha fechada */

#endif
