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

/* ---------- Clocks (HSI 16 MHz, AHB/APB1/APB2 sem prescaler) ---------- */
#define ENV_PCLK1_HZ 16000000U
#define ENV_PCLK2_HZ 16000000U

/* ---------- I2C1: SCL = PB6, SDA = PB7 (AF4) ---------- */
#define ENV_I2C            I2C1
#define ENV_I2C_SPEED      I2C_SPEED_FAST
#define ENV_I2C_SCL_PORT   GPIOB
#define ENV_I2C_SCL_PIN    6U
#define ENV_I2C_SDA_PORT   GPIOB
#define ENV_I2C_SDA_PIN    7U
#define ENV_I2C_AF         4U

/* ---------- USART1: TX = PA9, RX = PA10 (AF7) ---------- */
#define ENV_UART           USART1
#define ENV_UART_BAUDRATE  115200U
#define ENV_UART_PORT      GPIOA
#define ENV_UART_TX_PIN    9U
#define ENV_UART_RX_PIN    10U
#define ENV_UART_AF        7U

/*
 * ---------- Timer da leitura do AS5600 (general purpose) ----------
 * Ao trocar o timer, ajuste também o clock (APB1: TIM2..5; APB2: TIM9..11)
 * e o nome do handler da tabela de vetores.
 */
#define ENV_AS5600_TIM              TIM10
#define ENV_AS5600_TIM_CLOCK_HZ     ENV_PCLK2_HZ /* APB2 sem prescaler: clock do timer = PCLK2 */
#define ENV_AS5600_TIM_PERIOD_US    10000U       /* 10 ms */
#define ENV_AS5600_TIM_IRQ_PRIORITY 6U           /* >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) */
#define ENV_AS5600_TIM_IRQHandler   TIM1_UP_TIM10_IRQHandler /* compartilhado com o update do TIM1 */

/*
 * ---------- DRV8313 (SimpleFOC Mini) ----------
 * PWM no TIM3: IN1 = PA6 (CH1), IN2 = PA7 (CH2), IN3 = PB0 (CH3), AF2.
 * Confira as ligações: os pinos de controle abaixo são sugestões.
 */
#define ENV_DRV_TIM             TIM3
#define ENV_DRV_TIM_CLOCK_HZ    ENV_PCLK1_HZ /* APB1 sem prescaler */
#define ENV_DRV_PWM_HZ          20000U       /* 20 kHz: ARR = 400 com 16 MHz */
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
 * ---------- Motor e partida em malha aberta ----------
 * Valores iniciais: ajuste para o seu motor (tensão alta demais aquece o
 * motor e aciona a proteção do DRV8313; baixa demais perde o sincronismo).
 */
#define ENV_MOTOR_POLE_PAIRS      7U
#define ENV_MOTOR_ALIGN_U         0.15f
#define ENV_MOTOR_ALIGN_MS        500U
#define ENV_MOTOR_START_RPM       60.0f
#define ENV_MOTOR_TARGET_RPM      600.0f
#define ENV_MOTOR_MAX_RPM         1500.0f
#define ENV_MOTOR_RAMP_RPM_PER_S  300.0f
#define ENV_MOTOR_U_OFFSET        0.10f
#define ENV_MOTOR_U_PER_RPM       0.0002f
#define ENV_MOTOR_U_MAX           0.50f
#define ENV_MOTOR_AUTOSTART       1           /* 1: parte sozinho ao ligar */

#endif
