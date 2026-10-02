/**
 * @file inverter.h
 * @brief Biblioteca de controle de inversor trifásico com o timer advanced (TIM1) do STM32F411.
 *
 * Gera 3 pares de PWM complementares (high/low) com dead-time, proteção por break,
 * sincronismo com ADC e callbacks para a malha de controle.
 *
 * Fluxo típico:
 * @code
 * inverter_init(&inv, &cfg);
 * inverter_set_callbacks(&inv, &cbs);
 * inverter_start(&inv);
 * inverter_set_state(&inv, INVERTER_STATE_ON);
 * inverter_set_duty_cycle_normalized(&inv, &duty);
 * @endcode
 */

#ifndef INVERTER_H
#define INVERTER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <stm32f411xe.h>

#include "drivers/gpio/driver_gpio.h"

/** @brief Único timer advanced disponível no STM32F411. */
#define INVERTER_TIMER TIM1

/** @brief Número de fases (canais) do inversor. */
#define MAX_CHN_INVERTER 3

/* ------------------------------------------------------------------ */
/* Tipos                                                               */
/* ------------------------------------------------------------------ */

/** @brief Códigos de retorno da biblioteca. */
typedef enum
{
    INVERTER_OK = 0,                       /**< Sucesso. */
    INVERTER_ERROR_INVALID_ARGUMENT = -1,  /**< Ponteiro NULL ou parâmetro inválido. */
    INVERTER_ERROR_NO_STATE = -2,          /**< Estado solicitado inexistente. */
    INVERTER_ERROR_OUT_OF_RANGE = -3,      /**< Valor fora da faixa suportada (ARR, duty, dead-time...). */
    INVERTER_ERROR_LOCKED = -4,            /**< Registrador travado pelo LOCK do BDTR. */
    INVERTER_ERROR_FAULT = -5,             /**< Falha de break ativa (flag BIF). */
    INVERTER_ERROR_NOT_INITIALIZED = -6,   /**< Handle não inicializado com inverter_init(). */
} inverter_error_t;

/** @brief Estado das saídas PWM (bit MOE do BDTR). */
typedef enum
{
    INVERTER_STATE_OFF = 0, /**< Saídas desabilitadas (pinos no estado idle). */
    INVERTER_STATE_ON = 1,  /**< Saídas habilitadas. */
} inverter_state_t;

/** @brief Modo de alinhamento do contador (bits CMS do CR1). */
typedef enum
{
    INVERTER_ALIGN_EDGE = 0,     /**< Edge-aligned (contagem crescente). */
    INVERTER_ALIGN_CENTER_1 = 1, /**< Center-aligned 1: flag de comparação só na descida. */
    INVERTER_ALIGN_CENTER_2 = 2, /**< Center-aligned 2: flag de comparação só na subida. */
    INVERTER_ALIGN_CENTER_3 = 3, /**< Center-aligned 3: flag de comparação na subida e na descida. */
} inverter_align_t;

/** @brief Modo de saída do canal (bits OCxM). */
typedef enum
{
    INVERTER_OUT_PWM1 = 0x6,          /**< PWM modo 1. */
    INVERTER_OUT_PWM2 = 0x7,          /**< PWM modo 2 (lógica invertida em relação ao PWM1). */
    INVERTER_OUT_FORCE_INACTIVE = 0x4,/**< Força OCxREF inativo. */
    INVERTER_OUT_FORCE_ACTIVE = 0x5,  /**< Força OCxREF ativo. */
} inverter_output_mode_t;

/** @brief Polaridade de saída (CCxP / CCxNP). */
typedef enum
{
    INVERTER_POLARITY_ACTIVE_HIGH = 0, /**< Ativo em nível alto. */
    INVERTER_POLARITY_ACTIVE_LOW = 1,  /**< Ativo em nível baixo (drivers invertidos). */
} inverter_polarity_t;

/**
 * @brief Nível de proteção de escrita do BDTR (bits LOCK).
 * @warning O LOCK só pode ser escrito uma vez após o reset do timer.
 */
typedef enum
{
    INVERTER_LOCK_OFF = 0,    /**< Sem proteção. */
    INVERTER_LOCK_LEVEL1 = 1, /**< Trava dead-time, break, AOE e estados idle (OISx). */
    INVERTER_LOCK_LEVEL2 = 2, /**< Nível 1 + polaridade e OSSR/OSSI. */
    INVERTER_LOCK_LEVEL3 = 3, /**< Nível 2 + modo de saída (OCxM) e preload (OCxPE). */
} inverter_lock_level_t;

/** @brief Identificador de fase do inversor. */
typedef enum
{
    INVERTER_CH_A = 0, /**< Fase A (CH1/CH1N). */
    INVERTER_CH_B = 1, /**< Fase B (CH2/CH2N). */
    INVERTER_CH_C = 2, /**< Fase C (CH3/CH3N). */
} inverter_channel_t;

/** @brief Configuração de um pino de saída PWM. */
typedef struct
{
    uint16_t pin;                /**< Número do pino (conforme o driver_gpio). */
    uint16_t alternate_function; /**< Função alternativa (AFx) do TIM1 para o pino. */
} inverter_gpio_config_t;

/** @brief Estado dos pinos quando MOE = 0 (idle state). */
typedef struct
{
    bool high_idle_level[MAX_CHN_INVERTER]; /**< Nível idle das saídas high (OISx). */
    bool low_idle_level[MAX_CHN_INVERTER];  /**< Nível idle das saídas low (OISxN). */
    bool off_state_run;                     /**< OSSR: saídas controladas pelo timer em idle com MOE = 1. */
    bool off_state_idle;                    /**< OSSI: saídas mantêm o nível idle com MOE = 0. */
} inverter_idle_config_t;

/** @brief Configuração da entrada de break (BKIN). */
typedef struct
{
    bool enable;           /**< BKE: habilita a entrada de break. */
    bool active_high;      /**< BKP: true = ativo em nível alto. */
    uint8_t filter;        /**< BKF: filtro digital (0..15). */
    bool automatic_output; /**< AOE: religa o MOE automaticamente no próximo update. */
} inverter_break_config_t;

/** @brief Configuração do trigger do ADC via canal 4 do timer. */
typedef struct
{
    bool enable;            /**< Habilita/desabilita o trigger. */
    uint16_t compare_value; /**< CCR4: posição do disparo no período (deve ser <= ARR). */
    bool use_trgo;          /**< true: também envia OC4REF em TRGO; false: só o evento CC4. */
} inverter_adc_trigger_config_t;

/**
 * @brief Tipo de callback chamado de dentro de uma ISR.
 * @param context Ponteiro definido pelo usuário em inverter_callbacks_t::context.
 */
typedef void (*inverter_callback_t)(void *context);

/** @brief Conjunto de callbacks do inversor. */
typedef struct
{
    inverter_callback_t on_update; /**< Chamado a cada update event (período ou RCR). */
    inverter_callback_t on_break;  /**< Chamado em evento de break/falha. */
    void *context;                 /**< Argumento repassado aos callbacks. */
} inverter_callbacks_t;

/** @brief Configuração de inicialização do inversor. */
typedef struct
{
    TIM_TypeDef *advanced_timer; /**< Deve ser #INVERTER_TIMER (TIM1). */

    uint32_t timer_clock_hz;     /**< Clock real do TIM1 (clock de timer do APB2), em Hz. */

    uint8_t deadtime;            /**< Valor bruto do DTG (codificação não linear, ver RM0383). */
    uint16_t autorreload;        /**< ARR (deve ser > 0). */
    uint16_t prescale;           /**< PSC. */
    uint8_t repetition_counter;  /**< RCR: update event a cada RCR+1 períodos. */

    inverter_align_t alignment;         /**< Modo de alinhamento do contador. */
    inverter_polarity_t polarity_high;  /**< Polaridade das saídas high. */
    inverter_polarity_t polarity_low;   /**< Polaridade das saídas low. */

    GPIO_TypeDef *gpioH; /**< Porta GPIO dos pinos high (CHx). */
    GPIO_TypeDef *gpioL; /**< Porta GPIO dos pinos low (CHxN). */

    inverter_gpio_config_t config_gpioH[MAX_CHN_INVERTER]; /**< Pinos high das fases A, B e C. */
    inverter_gpio_config_t config_gpioL[MAX_CHN_INVERTER]; /**< Pinos low das fases A, B e C. */

    inverter_idle_config_t idle;  /**< Configuração do estado idle. */
    inverter_break_config_t brk;  /**< Configuração do break. */

} inverter_config_t;

/**
 * @brief Handle do inversor, compartilhado por todas as funções.
 * @note Os campos são de uso interno; não os modifique diretamente.
 */
typedef struct
{
    TIM_TypeDef *timer;            /**< Timer em uso. */
    uint32_t timer_clock_hz;       /**< Clock do timer, em Hz. */
    uint16_t autorreload;          /**< Último ARR configurado. */
    uint16_t prescale;             /**< Último PSC configurado. */
    uint8_t deadtime;              /**< Último DTG configurado. */
    inverter_align_t alignment;    /**< Alinhamento atual. */
    bool initialized;              /**< true após inverter_init() bem-sucedido. */
    inverter_callbacks_t callbacks;/**< Callbacks registrados. */
} inverter_t;

/** @brief Duty cycles em contagens do timer (0..ARR). */
typedef struct
{
    uint16_t a; /**< Fase A (CCR1). */
    uint16_t b; /**< Fase B (CCR2). */
    uint16_t c; /**< Fase C (CCR3). */
} inverter_duty_t;

/** @brief Duty cycles normalizados (0.0 .. 1.0). Valores fora da faixa são saturados. */
typedef struct
{
    float a; /**< Fase A. */
    float b; /**< Fase B. */
    float c; /**< Fase C. */
} inverter_duty_norm_t;

/* ------------------------------------------------------------------ */
/* Ciclo de vida                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Inicializa o timer e os GPIOs do inversor.
 *
 * Reseta o TIM1 (limpando qualquer LOCK anterior) e o configura. Ao final o timer
 * está parado e as saídas desligadas (MOE = 0); use inverter_start() e
 * inverter_set_state() para ativá-las.
 *
 * @param[out] inv    Handle a ser preenchido.
 * @param[in]  config Configuração desejada.
 * @retval INVERTER_OK                      Sucesso.
 * @retval INVERTER_ERROR_INVALID_ARGUMENT  NULL, timer diferente de TIM1, ARR/clock = 0, filtro > 15 ou falha no driver GPIO.
 */
inverter_error_t inverter_init(inverter_t *inv, const inverter_config_t *config);

/**
 * @brief Desliga as saídas e o timer, desabilita interrupções e o clock do TIM1.
 * @param[in,out] inv Handle inicializado.
 * @retval INVERTER_OK                     Sucesso.
 * @retval INVERTER_ERROR_NOT_INITIALIZED  Handle não inicializado.
 * @note Os GPIOs não são devolvidos ao estado analógico.
 */
inverter_error_t inverter_deinit(inverter_t *inv);

/**
 * @brief Inicia o contador do timer (CEN = 1).
 * @param[in,out] inv Handle inicializado.
 * @retval INVERTER_OK Sucesso.
 * @note Não habilita as saídas; veja inverter_set_state().
 */
inverter_error_t inverter_start(inverter_t *inv);

/**
 * @brief Desliga as saídas (MOE = 0) e para o contador (CEN = 0).
 * @param[in,out] inv Handle inicializado.
 * @retval INVERTER_OK Sucesso.
 */
inverter_error_t inverter_stop(inverter_t *inv);

/* ------------------------------------------------------------------ */
/* Estado das saídas (MOE)                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief Habilita ou desabilita as saídas PWM (bit MOE).
 * @param[in,out] inv   Handle inicializado.
 * @param[in]     state Estado desejado.
 * @retval INVERTER_OK             Sucesso.
 * @retval INVERTER_ERROR_FAULT    Tentativa de ligar com falha de break pendente; chame inverter_clear_fault().
 * @retval INVERTER_ERROR_NO_STATE Estado inválido.
 */
inverter_error_t inverter_set_state(inverter_t *inv, inverter_state_t state);

/**
 * @brief Lê o estado atual das saídas.
 * @param[in] inv Handle do inversor.
 * @return #INVERTER_STATE_ON ou #INVERTER_STATE_OFF (também retorna OFF se @p inv for NULL).
 */
inverter_state_t inverter_get_state(const inverter_t *inv);

/* ------------------------------------------------------------------ */
/* Frequência / dead-time / duty                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Define a frequência de chaveamento (recalcula PSC e ARR).
 *
 * Center-aligned: f = fclk / (2 · (PSC+1) · ARR).
 * Edge-aligned:   f = fclk / ((PSC+1) · (ARR+1)).
 *
 * Com o timer parado a alteração é aplicada imediatamente; com ele rodando, no próximo update.
 *
 * @param[in,out] inv          Handle inicializado.
 * @param[in]     frequency_hz Frequência desejada em Hz (> 0).
 * @retval INVERTER_OK                     Sucesso.
 * @retval INVERTER_ERROR_INVALID_ARGUMENT Frequência zero.
 * @retval INVERTER_ERROR_OUT_OF_RANGE     Frequência inatingível com o clock do timer.
 * @warning Os duty cycles não são reescalados; reajuste-os após mudar o ARR.
 */
inverter_error_t inverter_set_frequency(inverter_t *inv, uint32_t frequency_hz);

/**
 * @brief Calcula a frequência de chaveamento atual a partir de PSC/ARR.
 * @param[in]  inv          Handle inicializado.
 * @param[out] frequency_hz Frequência em Hz.
 * @retval INVERTER_OK Sucesso.
 */
inverter_error_t inverter_get_frequency(const inverter_t *inv, uint32_t *frequency_hz);

/**
 * @brief Define o dead-time com o valor bruto do DTG.
 * @param[in,out] inv Handle inicializado.
 * @param[in]     dtg Valor do campo DTG[7:0] (codificação não linear).
 * @retval INVERTER_OK            Sucesso.
 * @retval INVERTER_ERROR_LOCKED  BDTR travado (LOCK >= 1).
 */
inverter_error_t inverter_set_deadtime(inverter_t *inv, uint8_t dtg);

/**
 * @brief Define o dead-time em nanossegundos, escolhendo a codificação do DTG.
 *
 * Considera tDTS = 1/fclk (CKD = 0). Valores acima do mínimo representável
 * são arredondados para o passo disponível naquela faixa.
 *
 * @param[in,out] inv         Handle inicializado.
 * @param[in]     deadtime_ns Dead-time desejado em ns.
 * @retval INVERTER_OK                 Sucesso.
 * @retval INVERTER_ERROR_OUT_OF_RANGE Dead-time maior que o máximo representável.
 * @retval INVERTER_ERROR_LOCKED       BDTR travado.
 */
inverter_error_t inverter_set_deadtime_ns(inverter_t *inv, uint32_t deadtime_ns);

/**
 * @brief Atualiza os três duty cycles em contagens do timer.
 * @param[in,out] inv  Handle inicializado.
 * @param[in]     duty Valores de CCR1..CCR3 (cada um <= ARR).
 * @retval INVERTER_OK                 Sucesso.
 * @retval INVERTER_ERROR_OUT_OF_RANGE Algum valor maior que ARR (nenhum canal é alterado).
 * @note Os CCRs têm preload: o valor passa a valer no próximo update event.
 */
inverter_error_t inverter_set_duty_cycle(inverter_t *inv, const inverter_duty_t *duty);

/**
 * @brief Atualiza os três duty cycles em valores normalizados (0.0 a 1.0).
 * @param[in,out] inv  Handle inicializado.
 * @param[in]     duty Duty por fase; valores fora da faixa (e NaN) são saturados.
 * @retval INVERTER_OK Sucesso.
 */
inverter_error_t inverter_set_duty_cycle_normalized(inverter_t *inv, const inverter_duty_norm_t *duty);

/**
 * @brief Lê os valores atuais de CCR1..CCR3.
 * @param[in]  inv  Handle inicializado.
 * @param[out] duty Duty cycles em contagens do timer.
 * @retval INVERTER_OK Sucesso.
 */
inverter_error_t inverter_get_duty_cycle(const inverter_t *inv, inverter_duty_t *duty);

/* ------------------------------------------------------------------ */
/* Proteção                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Configura a entrada de break (BKE, BKP, BKF e AOE).
 * @param[in,out] inv Handle inicializado.
 * @param[in]     cfg Configuração do break.
 * @retval INVERTER_OK                     Sucesso.
 * @retval INVERTER_ERROR_INVALID_ARGUMENT NULL ou filtro > 15.
 * @retval INVERTER_ERROR_LOCKED           BDTR travado.
 */
inverter_error_t inverter_config_break(inverter_t *inv, const inverter_break_config_t *cfg);

/**
 * @brief Indica se há falha de break pendente (flag BIF).
 * @param[in] inv Handle do inversor.
 * @return true se houver falha; false caso contrário (ou se @p inv for NULL).
 */
bool inverter_get_fault(const inverter_t *inv);

/**
 * @brief Limpa o flag de falha (BIF).
 *
 * O MOE não é religado aqui: depende do AOE ou de inverter_set_state(ON).
 *
 * @param[in,out] inv Handle inicializado.
 * @retval INVERTER_OK          Falha limpa.
 * @retval INVERTER_ERROR_FAULT A entrada de break ainda está ativa e o flag foi setado de novo.
 */
inverter_error_t inverter_clear_fault(inverter_t *inv);

/**
 * @brief Configura o nível dos pinos quando as saídas estão desabilitadas (OISx, OISxN, OSSR, OSSI).
 * @param[in,out] inv Handle inicializado.
 * @param[in]     cfg Configuração do estado idle.
 * @retval INVERTER_OK            Sucesso.
 * @retval INVERTER_ERROR_LOCKED  Registradores travados.
 */
inverter_error_t inverter_set_idle_state(inverter_t *inv, const inverter_idle_config_t *cfg);

/**
 * @brief Trava a configuração de proteção usando os bits LOCK do BDTR.
 * @param[in,out] inv   Handle inicializado.
 * @param[in]     level Nível de travamento.
 * @retval INVERTER_OK           Sucesso (ou @p level = #INVERTER_LOCK_OFF).
 * @retval INVERTER_ERROR_LOCKED Já travado anteriormente ou a escrita não surtiu efeito.
 * @warning Irreversível até o reset do timer (inverter_init() / inverter_deinit()).
 */
inverter_error_t inverter_lock_config(inverter_t *inv, inverter_lock_level_t level);

/* ------------------------------------------------------------------ */
/* Sincronismo com ADC / malha de controle                             */
/* ------------------------------------------------------------------ */

/**
 * @brief Configura o canal 4 para disparar o ADC (injetado, JEXTSEL = TIM1_CC4) e/ou TRGO.
 *
 * O CH4 não gera saída no pino (CC4E = 0); serve apenas como evento interno.
 *
 * @param[in,out] inv Handle inicializado.
 * @param[in]     cfg Configuração do trigger.
 * @retval INVERTER_OK                 Sucesso.
 * @retval INVERTER_ERROR_OUT_OF_RANGE compare_value > ARR.
 */
inverter_error_t inverter_config_adc_trigger(inverter_t *inv, const inverter_adc_trigger_config_t *cfg);

/**
 * @brief Define o repetition counter (RCR).
 * @param[in,out] inv Handle inicializado.
 * @param[in]     rcr Update event a cada rcr+1 períodos; vale no próximo update.
 * @retval INVERTER_OK Sucesso.
 */
inverter_error_t inverter_set_repetition_counter(inverter_t *inv, uint8_t rcr);

/**
 * @brief Registra os callbacks de update e break (operação atômica).
 * @param[in,out] inv       Handle inicializado.
 * @param[in]     callbacks Callbacks a registrar; NULL remove todos.
 * @retval INVERTER_OK Sucesso.
 * @note Os callbacks executam em contexto de interrupção; mantenha-os curtos.
 */
inverter_error_t inverter_set_callbacks(inverter_t *inv, const inverter_callbacks_t *callbacks);

/**
 * @brief Habilita/desabilita a interrupção de update (UIE) e o vetor no NVIC.
 * @param[in,out] inv    Handle inicializado.
 * @param[in]     enable true para habilitar.
 * @retval INVERTER_OK Sucesso.
 * @note Ao desabilitar, o NVIC não é desligado pois o vetor é compartilhado com o TIM10.
 */
inverter_error_t inverter_enable_update_irq(inverter_t *inv, bool enable);

/**
 * @brief Habilita/desabilita a interrupção de break (BIE) e o vetor no NVIC.
 * @param[in,out] inv    Handle inicializado.
 * @param[in]     enable true para habilitar.
 * @retval INVERTER_OK Sucesso.
 */
inverter_error_t inverter_enable_break_irq(inverter_t *inv, bool enable);

/**
 * @brief Handler da interrupção de update; chame de TIM1_UP_TIM10_IRQHandler().
 *
 * Limpa o flag UIF e executa inverter_callbacks_t::on_update.
 *
 * @param[in,out] inv Handle do inversor.
 */
void inverter_irq_update_handler(inverter_t *inv);

/**
 * @brief Handler da interrupção de break; chame de TIM1_BRK_TIM9_IRQHandler().
 *
 * Executa inverter_callbacks_t::on_break e desabilita o BIE para evitar re-disparo.
 * O flag BIF é mantido; após inverter_clear_fault(), reabilite com inverter_enable_break_irq().
 *
 * @param[in,out] inv Handle do inversor.
 */
void inverter_irq_break_handler(inverter_t *inv);

/* ------------------------------------------------------------------ */
/* Flexibilidade                                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief Habilita/desabilita individualmente os lados high e low de uma fase (CCxE / CCxNE).
 * @param[in,out] inv  Handle inicializado.
 * @param[in]     ch   Fase.
 * @param[in]     high true para habilitar a saída high.
 * @param[in]     low  true para habilitar a saída low.
 * @retval INVERTER_OK                     Sucesso.
 * @retval INVERTER_ERROR_INVALID_ARGUMENT Fase inválida.
 */
inverter_error_t inverter_set_channel_enable(inverter_t *inv, inverter_channel_t ch, bool high, bool low);

/**
 * @brief Define a polaridade das saídas high e low (aplicada às três fases).
 * @param[in,out] inv  Handle inicializado.
 * @param[in]     high Polaridade das saídas high.
 * @param[in]     low  Polaridade das saídas low.
 * @retval INVERTER_OK           Sucesso.
 * @retval INVERTER_ERROR_LOCKED LOCK >= 2.
 */
inverter_error_t inverter_set_polarity(inverter_t *inv, inverter_polarity_t high, inverter_polarity_t low);

/**
 * @brief Altera o modo de alinhamento do contador.
 * @param[in,out] inv       Handle inicializado.
 * @param[in]     alignment Novo alinhamento.
 * @retval INVERTER_OK                     Sucesso.
 * @retval INVERTER_ERROR_INVALID_ARGUMENT Alinhamento inválido ou timer em execução.
 * @note A frequência muda com o alinhamento; recalcule com inverter_set_frequency().
 */
inverter_error_t inverter_set_alignment(inverter_t *inv, inverter_align_t alignment);

/**
 * @brief Define o modo de saída (OCxM) de uma fase.
 * @param[in,out] inv  Handle inicializado.
 * @param[in]     ch   Fase.
 * @param[in]     mode Modo de saída.
 * @retval INVERTER_OK                     Sucesso.
 * @retval INVERTER_ERROR_INVALID_ARGUMENT Fase ou modo inválido.
 * @retval INVERTER_ERROR_LOCKED           LOCK = 3.
 */
inverter_error_t inverter_set_output_mode(inverter_t *inv, inverter_channel_t ch, inverter_output_mode_t mode);

/**
 * @brief Força a saída de uma fase em ativo/inativo (útil para testes e frenagem).
 * @param[in,out] inv    Handle inicializado.
 * @param[in]     ch     Fase.
 * @param[in]     active true = force active; false = force inactive.
 * @retval INVERTER_OK Sucesso.
 * @note Para voltar ao PWM use inverter_set_output_mode() com #INVERTER_OUT_PWM1.
 */
inverter_error_t inverter_force_output(inverter_t *inv, inverter_channel_t ch, bool active);

#endif /* INVERTER_H */