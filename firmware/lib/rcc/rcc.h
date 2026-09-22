#ifndef RCC_H
#define RCC_H

#include <stdint.h>

#include <stm32f411xe.h>

/**
 * @brief Códigos de retorno das funções do módulo RCC.
 *
 * Os códigos são utilizados para indicar o resultado da execução
 * das funções de configuração do sistema de clock.
 */
typedef enum
{
    RCC_OK = 0,                            /**< Operação executada com sucesso. */
    RCC_ERR_INVALID_CLOCK_SOURCE = -1,    /**< Fonte de clock inválida. */
    RCC_ERR_INVALID_APB_NUM = -2,         /**< Número do barramento APB inválido. */
    RCC_ERR_INVALID_APB_DIVIDER = -3,     /**< Divisor do barramento APB inválido. */
    RCC_ERR_CLOCK_NOT_RDY = -4,           /**< Fonte de clock não está pronta. */
    RCC_ERR_SET_CLOCK_SOURCE_FAILED = -5, /**< Falha ao selecionar a fonte de clock. */

    RCC_ERR_INVALID_PLL_P_FACTOR = -6, /**< Fator de divisão P do PLL inválido. */
    RCC_ERR_INVALID_PLL_M_FACTOR = -7, /**< Fator de divisão M do PLL inválido. */
    RCC_ERR_INVALID_PLL_N_FACTOR = -8  /**< Fator de divisão N do PLL inválido. */
} rcc_err_t;


/**
 * @brief Fontes de clock disponíveis para o sistema.
 *
 * Define as fontes de clock que podem ser selecionadas como
 * clock do sistema do microcontrolador.
 */
typedef enum
{
    RCC_HSI = 0, /**< HSI (High-Speed Internal). */
    RCC_HSE = 1, /**< HSE (High-Speed External). */
    RCC_PLL = 2  /**< PLL (Phase-Locked Loop). */
} rcc_clock_source_t;


/**
 * @brief Fatores de divisão P válidos para o PLL.
 *
 * Define os fatores de divisão disponíveis para a saída principal
 * do PLL do STM32F411.
 */
typedef enum
{
    RCC_PLL_P_DIV_2 = 2, /**< Divisão da saída do PLL por 2. */
    RCC_PLL_P_DIV_4 = 4, /**< Divisão da saída do PLL por 4. */
    RCC_PLL_P_DIV_6 = 6, /**< Divisão da saída do PLL por 6. */
    RCC_PLL_P_DIV_8 = 8  /**< Divisão da saída do PLL por 8. */
} rcc_pll_p_factor_t;

/**
 * @brief Divisores disponíveis para os barramentos APB.
 *
 * Define os valores de divisão aplicáveis aos barramentos
 * APB1 e APB2 a partir do clock do barramento AHB.
 *
 * Os valores correspondem diretamente aos campos PPRE1 e PPRE2
 * do registrador RCC_CFGR.
 */
typedef enum
{
    RCC_APBx_DIV_0  = 0, /**< Divisão por 1. */
    RCC_APBx_DIV_2  = 4, /**< Divisão por 2. */
    RCC_APBx_DIV_4  = 5, /**< Divisão por 4. */
    RCC_APBx_DIV_8  = 6, /**< Divisão por 8. */
    RCC_APBx_DIV_16 = 7  /**< Divisão por 16. */
} rcc_apb_divider_t;


/**
 * @brief Divisores disponíveis para o barramento AHB.
 *
 * Define os valores de divisão aplicáveis ao clock do barramento
 * AHB a partir do clock do sistema.
 *
 * Os valores correspondem diretamente aos campos HPRE
 * do registrador RCC_CFGR.
 */
typedef enum
{
    RCC_AHB_DIV_0   = 0,  /**< Divisão por 1. */
    RCC_AHB_DIV_2   = 8,  /**< Divisão por 2. */
    RCC_AHB_DIV_4   = 9,  /**< Divisão por 4. */
    RCC_AHB_DIV_8   = 10, /**< Divisão por 8. */
    RCC_AHB_DIV_16  = 11, /**< Divisão por 16. */
    RCC_AHB_DIV_64  = 12, /**< Divisão por 64. */
    RCC_AHB_DIV_128 = 13, /**< Divisão por 128. */
    RCC_AHB_DIV_256 = 14, /**< Divisão por 256. */
    RCC_AHB_DIV_512 = 15  /**< Divisão por 512. */
} rcc_ahb_divider_t;


/**
 * @brief Estrutura contendo o estado atual da configuração de clock.
 *
 * Armazena a fonte de clock atualmente selecionada e os divisores
 * configurados para os barramentos AHB, APB1 e APB2.
 */
typedef struct
{
    rcc_clock_source_t clock_source; /**< Fonte de clock do sistema. */
    rcc_ahb_divider_t ahb_divider;   /**< Divisor configurado para o AHB. */
    rcc_apb_divider_t apb1_divider;  /**< Divisor configurado para o APB1. */
    rcc_apb_divider_t apb2_divider;  /**< Divisor configurado para o APB2. */
} rcc_clock_status_t;


/**
 * @brief Parâmetros de configuração do PLL.
 *
 * Define a fonte de entrada e os fatores de divisão e multiplicação
 * utilizados pelo PLL para gerar o clock de saída.
 *
 * A frequência de saída do PLL é determinada por:
 *
 * @f[
 * f_{PLL} = \frac{f_{source}}{M} \times \frac{N}{P}
 * @f]
 *
 * onde:
 * - @c source é a fonte de entrada do PLL;
 * - @c M é o fator de divisão da entrada do PLL;
 * - @c N é o fator de multiplicação do VCO;
 * - @c P é o fator de divisão da saída do PLL.
 */
typedef struct
{
    rcc_clock_source_t source; /**< Fonte de entrada do PLL. */
    uint32_t m_factor;         /**< Fator de divisão M. */
    uint32_t n_factor;         /**< Fator de multiplicação N. */
    rcc_pll_p_factor_t p_factor; /**< Fator de divisão P. */
} rcc_pll_config_t;


/**
 * @brief Estrutura de configuração do sistema de clock.
 *
 * Contém os parâmetros utilizados para configurar a fonte de clock
 * do sistema, o PLL e os divisores dos barramentos AHB, APB1 e APB2.
 *
 * Quando o campo @c clock_source é configurado como @c RCC_PLL,
 * os parâmetros presentes em @c pll_conf são utilizados para
 * configurar o PLL.
 */
typedef struct
{
    rcc_clock_source_t clock_source; /**< Fonte de clock do sistema. */

    rcc_pll_config_t pll_conf; /**< Parâmetros de configuração do PLL. */

    rcc_ahb_divider_t ahb_divider;  /**< Divisor do clock do barramento AHB. */
    rcc_apb_divider_t apb1_divider; /**< Divisor do clock do barramento APB1. */
    rcc_apb_divider_t apb2_divider; /**< Divisor do clock do barramento APB2. */
} rcc_config_t;

/**
 * @brief Inicializa e configura o sistema de clock.
 *
 * Configura a fonte de clock do sistema, o PLL quando necessário e os
 * divisores dos barramentos AHB, APB1 e APB2 de acordo com os parâmetros
 * fornecidos.
 *
 * Quando os fatores do PLL não estão configurados, a função utiliza
 * diretamente a fonte de clock especificada em @p config. Quando os
 * fatores M, N e P do PLL são válidos, a função configura o PLL antes
 * de selecioná-lo como fonte do clock do sistema.
 *
 * A função também atualiza a variável global @c SystemCoreClock após
 * as alterações na configuração do clock.
 *
 * @param[in] config
 *     Ponteiro para a estrutura contendo os parâmetros de configuração
 *     do sistema de clock.
 *
 * @return Código indicando o resultado da operação.
 *
 * @retval RCC_OK
 *     Configuração realizada com sucesso.
 *
 * @retval RCC_ERR_INVALID_CLOCK_SOURCE
 *     Fonte de clock inválida para a configuração solicitada.
 *
 * @retval RCC_ERR_INVALID_APB_NUM
 *     Número do barramento APB inválido.
 *
 * @retval RCC_ERR_INVALID_APB_DIVIDER
 *     Divisor de APB inválido.
 *
 * @retval RCC_ERR_INVALID_PLL_P_FACTOR
 *     Fator P do PLL inválido.
 *
 * @note
 *     Quando o PLL é utilizado, a implementação considera o HSE como
 *     fonte de entrada do PLL.
 *
 * @warning
 *     O ponteiro @p config deve apontar para uma estrutura de configuração
 *     válida.
 */
rcc_err_t rcc_init(rcc_config_t *config);

/**
 * @brief Seleciona a fonte de clock do sistema.
 *
 * Habilita a fonte de clock especificada, aguarda sua estabilização,
 * seleciona-a como SYSCLK e aguarda a confirmação da alteração no
 * registrador RCC_CFGR.
 *
 * @param[in] clock_source
 *     Fonte de clock a ser selecionada.
 *
 * @return Código indicando o resultado da operação.
 *
 * @retval RCC_OK
 *     Fonte de clock selecionada com sucesso.
 *
 * @retval RCC_ERR_INVALID_CLOCK_SOURCE
 *     Fonte de clock especificada não é válida.
 *
 * @note
 *     As fontes disponíveis são HSI, HSE e PLL.
 *
 * @note
 *     Quando RCC_PLL é selecionado, o PLL deve ter sido previamente
 *     configurado no registrador RCC_PLLCFGR.
 */
rcc_err_t rcc_set_system_clock_source(rcc_clock_source_t clock_source);


/**
 * @brief Configura os parâmetros do PLL.
 *
 * Configura a fonte de entrada, os fatores de divisão M e P e o fator
 * de multiplicação N do PLL. O PLL é desabilitado antes da alteração
 * do registrador RCC_PLLCFGR e habilitado novamente após a configuração.
 *
 * A frequência do PLL é determinada por:
 *
 * @code
 * PLL_input  = source / M
 * VCO        = PLL_input * N
 * PLL_output = VCO / P
 * @endcode
 *
 * @param[in] pll
 *     Ponteiro para a estrutura contendo os parâmetros de configuração
 *     do PLL.
 *
 * @return Código indicando o resultado da operação.
 *
 * @retval RCC_OK
 *     PLL configurado e estabilizado com sucesso.
 *
 * @retval RCC_ERR_INVALID_CLOCK_SOURCE
 *     Fonte de clock especificada para o PLL é inválida.
 *
 * @retval RCC_ERR_INVALID_PLL_P_FACTOR
 *     Fator de divisão P inválido. São permitidos os valores 2, 4, 6 e 8.
 *
 * @note
 *     O fator P é convertido para o campo PLLP do registrador
 *     RCC_PLLCFGR conforme a codificação definida pelo STM32F411.
 *
 * @note
 *     O PLL é desabilitado antes da alteração de seus parâmetros.
 *
 * @warning
 *     O ponteiro @p pll deve apontar para uma estrutura de configuração
 *     válida.
 */
rcc_err_t rcc_configure_pll(rcc_pll_config_t *pll);


/**
 * @brief Configura o divisor de um barramento APB.
 *
 * Configura o prescaler do barramento APB1 ou APB2 no registrador
 * RCC_CFGR.
 *
 * @param[in] apb_num
 *     Número do barramento APB a ser configurado.
 *     O valor 1 seleciona o APB1 e o valor 2 seleciona o APB2.
 *
 * @param[in] divider
 *     Divisor a ser aplicado ao barramento APB selecionado.
 *
 * @return Código indicando o resultado da operação.
 *
 * @retval RCC_OK
 *     Divisor configurado com sucesso.
 *
 * @retval RCC_ERR_INVALID_APB_NUM
 *     O número do barramento APB não corresponde a APB1 ou APB2.
 *
 * @retval RCC_ERR_INVALID_APB_DIVIDER
 *     Divisor APB inválido.
 *
 * @note
 *     Os divisores disponíveis são 1, 2, 4, 8 e 16.
 */
rcc_err_t rcc_set_apbx_divider(int apb_num, rcc_apb_divider_t divider);

/**
 * @brief Configura o divisor do barramento AHB.
 *
 * Configura o prescaler do barramento AHB no campo HPRE do registrador
 * RCC_CFGR.
 *
 * @param[in] divider
 *     Divisor a ser aplicado ao clock do barramento AHB.
 *
 * @return Código indicando o resultado da operação.
 *
 * @retval RCC_OK
 *     Divisor configurado com sucesso.
 *
 * @retval RCC_ERR_INVALID_APB_DIVIDER
 *     O divisor fornecido não corresponde a um valor válido.
 *
 * @note
 *     Os divisores disponíveis são 1, 2, 4, 8, 16, 64, 128, 256 e 512.
 *
 * @warning
 *     O código de erro utilizado atualmente é
 *     @c RCC_ERR_INVALID_APB_DIVIDER. Recomenda-se criar um código
 *     específico para divisor AHB, como @c RCC_ERR_INVALID_AHB_DIVIDER.
 */
rcc_err_t rcc_set_ahb_divider(rcc_ahb_divider_t divider);

/**
 * @brief Obtém a configuração atual do sistema de clock.
 *
 * Lê os campos SWS, HPRE, PPRE1 e PPRE2 do registrador RCC_CFGR
 * e armazena na estrutura fornecida a fonte de clock selecionada
 * e os divisores configurados para os barramentos AHB, APB1 e APB2.
 *
 * @param[out] status
 *     Ponteiro para a estrutura que receberá o estado atual da
 *     configuração do sistema de clock.
 *
 * @note
 *     O campo @c clock_source é determinado a partir do campo SWS
 *     do registrador RCC_CFGR.
 *
 * @note
 *     Os campos @c ahb_divider, @c apb1_divider e @c apb2_divider
 *     recebem diretamente os valores codificados nos respectivos
 *     campos do registrador RCC_CFGR.
 *
 * @warning
 *     O ponteiro @p status deve apontar para uma estrutura válida.
 */
void rcc_get_clock_status(rcc_clock_status_t *status);

#endif