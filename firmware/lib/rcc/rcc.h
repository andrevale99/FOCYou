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
    RCC_OK = 0,                         /**< Operação executada com sucesso. */
    RCC_ERR_INVALID_CLOCK_SOURCE = -1, /**< Fonte de clock inválida. */
    RCC_ERR_INVALID_APB_NUM = -2,      /**< Número do barramento APB inválido. */
    RCC_ERR_INVALID_APB_DIVIDER = -3,  /**< Divisor do barramento APB inválido. */
    RCC_ERR_CLOCK_NOT_RDY = -4,        /**< Fonte de clock não está pronta. */
    RCC_ERR_SET_CLOCK_SOURCE_FAILED = -5 /**< Falha ao selecionar a fonte de clock. */
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
 * @brief Estrutura de configuração do sistema de clock.
 *
 * Contém os parâmetros utilizados para configurar a fonte de clock
 * e os divisores dos barramentos AHB, APB1 e APB2.
 */
typedef struct
{
    rcc_clock_source_t clock_source; /**< Fonte de clock a ser utilizada. */
    rcc_ahb_divider_t ahb_divider;   /**< Divisor do clock do barramento AHB. */
    rcc_apb_divider_t apb1_divider;  /**< Divisor do clock do barramento APB1. */
    rcc_apb_divider_t apb2_divider;  /**< Divisor do clock do barramento APB2. */
} rcc_confg_t;

/**
 * @brief Inicializa e configura o sistema de clock do microcontrolador.
 *
 * Configura a fonte de clock do sistema e os divisores dos barramentos
 * AHB, APB1 e APB2 de acordo com os parâmetros fornecidos na estrutura
 * de configuração.
 *
 * A configuração é realizada na seguinte ordem:
 * 1. Seleção da fonte de clock do sistema;
 * 2. Configuração do divisor do barramento AHB;
 * 3. Configuração do divisor do barramento APB1;
 * 4. Configuração do divisor do barramento APB2.
 *
 * Caso ocorra uma falha em qualquer etapa da configuração, a função
 * interrompe a inicialização e retorna imediatamente o código de erro
 * correspondente.
 *
 * @param[in] config Ponteiro para a estrutura contendo os parâmetros
 *                   de configuração do sistema de clock.
 *
 * @return Código de status da operação.
 * @retval RCC_OK Configuração realizada com sucesso.
 * @retval RCC_ERR_SET_CLOCK_SOURCE_FAILED Falha ao configurar a fonte
 *         de clock selecionada.
 * @retval RCC_ERR_INVALID_CLOCK_SOURCE Fonte de clock inválida.
 * @retval RCC_ERR_INVALID_APB_NUM Número do barramento APB inválido.
 * @retval RCC_ERR_INVALID_APB_DIVIDER Divisor de APB inválido.
 *
 * @warning O ponteiro @p config deve apontar para uma estrutura
 *          previamente inicializada com parâmetros válidos.
 */
rcc_err_t rcc_init(rcc_confg_t *config);

/**
 * @brief Seleciona a fonte de clock do sistema.
 *
 * Habilita a fonte de clock especificada, aguarda sua estabilização
 * e, caso esteja pronta, seleciona-a como fonte de clock do sistema
 * por meio dos bits SW do registrador RCC_CFGR.
 *
 * As fontes disponíveis são:
 * - HSI: oscilador interno de alta velocidade;
 * - HSE: oscilador externo de alta velocidade;
 * - PLL: Phase-Locked Loop.
 *
 * A função verifica os respectivos bits de ready (HSIRDY, HSERDY
 * ou PLLRDY) antes de selecionar a fonte.
 *
 * @param[in] clock_source Fonte de clock que será utilizada pelo
 *                         sistema.
 *
 * @return Código de status da operação.
 * @retval RCC_OK Fonte de clock configurada com sucesso.
 * @retval RCC_ERR_INVALID_CLOCK_SOURCE Fonte de clock inválida.
 * @retval RCC_ERR_SET_CLOCK_SOURCE_FAILED A fonte selecionada não
 *         atingiu o estado de pronta para operação.
 *
 * @note A função utiliza um atraso baseado em laço de software para
 *       aguardar a estabilização da fonte de clock.
 *
 * @warning Antes de selecionar o PLL, seus parâmetros de multiplicação,
 *          divisão e fonte de entrada devem estar previamente configurados.
 */
rcc_err_t rcc_set_clock_source(rcc_clock_source_t clock_source);

/**
 * @brief Configura o divisor de frequência de um barramento APB.
 *
 * Configura o prescaler do barramento APB1 ou APB2 de acordo com o
 * número do barramento especificado e o divisor selecionado.
 *
 * O barramento é selecionado pelo parâmetro @p apb_num:
 * - 1: APB1;
 * - 2: APB2.
 *
 * Os divisores disponíveis são:
 * - DIV_0: divisão por 1;
 * - DIV_2: divisão por 2;
 * - DIV_4: divisão por 4;
 * - DIV_8: divisão por 8;
 * - DIV_16: divisão por 16.
 *
 * @param[in] apb_num Número do barramento APB que será configurado.
 *                    Deve ser 1 ou 2.
 * @param[in] divider Divisor de frequência selecionado para o
 *                    barramento APB.
 *
 * @return Código de status da operação.
 * @retval RCC_OK Divisor configurado com sucesso.
 * @retval RCC_ERR_INVALID_APB_NUM Número do barramento APB inválido.
 * @retval RCC_ERR_INVALID_APB_DIVIDER Divisor de APB inválido.
 *
 * @note A configuração é realizada diretamente no registrador RCC_CFGR,
 *       utilizando os campos PPRE1 para APB1 e PPRE2 para APB2.
 *
 * @warning O divisor selecionado deve respeitar os limites máximos
 *          de frequência dos periféricos conectados ao respectivo
 *          barramento APB.
 */
rcc_err_t rcc_set_apbx_divider(int apb_num, rcc_apb_divider_t divider);

 /**
 * @brief Configura o divisor de frequência do barramento AHB.
 *
 * Configura o prescaler do barramento AHB por meio do campo HPRE do
 * registrador RCC_CFGR.
 *
 * Os divisores disponíveis são:
 * - DIV_0: divisão por 1;
 * - DIV_2: divisão por 2;
 * - DIV_4: divisão por 4;
 * - DIV_8: divisão por 8;
 * - DIV_16: divisão por 16;
 * - DIV_64: divisão por 64;
 * - DIV_128: divisão por 128;
 * - DIV_256: divisão por 256;
 * - DIV_512: divisão por 512.
 *
 * @param[in] divider Divisor de frequência que será aplicado ao
 *                    barramento AHB.
 *
 * @return Código de status da operação.
 * @retval RCC_OK Divisor configurado com sucesso.
 * @retval RCC_ERR_INVALID_AHB_DIVIDER Divisor de AHB inválido.
 *
 * @note O clock do barramento AHB é obtido a partir do clock do sistema
 *       após a aplicação do divisor configurado em HPRE.
 */
rcc_err_t rcc_set_ahb_divider(rcc_ahb_divider_t divider);

/**
 * @brief Obtém o estado atual da configuração do sistema de clock.
 *
 * Lê o registrador RCC_CFGR e obtém a fonte de clock atualmente
 * selecionada para o sistema, bem como os divisores configurados
 * para os barramentos AHB, APB1 e APB2.
 *
 * As informações obtidas são armazenadas na estrutura apontada por
 * @p status.
 *
 * A fonte de clock é identificada por meio dos bits SWS:
 * - RCC_CFGR_SWS_HSI: HSI;
 * - RCC_CFGR_SWS_HSE: HSE;
 * - RCC_CFGR_SWS_PLL: PLL.
 *
 * Os divisores dos barramentos são obtidos a partir dos campos:
 * - HPRE: divisor do AHB;
 * - PPRE1: divisor do APB1;
 * - PPRE2: divisor do APB2.
 *
 * @param[out] status Ponteiro para a estrutura que receberá o estado
 *                    atual da configuração do clock.
 *
 * @note Os valores armazenados em @p status correspondem aos campos
 *       brutos dos respectivos registradores, e não necessariamente
 *       ao valor numérico direto do divisor.
 *
 * @warning O ponteiro @p status deve ser válido. Caso seja NULL,
 *          o acesso aos seus membros causará comportamento indefinido.
 */
void rcc_get_clock_status(rcc_clock_status_t *status);

#endif