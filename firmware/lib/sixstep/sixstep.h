/**
 * @file sixstep.h
 * @brief Lógica de comutação six-step (trapezoidal 120°) para motor BLDC.
 *
 * Biblioteca sem acesso a hardware: converte setor + comando de tensão nos
 * duty cycles das três fases. A aplicação lê a posição (Hall ou malha aberta)
 * e envia o resultado ao driver de potência (drv8313, inverter...).
 *
 * Em cada setor uma fase vai para cima (+), outra para baixo (-) e a terceira
 * fica desligada. Com o comando @c u (-1.0 a 1.0):
 *
 * - fase +          : 0.5 + u/2
 * - fase -          : 0.5 - u/2
 * - fase desligada  : 0.5
 *
 * A fase desligada fica no ponto médio porque no SimpleFOC Mini os três ENx
 * estão ligados juntos e não há como deixá-la em alta impedância. Com ENs
 * independentes (ou com o inverter), use sixstep_output_t::floating para
 * desligar essa meia ponte.
 *
 * O sinal de @c u define o sentido do torque: u < 0 troca as fases + e -,
 * o que gira o vetor de tensão em 180°. Assim a saída simétrica do PID
 * (saturation = 1.0) pode ir direto para @c u.
 *
 * Sequência de setores (vetor de tensão avança 60° a cada setor):
 *
 * | Setor | +  | -  | Desligada |
 * |-------|----|----|-----------|
 * |   0   | A  | B  |     C     |
 * |   1   | A  | C  |     B     |
 * |   2   | B  | C  |     A     |
 * |   3   | B  | A  |     C     |
 * |   4   | C  | A  |     B     |
 * |   5   | C  | B  |     A     |
 *
 * Exemplo em malha aberta (sem sensores):
 * @code
 * uint8_t sector = 0;
 * sixstep_output_t out;
 *
 * // a cada período de comutação (ISR de um timer):
 * sector = sixstep_next_sector(sector, SIXSTEP_DIR_FORWARD);
 * sixstep_commutate(sector, 0.2f, &out);
 *
 * const drv8313_duty_t duty = {out.a, out.b, out.c};
 * drv8313_set_duty_cycle(&drv, &duty);
 * @endcode
 *
 * Exemplo com sensores Hall:
 * @code
 * static sixstep_t ss;
 *
 * // códigos Hall lidos em cada setor durante a calibração (ver sixstep_init())
 * const uint8_t hall_by_sector[SIXSTEP_NUM_SECTORS] = {5, 1, 3, 2, 6, 4};
 * sixstep_init(&ss, hall_by_sector, 1);
 *
 * // na ISR de borda dos Hall:
 * uint8_t sector;
 * if (sixstep_hall_to_sector(&ss, hall_code, &sector) == SIXSTEP_OK)
 *     sixstep_commutate(sector, u, &out);
 * @endcode
 */

#ifndef SIXSTEP_H
#define SIXSTEP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief Número de setores de um período elétrico. */
#define SIXSTEP_NUM_SECTORS 6U

/** @brief Valor da tabela Hall para códigos inválidos (000 e 111). */
#define SIXSTEP_INVALID_SECTOR 0xFFU

/** @brief Códigos de retorno da biblioteca. */
typedef enum
{
    SIXSTEP_OK = 0,                   /**< Sucesso. */
    SIXSTEP_ERR_INVALID_ARG = -1,     /**< Ponteiro NULL ou parâmetro inválido. */
    SIXSTEP_ERR_NOT_INITIALIZED = -2, /**< sixstep_init() ainda não foi chamado. */
    SIXSTEP_ERR_INVALID_HALL = -3,    /**< Código Hall inválido (000, 111 ou fora da tabela). */
} sixstep_err_t;

/** @brief Fase do motor. */
typedef enum
{
    SIXSTEP_PHASE_A = 0, /**< Fase A (IN1). */
    SIXSTEP_PHASE_B = 1, /**< Fase B (IN2). */
    SIXSTEP_PHASE_C = 2, /**< Fase C (IN3). */
} sixstep_phase_t;

/** @brief Sentido de avanço dos setores (usado em malha aberta). */
typedef enum
{
    SIXSTEP_DIR_FORWARD = 0, /**< Setor 0 -> 1 -> ... -> 5 -> 0. */
    SIXSTEP_DIR_REVERSE = 1, /**< Setor 5 -> 4 -> ... -> 0 -> 5. */
} sixstep_dir_t;

/** @brief Resultado da comutação. */
typedef struct
{
    float a; /**< Duty da fase A (0.0 a 1.0). */
    float b; /**< Duty da fase B (0.0 a 1.0). */
    float c; /**< Duty da fase C (0.0 a 1.0). */

    sixstep_phase_t floating; /**< Fase desligada neste setor. */
} sixstep_output_t;

/**
 * @brief Handle para a comutação com sensores Hall.
 * @note Os campos são de uso interno; não os modifique diretamente.
 */
typedef struct
{
    uint8_t hall_to_sector[8]; /**< Código Hall (0..7) -> setor a aplicar. */
    bool initialized;          /**< true após sixstep_init() bem-sucedido. */
} sixstep_t;

/**
 * @brief Monta a tabela Hall -> setor.
 *
 * @p hall_by_sector[s] é o código Hall (1 a 6) lido com o rotor alinhado ao
 * setor @c s. Para obtê-la, aplique cada setor em sequência com um @c u
 * baixo e o motor parado, espere o rotor alinhar e leia os três Hall.
 *
 * Com o rotor alinhado ao setor @c s, o torque é máximo aplicando um vetor
 * adiantado de 60° a 120°; por isso, ao ler o código de @c s, a biblioteca
 * aplica o setor <tt>s + lead</tt>. Comece com @p lead = 1, teste 2 e fique
 * com o que der menor corrente (ou maior velocidade) para o mesmo @c u.
 *
 * @param[out] ss             Handle a ser preenchido.
 * @param[in]  hall_by_sector Código Hall de cada setor alinhado.
 * @param[in]  lead           Avanço em setores (0 a 5).
 *
 * @retval SIXSTEP_OK               Sucesso.
 * @retval SIXSTEP_ERR_INVALID_ARG  NULL, @p lead > 5 ou códigos repetidos.
 * @retval SIXSTEP_ERR_INVALID_HALL Algum código igual a 0, 7 ou maior que 7.
 */
sixstep_err_t sixstep_init(sixstep_t *ss, const uint8_t hall_by_sector[SIXSTEP_NUM_SECTORS],
                           uint8_t lead);

/**
 * @brief Converte o código Hall no setor a aplicar.
 *
 * @param[in]  ss     Handle inicializado.
 * @param[in]  hall   Código Hall: bit0 = Hall A, bit1 = Hall B, bit2 = Hall C.
 * @param[out] sector Setor a aplicar (0 a 5).
 *
 * @retval SIXSTEP_OK                  Sucesso.
 * @retval SIXSTEP_ERR_INVALID_ARG     @p sector é NULL.
 * @retval SIXSTEP_ERR_NOT_INITIALIZED Handle não inicializado.
 * @retval SIXSTEP_ERR_INVALID_HALL    Código inválido (sensor desconectado ou ruído).
 *                                     Desligue as saídas nesse caso.
 */
sixstep_err_t sixstep_hall_to_sector(const sixstep_t *ss, uint8_t hall, uint8_t *sector);

/**
 * @brief Calcula os duty cycles das três fases para um setor.
 *
 * @param[in]  sector Setor (0 a 5).
 * @param[in]  u      Comando de tensão normalizado (-1.0 a 1.0); valores fora
 *                    da faixa são saturados e NaN vira 0.
 * @param[out] out    Duty cycles e fase desligada.
 *
 * @retval SIXSTEP_OK              Sucesso.
 * @retval SIXSTEP_ERR_INVALID_ARG @p out é NULL ou @p sector > 5.
 */
sixstep_err_t sixstep_commutate(uint8_t sector, float u, sixstep_output_t *out);

/**
 * @brief Retorna o próximo setor no sentido indicado (malha aberta).
 * @param[in] sector Setor atual (valores > 5 são reduzidos módulo 6).
 * @param[in] dir    Sentido de avanço.
 * @return Próximo setor (0 a 5).
 */
uint8_t sixstep_next_sector(uint8_t sector, sixstep_dir_t dir);

#endif /* SIXSTEP_H */
