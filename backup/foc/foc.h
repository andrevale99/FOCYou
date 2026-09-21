/**
 * @defgroup FOC_Transforms Transformações do FOC
 * @brief Funções responsáveis pelas transformações entre referenciais
 *        trifásico (uvw), estacionário (alpha beta) e síncrono (dq).
 */

#ifndef FOC_H
#define FOC_H

#include <math.h>

// 3-phase uvw coord data type
typedef struct foc_uvw_coord
{
    float u;
    float v;
    float w;
} foc_uvw_coord_t;

// alpha-beta axis static coord data type
typedef struct foc_ab_coord
{
    float alpha;
    float beta;
} foc_ab_coord_t;

// d-q (direct-quadrature) axis rotate coord data type
typedef struct foc_dq_coord
{
    float d;
    float q;
} foc_dq_coord_t;

/**
 * @brief Realiza a transformação de Clarke.
 *
 * Converte grandezas trifásicas no referencial estacionário abc
 * para o referencial ortogonal estacionário αβ.
 *
 * @param[in] uvw Ponteiro para as componentes trifásicas u, v e w.
 * @param[out] ab Ponteiro para a estrutura que armazenará as componentes α e β.
 */
void foc_clarke_transform(const foc_uvw_coord_t *uvw,
                          foc_ab_coord_t *ab);

/**
 * @brief Realiza a transformação inversa de Clarke.
 *
 * Converte grandezas do referencial ortogonal estacionário αβ
 * para o sistema trifásico u-v-w.
 *
 * @param[in] ab Ponteiro para as componentes α e β.
 * @param[out] uvw Ponteiro para a estrutura que armazenará as componentes u, v e w.
 */
void foc_inverse_clarke_trasform(const foc_ab_coord_t *ab,
                                 foc_uvw_coord_t *uvw);

/**
 * @brief Realiza a transformação de Park.
 *
 * Converte grandezas do referencial estacionário αβ para o
 * referencial síncrono d-q utilizando o ângulo elétrico do rotor.
 *
 * @param[in] theta_e Ângulo elétrico do rotor em radianos.
 * @param[in] ab Ponteiro para as componentes α e β.
 * @param[out] dq Ponteiro para a estrutura que armazenará as componentes d e q.
 *
 * @note O eixo d está alinhado com o fluxo do rotor e o eixo q é
 *       ortogonal ao eixo d.
 */
void foc_park_transform(const float theta_e,
                        const foc_ab_coord_t *ab,
                        foc_dq_coord_t *dq);

/**
 * @brief Realiza a transformação inversa de Park.
 *
 * Converte grandezas do referencial síncrono d-q para o
 * referencial estacionário αβ utilizando o ângulo elétrico do rotor.
 *
 * @param[in] theta_e Ângulo elétrico do rotor em radianos.
 * @param[in] dq Ponteiro para as componentes d e q.
 * @param[out] ab Ponteiro para a estrutura que armazenará as componentes α e β.
 *
 * @note Esta transformação é normalmente utilizada antes da
 *       modulação SVPWM ou SPWM em algoritmos FOC.
 */
void foc_inverse_park_transform(const float theta_e,
                                const foc_dq_coord_t *dq,
                                foc_ab_coord_t *ab);

#endif