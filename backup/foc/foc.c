#include "foc.h"

#define SQRT_3 1.73205080757f

void foc_clarke_transform(const foc_uvw_coord_t *uvw,
                          foc_ab_coord_t *ab)
{
    ab->alpha = uvw->u;
    ab->beta = 1.f/SQRT_3 * (uvw->v - uvw->w);
}

void foc_inverse_clarke_trasform(const foc_ab_coord_t *ab,
                                 foc_uvw_coord_t *uvw)
{
    uvw->u = ab->alpha;
    uvw->v = -0.5f*ab->alpha + SQRT_3/2.f * ab->beta;
    uvw->w = -0.5f*ab->alpha - SQRT_3/2.f * ab->beta;
}

void foc_park_transform(const float theta_e,
                        const foc_ab_coord_t *ab,
                        foc_dq_coord_t *dq)
{
    dq->d = ab->alpha*cos(theta_e) + ab->beta*sin(theta_e);
    dq->q = -ab->alpha*sin(theta_e) + ab->beta*cos(theta_e);
}

void foc_inverse_park_transform(const float theta_e,
                                const foc_dq_coord_t *dq,
                                foc_ab_coord_t *ab)
{
    ab->alpha = dq->d * cos(theta_e) - dq->q * sin(theta_e);
    ab->beta = dq->d * sin(theta_e) + dq->q * cos(theta_e);
}