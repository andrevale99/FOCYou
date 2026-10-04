#include "foc.h"

#define SQRT_3 1.73205080757f

void foc_clarke_transform(const foc_uvw_coord_t *uvw,
                          foc_ab_coord_t *ab)
{
    ab->alpha = uvw->u;
    ab->beta = 1.f/SQRT_3 * (uvw->v - uvw->w);
}

void foc_inverse_clarke_transform(const foc_ab_coord_t *ab,
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
    const float sin_theta = sinf(theta_e);
    const float cos_theta = cosf(theta_e);

    dq->d = ab->alpha*cos_theta + ab->beta*sin_theta;
    dq->q = -ab->alpha*sin_theta + ab->beta*cos_theta;
}

void foc_inverse_park_transform(const float theta_e,
                                const foc_dq_coord_t *dq,
                                foc_ab_coord_t *ab)
{
    const float sin_theta = sinf(theta_e);
    const float cos_theta = cosf(theta_e);

    ab->alpha = dq->d * cos_theta - dq->q * sin_theta;
    ab->beta = dq->d * sin_theta + dq->q * cos_theta;
}