#include "sixstep.h"

/* ------------------------------------------------------------------ */
/* Tabela de comutação                                                 */
/* ------------------------------------------------------------------ */

typedef struct
{
    sixstep_phase_t high; /* fase + */
    sixstep_phase_t low;  /* fase - */
    sixstep_phase_t off;  /* fase desligada */
} sixstep_step_t;

/* O vetor de tensão avança 60° a cada setor: -30°, 30°, 90°, 150°, 210°, 270° */
static const sixstep_step_t step_table[SIXSTEP_NUM_SECTORS] = {
    {SIXSTEP_PHASE_A, SIXSTEP_PHASE_B, SIXSTEP_PHASE_C},
    {SIXSTEP_PHASE_A, SIXSTEP_PHASE_C, SIXSTEP_PHASE_B},
    {SIXSTEP_PHASE_B, SIXSTEP_PHASE_C, SIXSTEP_PHASE_A},
    {SIXSTEP_PHASE_B, SIXSTEP_PHASE_A, SIXSTEP_PHASE_C},
    {SIXSTEP_PHASE_C, SIXSTEP_PHASE_A, SIXSTEP_PHASE_B},
    {SIXSTEP_PHASE_C, SIXSTEP_PHASE_B, SIXSTEP_PHASE_A},
};

/* ------------------------------------------------------------------ */
/* Helpers internos                                                    */
/* ------------------------------------------------------------------ */

/** Satura em -1.0 a 1.0; NaN vira 0 (a comparação com NaN é sempre falsa). */
static float clamp_command(float u)
{
    if (u > 1.0f)
        return 1.0f;
    if (u < -1.0f)
        return -1.0f;
    if (!(u == u))
        return 0.0f;
    return u;
}

static void set_phase(sixstep_output_t *out, sixstep_phase_t ph, float duty)
{
    switch (ph)
    {
    case SIXSTEP_PHASE_A:
        out->a = duty;
        break;
    case SIXSTEP_PHASE_B:
        out->b = duty;
        break;
    default:
        out->c = duty;
        break;
    }
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

sixstep_err_t sixstep_init(sixstep_t *ss, const uint8_t hall_by_sector[SIXSTEP_NUM_SECTORS],
                           uint8_t lead)
{
    if (ss == NULL || hall_by_sector == NULL || lead >= SIXSTEP_NUM_SECTORS)
        return SIXSTEP_ERR_INVALID_ARG;

    uint8_t table[8];
    for (uint8_t i = 0; i < 8U; i++)
        table[i] = SIXSTEP_INVALID_SECTOR;

    /* Valida tudo antes de alterar o handle */
    for (uint8_t s = 0; s < SIXSTEP_NUM_SECTORS; s++)
    {
        const uint8_t hall = hall_by_sector[s];

        if (hall == 0U || hall >= 7U)
            return SIXSTEP_ERR_INVALID_HALL;
        if (table[hall] != SIXSTEP_INVALID_SECTOR)
            return SIXSTEP_ERR_INVALID_ARG; /* código repetido */

        table[hall] = (uint8_t)((s + lead) % SIXSTEP_NUM_SECTORS);
    }

    for (uint8_t i = 0; i < 8U; i++)
        ss->hall_to_sector[i] = table[i];
    ss->initialized = true;

    return SIXSTEP_OK;
}

sixstep_err_t sixstep_hall_to_sector(const sixstep_t *ss, uint8_t hall, uint8_t *sector)
{
    if (ss == NULL || sector == NULL)
        return SIXSTEP_ERR_INVALID_ARG;
    if (!ss->initialized)
        return SIXSTEP_ERR_NOT_INITIALIZED;
    if (hall >= 8U)
        return SIXSTEP_ERR_INVALID_HALL;

    const uint8_t s = ss->hall_to_sector[hall];
    if (s == SIXSTEP_INVALID_SECTOR)
        return SIXSTEP_ERR_INVALID_HALL;

    *sector = s;
    return SIXSTEP_OK;
}

sixstep_err_t sixstep_commutate(uint8_t sector, float u, sixstep_output_t *out)
{
    if (out == NULL || sector >= SIXSTEP_NUM_SECTORS)
        return SIXSTEP_ERR_INVALID_ARG;

    const sixstep_step_t *st = &step_table[sector];
    const float half = 0.5f * clamp_command(u);

    /* u < 0 inverte as fases + e - automaticamente */
    set_phase(out, st->high, 0.5f + half);
    set_phase(out, st->low, 0.5f - half);
    set_phase(out, st->off, 0.5f);
    out->floating = st->off;

    return SIXSTEP_OK;
}

uint8_t sixstep_next_sector(uint8_t sector, sixstep_dir_t dir)
{
    sector %= SIXSTEP_NUM_SECTORS;

    if (dir == SIXSTEP_DIR_REVERSE)
        return (sector == 0U) ? (uint8_t)(SIXSTEP_NUM_SECTORS - 1U) : (uint8_t)(sector - 1U);

    return (uint8_t)((sector + 1U) % SIXSTEP_NUM_SECTORS);
}
