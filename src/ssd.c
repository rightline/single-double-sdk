#include "rightline/ssd.h"

#include <stddef.h>

const char *rl_ssd_status_str(rl_ssd_status status) {
    switch (status) {
    case RL_SSD_OK: return "ok";
    case RL_SSD_E_NULL: return "null argument";
    case RL_SSD_E_RANGE: return "out of INTEGER16 range";
    case RL_SSD_E_SPLIT_NEGATIVE: return "negative split";
    case RL_SSD_E_LEFT_REACH: return "left reach exceeded";
    case RL_SSD_E_RIGHT_REACH: return "right reach exceeded";
    case RL_SSD_E_LEFT_INSIDE: return "left inside limit exceeded";
    case RL_SSD_E_RIGHT_INSIDE: return "right inside limit exceeded";
    case RL_SSD_E_LEFT_FRAME_SHIFT: return "left frame shift exceeded";
    case RL_SSD_E_RIGHT_FRAME_SHIFT: return "right frame shift exceeded";
    }

    return "unknown status";
}

static int64_t floor_div(int64_t a, int64_t b) {
    const int64_t q = a / b;

    return (a % b != 0 && (a < 0) != (b < 0)) ? q - 1 : q;
}

static bool fits_i16(int64_t v) {
    return v >= INT16_MIN && v <= INT16_MAX;
}

rl_ssd_status rl_ssd_check(const rl_ssd_posture *p, const rl_ssd_constants *c) {
    if (p == NULL || c == NULL) return RL_SSD_E_NULL;
    if (p->left_split < 0 || p->right_split < 0) return RL_SSD_E_SPLIT_NEGATIVE;

    // Quarter mu, so that a center plus half a split stays whole.
    const int64_t w = 4 * (int64_t)c->fork_width;
    const int64_t lc = 2 * (int64_t)p->left_center, rc = 2 * (int64_t)p->right_center;
    const int64_t ls = p->left_split, rs = p->right_split;

    if (lc - ls - w < -4 * (int64_t)c->left_max_outside_edge) return RL_SSD_E_LEFT_REACH;
    if (rc + rs + w > 4 * (int64_t)c->right_max_outside_edge) return RL_SSD_E_RIGHT_REACH;
    if (lc + ls + w > -4 * (int64_t)c->left_min_inside_edge) return RL_SSD_E_LEFT_INSIDE;
    if (rc - rs - w < 4 * (int64_t)c->right_min_inside_edge) return RL_SSD_E_RIGHT_INSIDE;
    if (p->frame_shift < -RL_SSD_HMU(c->left_max_frame_shift)) return RL_SSD_E_LEFT_FRAME_SHIFT;
    if (p->frame_shift > RL_SSD_HMU(c->right_max_frame_shift)) return RL_SSD_E_RIGHT_FRAME_SHIFT;

    return RL_SSD_OK;
}

rl_ssd_status rl_ssd_posture_from(const rl_ssd_positions *x, rl_ssd_posture *out) {
    if (x == NULL || out == NULL) return RL_SSD_E_NULL;

    out->frame_shift = RL_SSD_HMU(x->frame_shift);
    out->left_center = RL_SSD_HMU(x->shift) - x->spread;
    out->right_center = RL_SSD_HMU(x->shift) + x->spread;
    out->left_split = RL_SSD_HMU(x->left_split);
    out->right_split = RL_SSD_HMU(x->right_split);

    return RL_SSD_OK;
}

/* Sum of squared errors of the four split edges (the fork edges facing each group center), in quarter mu. */
static int64_t split_edge_error(const rl_ssd_posture *want, const rl_ssd_posture *got) {
    const int64_t dl = 2 * ((int64_t)got->left_center - want->left_center);
    const int64_t dr = 2 * ((int64_t)got->right_center - want->right_center);
    const int64_t dls = (int64_t)got->left_split - want->left_split;
    const int64_t drs = (int64_t)got->right_split - want->right_split;
    const int64_t e[4] = {dl - dls, dl + dls, dr - drs, dr + drs};

    return e[0] * e[0] + e[1] * e[1] + e[2] * e[2] + e[3] * e[3];
}

rl_ssd_status rl_ssd_targets(const rl_ssd_posture *target, const rl_ssd_constants *constants,
                             rl_ssd_positions *out) {
    if (target == NULL || constants == NULL || out == NULL) return RL_SSD_E_NULL;

    rl_ssd_status status = rl_ssd_check(target, constants);
    if (status != RL_SSD_OK) return status;

    // Floors of the exact targets in mu; each candidate adds 0 or 1 to each.
    const int64_t sp0 = floor_div((int64_t)target->right_center - target->left_center, 2);
    const int64_t sh0 = floor_div((int64_t)target->left_center + target->right_center, 4);
    const int64_t ls0 = floor_div(target->left_split, 2);
    const int64_t rs0 = floor_div(target->right_split, 2);
    const int64_t fs = floor_div(target->frame_shift, 2);
    if (!fits_i16(fs)) return RL_SSD_E_RANGE;

    // Ascending order with a strict comparison breaks ties toward the smaller spread, shift,
    // left split, then right split.
    status = RL_SSD_E_RANGE;
    int64_t best_error = INT64_MAX;
    for (int i = 0; i < 16; i++) {
        const int64_t sp = sp0 + (i >> 3 & 1), sh = sh0 + (i >> 2 & 1);
        const int64_t ls = ls0 + (i >> 1 & 1), rs = rs0 + (i & 1);
        if (!fits_i16(sp) || !fits_i16(sh) || !fits_i16(ls) || !fits_i16(rs)) continue;

        const rl_ssd_positions cand = {(int16_t)fs, (int16_t)sh, (int16_t)sp, (int16_t)ls, (int16_t)rs};
        rl_ssd_posture reached;
        rl_ssd_posture_from(&cand, &reached);

        const rl_ssd_status s = rl_ssd_check(&reached, constants);
        if (s != RL_SSD_OK) {
            if (status == RL_SSD_E_RANGE) status = s;
            continue;
        }

        const int64_t error = split_edge_error(target, &reached);
        if (error < best_error) {
            best_error = error;
            *out = cand;
            status = RL_SSD_OK;
        }
    }

    return status;
}

bool rl_ssd_is_single(const rl_ssd_positions *x) {
    return x != NULL && x->left_split == 0 && x->right_split == 0;
}

rl_ssd_status rl_ssd_geometry_from(const rl_ssd_positions *x, const rl_ssd_constants *c, rl_ssd_geometry *out) {
    if (x == NULL || c == NULL || out == NULL) return RL_SSD_E_NULL;

    rl_ssd_posture p;
    rl_ssd_posture_from(x, &p);

    const int32_t w = RL_SSD_HMU(c->fork_width);
    const int32_t lh = x->left_split, rh = x->right_split;
    int32_t (*e)[2] = out->fork_edge;

    out->left_center = p.left_center;
    out->right_center = p.right_center;
    e[RL_SSD_FORK_LEFT_A][0] = p.left_center - lh - w;
    e[RL_SSD_FORK_LEFT_A][1] = p.left_center - lh;
    e[RL_SSD_FORK_LEFT_B][0] = p.left_center + lh;
    e[RL_SSD_FORK_LEFT_B][1] = p.left_center + lh + w;
    e[RL_SSD_FORK_RIGHT_A][0] = p.right_center - rh - w;
    e[RL_SSD_FORK_RIGHT_A][1] = p.right_center - rh;
    e[RL_SSD_FORK_RIGHT_B][0] = p.right_center + rh;
    e[RL_SSD_FORK_RIGHT_B][1] = p.right_center + rh + w;

    out->overall_width = e[RL_SSD_FORK_RIGHT_B][1] - e[RL_SSD_FORK_LEFT_A][0];
    out->group_gap = e[RL_SSD_FORK_RIGHT_A][0] - e[RL_SSD_FORK_LEFT_B][1];
    out->left_outside_margin = e[RL_SSD_FORK_LEFT_A][0] + RL_SSD_HMU(c->left_max_outside_edge);
    out->right_outside_margin = RL_SSD_HMU(c->right_max_outside_edge) - e[RL_SSD_FORK_RIGHT_B][1];
    out->left_inside_margin = -RL_SSD_HMU(c->left_min_inside_edge) - e[RL_SSD_FORK_LEFT_B][1];
    out->right_inside_margin = e[RL_SSD_FORK_RIGHT_A][0] - RL_SSD_HMU(c->right_min_inside_edge);

    return RL_SSD_OK;
}

static const struct {
    uint8_t category, error;
    rl_ssd_emcy_kind kind;
} EMCY_KINDS[] = {
    {0x01, 0x11, RL_SSD_EMCY_POSITION_SENSOR_DISCONNECTED},
    {0x01, 0x12, RL_SSD_EMCY_POSITION_SENSOR_MALFUNCTION},
    {0x01, 0x21, RL_SSD_EMCY_PRESSURE_OUT_OF_RANGE},
    {0x01, 0x22, RL_SSD_EMCY_PRESSURE_STUCK},
    {0x01, 0x31, RL_SSD_EMCY_CURRENT_SENSOR_OUT_OF_RANGE},
    {0x01, 0x32, RL_SSD_EMCY_CURRENT_SENSOR_STUCK},
    {0x02, 0x01, RL_SSD_EMCY_POSITION_UNACHIEVABLE},
    {0x02, 0x02, RL_SSD_EMCY_OVER_HYDRAULIC_FORCE},
    {0x03, 0x01, RL_SSD_EMCY_COIL_DRIVE},
    {0x04, 0x01, RL_SSD_EMCY_OVER_CURRENT},
    {0x05, 0x01, RL_SSD_EMCY_OVER_TEMPERATURE},
};

rl_ssd_status rl_ssd_emcy_decode(const uint8_t data[8], rl_ssd_emcy *out) {
    if (data == NULL || out == NULL) return RL_SSD_E_NULL;

    out->error_code = (uint16_t)(data[0] | data[1] << 8);
    out->error_register = data[2];
    out->category = data[3];
    out->error = data[4];

    out->kind = RL_SSD_EMCY_UNKNOWN;
    if (out->error_code == 0x0000) {
        out->kind = RL_SSD_EMCY_RESET;
        return RL_SSD_OK;
    }

    for (size_t i = 0; i < sizeof EMCY_KINDS / sizeof EMCY_KINDS[0]; i++) {
        if (EMCY_KINDS[i].category == out->category && EMCY_KINDS[i].error == out->error) {
            out->kind = EMCY_KINDS[i].kind;
            break;
        }
    }

    return RL_SSD_OK;
}

const char *rl_ssd_emcy_str(rl_ssd_emcy_kind kind) {
    switch (kind) {
    case RL_SSD_EMCY_RESET: return "error reset";
    case RL_SSD_EMCY_POSITION_UNACHIEVABLE: return "position unachievable";
    case RL_SSD_EMCY_OVER_HYDRAULIC_FORCE: return "over hydraulic force limit";
    case RL_SSD_EMCY_COIL_DRIVE: return "coil drive fault";
    case RL_SSD_EMCY_OVER_CURRENT: return "over current limit";
    case RL_SSD_EMCY_OVER_TEMPERATURE: return "over temperature limit";
    case RL_SSD_EMCY_POSITION_SENSOR_DISCONNECTED: return "position sensor disconnected";
    case RL_SSD_EMCY_POSITION_SENSOR_MALFUNCTION: return "position sensor malfunction";
    case RL_SSD_EMCY_PRESSURE_OUT_OF_RANGE: return "pressure sensor out of range";
    case RL_SSD_EMCY_PRESSURE_STUCK: return "pressure sensor stuck";
    case RL_SSD_EMCY_CURRENT_SENSOR_OUT_OF_RANGE: return "current sensor out of range";
    case RL_SSD_EMCY_CURRENT_SENSOR_STUCK: return "current sensor stuck";
    case RL_SSD_EMCY_UNKNOWN: break;
    }

    return "unknown";
}
