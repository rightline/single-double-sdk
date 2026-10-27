#include "rightline/ssd.h"

#include <stdio.h>
#include <stdlib.h>

static int failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond); \
            failures++;                                                   \
        }                                                                 \
    } while (0)

#define CHECK_EQ(a, b)                                                                     \
    do {                                                                                   \
        long long a_ = (long long)(a), b_ = (long long)(b);                                \
        if (a_ != b_) {                                                                    \
            fprintf(stderr, "%s:%d: %s == %lld, expected %lld\n", __FILE__, __LINE__, #a, a_, b_); \
            failures++;                                                                    \
        }                                                                                  \
    } while (0)

/* Example attachment: 5 in forks; each side 50 in maximum outside edge, 5 in minimum inside edge,
 * 20 in maximum split and 10 in maximum frame shift. */
static const rl_ssd_constants C = {
    .fork_width = 1270,
    .left_max_split = 5080,
    .right_max_split = 5080,
    .left_max_outside_edge = 12700,
    .right_max_outside_edge = 12700,
    .left_min_inside_edge = 1270,
    .right_min_inside_edge = 1270,
    .left_max_frame_shift = 2540,
    .right_max_frame_shift = 2540,
};

static rl_ssd_posture posture_mu(int32_t frame, int32_t lc, int32_t rc, int32_t ls, int32_t rs) {
    rl_ssd_posture p = {RL_SSD_HMU(frame), RL_SSD_HMU(lc), RL_SSD_HMU(rc), RL_SSD_HMU(ls), RL_SSD_HMU(rs)};
    return p;
}

static void test_units(void) {
    CHECK_EQ(RL_SSD_HMU(-3), -6);
    CHECK_EQ(RL_SSD_MU(3), 1);
    CHECK_EQ(RL_SSD_MU(-1), -1);
    CHECK_EQ(RL_SSD_MU(-2), -1);
    CHECK_EQ(RL_SSD_MU(-3), -2);
}

static void test_posture_from(void) {
    const rl_ssd_positions x = {100, 0, 5080, 0, 0};
    rl_ssd_posture p;
    CHECK_EQ(rl_ssd_posture_from(&x, &p), RL_SSD_OK);
    CHECK_EQ(p.frame_shift, 200);
    CHECK_EQ(p.left_center, -5080); /* −2540 mu */
    CHECK_EQ(p.right_center, 5080);
    CHECK(rl_ssd_is_single(&x));
}

static void test_double_exact(void) {
    const rl_ssd_posture p = posture_mu(0, -3540, 3000, 400, 600);
    rl_ssd_positions out;
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_OK);
    CHECK_EQ(out.spread, 6540);
    CHECK_EQ(out.shift, -270);
    CHECK_EQ(out.left_split, 400);
    CHECK_EQ(out.right_split, 600);
    CHECK(!rl_ssd_is_single(&out));
}

static void test_odd_rounds_to_closest(void) {
    /* Lc = −3000 mu, Rc = 3001 mu: shift would be 0.5 mu. */
    const rl_ssd_posture p = posture_mu(0, -3000, 3001, 0, 0);
    rl_ssd_positions out;
    rl_ssd_posture got;
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_OK);
    CHECK_EQ(out.spread, 6001);
    CHECK_EQ(out.shift, 0);
    rl_ssd_posture_from(&out, &got);
    CHECK_EQ(got.left_center, -6001); /* each group off by half an mu, the least possible */
    CHECK_EQ(got.right_center, 6001);
}

static void test_half_mu_split_rounds(void) {
    rl_ssd_posture p = posture_mu(0, -3000, 3000, 100, 0);
    p.left_split += 1; /* 100.5 mu */
    p.frame_shift = 7; /* 3.5 mu */
    rl_ssd_positions out;
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_OK);
    CHECK_EQ(out.left_split, 100); /* tie: smaller */
    CHECK_EQ(out.frame_shift, 3);
}

static void test_refusals(void) {
    rl_ssd_positions out = {1, 2, 3, 4, 5};
    rl_ssd_posture p;

    p = posture_mu(0, -100, 3000, 0, 0); /* left inside edge at +1170 mu */
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_E_LEFT_INSIDE);
    CHECK_EQ(out.frame_shift, 1); /* untouched */

    p = posture_mu(0, -12000, 3000, 0, 0); /* outside edge at −13270 mu */
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_E_LEFT_REACH);

    p = posture_mu(0, -3000, 11000, 0, 1000); /* outside edge at 12770 mu */
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_E_RIGHT_REACH);

    p = posture_mu(0, -3000, 2000, 0, 0); /* right inside edge at 730 mu */
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_E_RIGHT_INSIDE);

    p = posture_mu(0, -3000, 3000, -2, 0);
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_E_SPLIT_NEGATIVE);

    p = posture_mu(40000, -3000, 3000, 0, 0);
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_E_RIGHT_FRAME_SHIFT);

    /* Within a (large) frame shift limit but beyond INTEGER16. */
    rl_ssd_constants wide = C;
    wide.right_max_frame_shift = 50000;
    CHECK_EQ(rl_ssd_targets(&p, &wide, &out), RL_SSD_E_RANGE);

    CHECK_EQ(rl_ssd_targets(NULL, &C, &out), RL_SSD_E_NULL);
    CHECK_EQ(out.frame_shift, 1);
}

static void test_boundaries(void) {
    rl_ssd_positions out;
    rl_ssd_posture p;

    /* Inside edge exactly at the minimum is allowed; one mu closer is not. */
    p = posture_mu(0, -2540, 2540, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_OK);
    p = posture_mu(0, -2539, 2540, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_E_LEFT_INSIDE);
    p = posture_mu(0, -2540, 2539, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_E_RIGHT_INSIDE);

    /* Frame shift exactly at its limits. */
    p = posture_mu(-2540, -2540, 2540, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_OK);
    p = posture_mu(2540, -2540, 2540, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_OK);
    p = posture_mu(-2541, -2540, 2540, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_E_LEFT_FRAME_SHIFT);
    p = posture_mu(2541, -2540, 2540, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_E_RIGHT_FRAME_SHIFT);

    /* Outside edge exactly at the reach limit. */
    p = posture_mu(0, -11430, 3000, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_OK);
    p = posture_mu(0, -11431, 3000, 0, 0);
    CHECK_EQ(rl_ssd_check(&p, &C), RL_SSD_E_LEFT_REACH);

    /* At the inside limits with a half-mu request: rounding must pick a reachable neighbor. */
    p = posture_mu(0, -2540, 2540, 0, 0);
    p.right_center += 1;
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_OK);
    rl_ssd_posture got;
    rl_ssd_posture_from(&out, &got);
    CHECK_EQ(rl_ssd_check(&got, &C), RL_SSD_OK);
}

static void test_single_mode(void) {
    rl_ssd_positions out;
    rl_ssd_posture p = posture_mu(-50, -2880, 3120, 0, 0);
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_OK);
    CHECK_EQ(out.frame_shift, -50);
    CHECK_EQ(out.shift, 120);
    CHECK_EQ(out.spread, 6000);
    CHECK(rl_ssd_is_single(&out));
    p = posture_mu(0, -500, 500, 0, 0);
    CHECK_EQ(rl_ssd_targets(&p, &C, &out), RL_SSD_E_LEFT_INSIDE);
}

static void test_round_trip(void) {
    /* Any reachable dictionary positions survive posture_from -> targets unchanged. */
    uint32_t seed = 12345;
    int checked = 0;
    for (int i = 0; i < 200000; i++) {
        int16_t v[5];
        for (int k = 0; k < 5; k++) {
            seed = seed * 1664525u + 1013904223u;
            v[k] = (int16_t)((int32_t)(seed >> 16) % (k >= 3 ? 4000 : 13000) - (k >= 2 ? 0 : 6500));
        }
        const rl_ssd_positions x = {v[0], v[1], v[2], v[3] < 0 ? (int16_t)-v[3] : v[3], v[4] < 0 ? (int16_t)-v[4] : v[4]};
        rl_ssd_posture p;
        rl_ssd_posture_from(&x, &p);
        if (rl_ssd_check(&p, &C) != RL_SSD_OK) continue;
        rl_ssd_positions back;
        CHECK_EQ(rl_ssd_targets(&p, &C, &back), RL_SSD_OK);
        if (back.frame_shift != x.frame_shift || back.shift != x.shift || back.spread != x.spread ||
            back.left_split != x.left_split || back.right_split != x.right_split) {
            fprintf(stderr, "round trip failed for {%d,%d,%d,%d,%d}\n", x.frame_shift, x.shift, x.spread,
                    x.left_split, x.right_split);
            failures++;
            return;
        }
        checked++;
    }
    CHECK(checked > 1000);
}

static void test_geometry(void) {
    const rl_ssd_positions x = {0, 0, 5080, 500, 0};
    rl_ssd_geometry g;
    CHECK_EQ(rl_ssd_geometry_from(&x, &C, &g), RL_SSD_OK);
    CHECK_EQ(g.left_center, -5080);
    CHECK_EQ(g.fork_edge[RL_SSD_FORK_LEFT_A][0], -8120);
    CHECK_EQ(g.fork_edge[RL_SSD_FORK_LEFT_A][1], -5580);
    CHECK_EQ(g.fork_edge[RL_SSD_FORK_LEFT_B][0], -4580);
    CHECK_EQ(g.fork_edge[RL_SSD_FORK_LEFT_B][1], -2040);
    CHECK_EQ(g.fork_edge[RL_SSD_FORK_RIGHT_A][0], 2540);
    CHECK_EQ(g.fork_edge[RL_SSD_FORK_RIGHT_B][1], 7620);
    CHECK_EQ(g.overall_width, 15740);
    CHECK_EQ(g.group_gap, 4580);
    CHECK_EQ(g.left_outside_margin, 25400 - 8120);
    CHECK_EQ(g.right_outside_margin, 25400 - 7620);
    CHECK_EQ(g.left_inside_margin, -2540 - (-2040));
    CHECK_EQ(g.right_inside_margin, 2540 - 2540);
}

static void test_emcy(void) {
    rl_ssd_emcy e;
    const uint8_t malfunction[8] = {0x00, 0x50, 0x01, 0x01, 0x12, 0x03, 0x34, 0x12};
    CHECK_EQ(rl_ssd_emcy_decode(malfunction, &e), RL_SSD_OK);
    CHECK_EQ(e.kind, RL_SSD_EMCY_POSITION_SENSOR_MALFUNCTION);
    CHECK_EQ(e.error_code, 0x5000);
    CHECK_EQ(e.error_register, 0x01);
    CHECK_EQ(e.category, 0x01);
    CHECK_EQ(e.error, 0x12);

    const uint8_t unachievable[8] = {0x00, 0x63, 0x01, 0x02, 0x01, 0, 0, 0};
    rl_ssd_emcy_decode(unachievable, &e);
    CHECK_EQ(e.kind, RL_SSD_EMCY_POSITION_UNACHIEVABLE);

    const uint8_t reset[8] = {0};
    rl_ssd_emcy_decode(reset, &e);
    CHECK_EQ(e.kind, RL_SSD_EMCY_RESET);

    const uint8_t unknown[8] = {0x00, 0xFF, 0x81, 0x09, 0x01, 0, 0, 0};
    rl_ssd_emcy_decode(unknown, &e);
    CHECK_EQ(e.kind, RL_SSD_EMCY_UNKNOWN);
    CHECK_EQ(e.error_code, 0xFF00);
}

int main(void) {
    test_units();
    test_posture_from();
    test_double_exact();
    test_odd_rounds_to_closest();
    test_half_mu_split_rounds();
    test_refusals();
    test_boundaries();
    test_single_mode();
    test_round_trip();
    test_geometry();
    test_emcy();
    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    puts("all tests passed");
    return EXIT_SUCCESS;
}
