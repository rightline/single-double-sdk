/*
 * Rightline Smart Single Double SDK. C99, no heap, no floating point, no I/O.
 *
 * Converts between the five positions in the attachment's object dictionary and a posture
 * (group centers and splits), derives fork edges and margins, checks reachability, and decodes
 * emergency messages. Read and write the object dictionary with your own CANopen stack.
 *
 * Units: the object dictionary uses machine units (mu, 1/254 in). Postures and geometry use half
 * machine units (hmu, 1/508 in), in which every derived value is exact.
 *
 * Coordinates: lateral positions are measured from frame center, negative left and positive
 * right, viewed from the carriage looking at the back of the attachment. Frame shift is frame
 * center relative to carriage center.
 */
#ifndef RIGHTLINE_SSD_H
#define RIGHTLINE_SSD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** mu to hmu. */
#define RL_SSD_HMU(mu) ((int32_t)(mu) * 2)

/** hmu to mu, rounding toward negative infinity. Evaluates `hmu` more than once. */
#define RL_SSD_MU(hmu) ((int32_t)(hmu) >= 0 ? (int32_t)(hmu) / 2 : -((1 - (int32_t)(hmu)) / 2))

typedef enum {
    RL_SSD_OK = 0,
    /** A required pointer was NULL. */
    RL_SSD_E_NULL,
    /** A target does not fit in INTEGER16. */
    RL_SSD_E_RANGE,
    /** A split is negative. */
    RL_SSD_E_SPLIT_NEGATIVE,
    /** The left group's outside edge is beyond the left maximum outside edge (0x2300:4). */
    RL_SSD_E_LEFT_REACH,
    /** The right group's outside edge is beyond the right maximum outside edge (0x2300:5). */
    RL_SSD_E_RIGHT_REACH,
    /** The left group's inside edge is closer to frame center than the left minimum (0x2300:6). */
    RL_SSD_E_LEFT_INSIDE,
    /** The right group's inside edge is closer to frame center than the right minimum (0x2300:7). */
    RL_SSD_E_RIGHT_INSIDE,
    /** The frame shift is further left than the left maximum frame shift (0x2300:8). */
    RL_SSD_E_LEFT_FRAME_SHIFT,
    /** The frame shift is further right than the right maximum frame shift (0x2300:9). */
    RL_SSD_E_RIGHT_FRAME_SHIFT
} rl_ssd_status;

/** Short, stable description of a status, such as "left reach exceeded". */
const char *rl_ssd_status_str(rl_ssd_status status);

/** Object dictionary positions in mu: targets (0x2000) or actuals (0x2100), in sub-index order. */
typedef struct {
    int16_t frame_shift; /* :1  frame center relative to carriage center */
    int16_t shift;       /* :2  midpoint of the group centers relative to frame center */
    int16_t spread;      /* :3  left group center to right group center */
    int16_t left_split;  /* :4  clear gap between the left group's forks */
    int16_t right_split; /* :5  clear gap between the right group's forks */
} rl_ssd_positions;

/** Geometry constants (0x2300) in mu. Valid once the attachment is calibrated. */
typedef struct {
    uint16_t fork_width;             /* :1 */
    uint16_t left_max_split;         /* :2 */
    uint16_t right_max_split;        /* :3 */
    uint16_t left_max_outside_edge;  /* :4  frame center to the furthest left outside edge */
    uint16_t right_max_outside_edge; /* :5  frame center to the furthest right outside edge */
    uint16_t left_min_inside_edge;   /* :6  frame center to the nearest left inside edge */
    uint16_t right_min_inside_edge;  /* :7  frame center to the nearest right inside edge */
    uint16_t left_max_frame_shift;   /* :8  carriage center to the furthest left frame center */
    uint16_t right_max_frame_shift;  /* :9  carriage center to the furthest right frame center */
} rl_ssd_constants;

/** A posture in hmu. Both splits 0 is single mode. */
typedef struct {
    int32_t frame_shift;  /* frame center relative to carriage center */
    int32_t left_center;  /* left group center relative to frame center */
    int32_t right_center; /* right group center relative to frame center */
    int32_t left_split;   /* clear gap between the left group's forks */
    int32_t right_split;  /* clear gap between the right group's forks */
} rl_ssd_posture;

/**
 * Targets for the reachable posture closest to `target`. A posture in hmu does not always land on
 * whole mu; the README explains the rounding. If `target` is not reachable, returns the violated
 * constraint and leaves `out` unchanged. rl_ssd_posture_from(out) gives the posture reached.
 */
rl_ssd_status rl_ssd_targets(const rl_ssd_posture *target, const rl_ssd_constants *constants,
                             rl_ssd_positions *out);

/** The exact posture described by object dictionary positions. */
rl_ssd_status rl_ssd_posture_from(const rl_ssd_positions *positions, rl_ssd_posture *out);

/** RL_SSD_OK if `posture` is reachable, otherwise the first violated constraint. */
rl_ssd_status rl_ssd_check(const rl_ssd_posture *posture, const rl_ssd_constants *constants);

/** True if both splits are 0. */
bool rl_ssd_is_single(const rl_ssd_positions *positions);

/** Forks, left to right. */
enum { RL_SSD_FORK_LEFT_A, RL_SSD_FORK_LEFT_B, RL_SSD_FORK_RIGHT_A, RL_SSD_FORK_RIGHT_B, RL_SSD_FORKS };

/** Derived geometry in hmu, relative to frame center. */
typedef struct {
    int32_t left_center;
    int32_t right_center;
    /** [fork][0] is the fork's left edge, [fork][1] its right edge. */
    int32_t fork_edge[RL_SSD_FORKS][2];
    /** Outside edge to outside edge across all four forks. */
    int32_t overall_width;
    /** Clear gap between the two groups. */
    int32_t group_gap;
    /** Remaining travel before each group's outside edge reaches its maximum; negative beyond it. */
    int32_t left_outside_margin;
    int32_t right_outside_margin;
    /** Remaining travel before each group's inside edge reaches its minimum; negative beyond it. */
    int32_t left_inside_margin;
    int32_t right_inside_margin;
} rl_ssd_geometry;

/** Geometry of object dictionary positions, typically the actuals. */
rl_ssd_status rl_ssd_geometry_from(const rl_ssd_positions *positions, const rl_ssd_constants *constants,
                                   rl_ssd_geometry *out);

/* Emergency messages */

typedef enum {
    /** Error code 0x0000: a previously reported error has cleared. */
    RL_SSD_EMCY_RESET = 0,
    RL_SSD_EMCY_POSITION_UNACHIEVABLE,
    RL_SSD_EMCY_OVER_HYDRAULIC_FORCE,
    RL_SSD_EMCY_COIL_DRIVE,
    RL_SSD_EMCY_OVER_CURRENT,
    RL_SSD_EMCY_OVER_TEMPERATURE,
    RL_SSD_EMCY_POSITION_SENSOR_DISCONNECTED,
    RL_SSD_EMCY_POSITION_SENSOR_MALFUNCTION,
    RL_SSD_EMCY_PRESSURE_OUT_OF_RANGE,
    RL_SSD_EMCY_PRESSURE_STUCK,
    RL_SSD_EMCY_CURRENT_SENSOR_OUT_OF_RANGE,
    RL_SSD_EMCY_CURRENT_SENSOR_STUCK,
    /** An error this SDK version does not know. The raw fields are still valid. */
    RL_SSD_EMCY_UNKNOWN
} rl_ssd_emcy_kind;

typedef struct {
    rl_ssd_emcy_kind kind;
    uint16_t error_code;    /* bytes 0-1, CiA 301 */
    uint8_t error_register; /* byte 2, 0x1001 */
    uint8_t category;       /* byte 3 */
    uint8_t error;          /* byte 4 */
} rl_ssd_emcy;

/** Decode the 8 data bytes of an EMCY frame. Bytes 5-7 are private and ignored. */
rl_ssd_status rl_ssd_emcy_decode(const uint8_t data[8], rl_ssd_emcy *out);

/** Short, stable name for an emergency kind, such as "position unachievable". */
const char *rl_ssd_emcy_str(rl_ssd_emcy_kind kind);

#ifdef __cplusplus
}
#endif

#endif /* RIGHTLINE_SSD_H */
