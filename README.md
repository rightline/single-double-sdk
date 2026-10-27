# Smart Single Double SDK

C99 helpers for integrating the Rightline Smart Single Double: no heap, no floating point, no I/O.

The attachment's object dictionary holds five positions: frame shift, shift, spread, left split and
right split. This SDK turns a posture (where each group should be) into those targets, derives fork
edges and margins from them, checks reachability before you send anything, and decodes emergency
messages. Bring your own CANopen stack and PDO mapping.

`include/rightline/ssd.h` defines each position, constant and status.

## Build

Add `src/ssd.c` and `include/` to your build, or:

```sh
make        # build/libssd.a
make test   # unit tests under ASan and UBSan
```

## Use

```c
#include "rightline/ssd.h"

rl_ssd_constants c;   /* read 0x2300:1–9 */
rl_ssd_positions now; /* read 0x2100:1–5 */

/* Double mode: left group 30 in left of frame center, right group 28 in right, 6 in splits. */
rl_ssd_posture want = {
    .frame_shift  = RL_SSD_HMU(now.frame_shift),
    .left_center  = RL_SSD_HMU(-7620),
    .right_center = RL_SSD_HMU(7112),
    .left_split   = RL_SSD_HMU(1524),
    .right_split  = RL_SSD_HMU(1524),
};

rl_ssd_positions targets;
rl_ssd_status s = rl_ssd_targets(&want, &c, &targets);
if (s == RL_SSD_OK) {
    /* write targets to 0x2000:1–5 */
} else {
    /* rl_ssd_status_str(s) names the violated constraint */
}

rl_ssd_geometry g;
rl_ssd_geometry_from(&now, &c, &g); /* fork edges, widths and margins of the actual posture */
```

| Function | Does |
|---|---|
| `rl_ssd_targets` | Posture to the closest reachable targets, or the violated constraint. |
| `rl_ssd_posture_from` | Targets or actuals to the exact posture. |
| `rl_ssd_geometry_from` | Fork edges, overall width, group gap, outside and inside margins. |
| `rl_ssd_check` | Reachability of a posture. |
| `rl_ssd_is_single` | Whether both splits are 0. |
| `rl_ssd_emcy_decode` | Error code, register, category and error of an EMCY frame, and which error it is. |
| `rl_ssd_status_str`, `rl_ssd_emcy_str` | Short, stable descriptions. |

## Units

The object dictionary uses machine units (mu, 1/254 in). Postures and geometry use half machine units
(hmu, 1/508 in) so that group centers and fork edges are exact. `RL_SSD_HMU()` converts mu to hmu and
`RL_SSD_MU()` converts hmu to mu.

## Rounding

Targets are whole mu, but a posture in hmu may not be. When it is not, `rl_ssd_targets` rounds the
shift, spread and both splits each down or up, and keeps the reachable combination whose four split
edges (the fork edges facing each group center) are closest to the request by sum of squared error.
Ties go to the smaller spread, then shift, left split, right split. Frame shift rounds down on its own.
An unreachable request is refused, never rounded into range.

For example, groups at −3000 and 3001 mu with splits 0 need a shift of 0.5 mu. The candidates:

| Shift | Spread | Left reached | Right reached | Error |
|---|---|---|---|---|
| 0 | 6001 | −3000.5 | 3000.5 | 1 |
| 0 | 6002 | −3001 | 3001 | 2 |
| 1 | 6001 | −2999.5 | 3001.5 | 1 |
| 1 | 6002 | −3000 | 3002 | 2 |

Two tie at 1, and the smaller shift wins: shift 0, spread 6001. Each group lands ½ mu (0.05 mm) from
the request.
