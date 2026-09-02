# Direct-SDK bench scripts (`sdk_*.py`)

Four standalone scripts that drive the hand through `wuji_sdk` with **no ROS**: no node, no topics,
no guard chain. They exist to answer questions about the hand itself without the driver in the
loop, and to isolate one control variable at a time.

They are deliberately tiny. Each is a straight-line script whose parameters are constants at the
top. None of them takes arguments; edit the constant.

| Script | What it does |
|---|---|
| `sdk_open_home.py` | ramp every joint to home (logical zero) on a min-jerk profile, hold, release |
| `sdk_wave_fingers.py` | wave the fingers one after the next for 20 s (raised cosine per finger) |
| `sdk_rate_ab.py` | A/B the setpoint rate (100 vs 1000 Hz) on identical whole-hand motion |
| `sdk_ff_ab.py` | A/B hard-sign Coulomb feedforward (0 vs 0.174 A) on identical whole-hand motion |

Two more scripts from the original package, `sdk_replay_clip.py` and `sdk_replay_as_driver.py`,
were not vendored: each hard-coded a clip path on another machine. The measurements below that
name them are kept as recorded.

## Running them

`wuji_sdk` must be importable in the shell that runs them (in this repo, the teleop container):

```bash
python3 src/starport_wuji_hand/scripts/sdk_open_home.py
```

Discovery is a UDP probe, not a broadcast the SDK will route for you: the host needs an interface
**on the hand's own subnet**. `scan()` returning 0 devices with the hand plainly powered almost
always means that, not a firewall.

## What these three rules buy, measured

Every script follows the same three rules, and each is worth a number on this hardware:

1. **Never step the target.** Control is MIT impedance, so a stepped setpoint is a torque impulse.
   Every motion path here is min-jerk (zero velocity *and* acceleration at both ends) or a raised
   cosine, and every one starts from the *measured* pose rather than a guess.
2. **Send the setpoint's own velocity.** Damping is `kd × (commanded − measured)`, so sending zero
   asks a moving joint to be stationary. Measured on `l_index_finger_mcp_flex`, 10 s triangle:
   mean tracking error **0.0153 → 0.0076 rad** with velocity supplied (kp 10 → 9 at the same time,
   so the split between the two is not isolated; the driver README attributes nearly all of it to
   the velocity term).
3. **100 Hz, not 1 kHz.** At 1 kHz this hand has been seen to stop driving whole fingers mid-run
   while reporting healthy. `sdk_rate_ab.py` exists to test whether the rate matters otherwise.

## Results on `WH2JA01260810021` (left), 2026-08-27, hand loose on a bench

| Test | Motion | kp | mean err | max err |
|---|---|---|---|---|
| whole hand, gentle, 3 s dwells | 0.3 rad, 0.28 rad/s | 10 | 0.0055 | 0.023 |
| clip, slew-limited as the driver does | peak 2.0 rad/s (capped) | 10 | 0.0086 | 0.062 |
| clip, full speed, unlimited, 100 Hz | peak 3.75 rad/s | 9 | 0.0095 | 0.085 |
| clip, full speed, unlimited, 1000 Hz | peak 4.09 rad/s | 9 | 0.0079 | 0.086 |
| whole hand + hard-sign ff 0.174 A | 0.3 rad, 0.28 rad/s | 10 | 0.0114 | 0.026 |

Reference: the driver README records **0.0119 rad** mean for this same clip replayed through
`hand_node` on the right hand with velocity sent and no friction feedforward.

Two findings worth carrying:

- **Setpoint rate did not matter** for anything but the known freeze. 100 Hz and 1 kHz were
  indistinguishable by ear and within noise on tracking error, on both gentle whole-hand motion
  with dwells and the full-speed clip.
- **Hard-sign feedforward is the worst thing measured**, and the only mechanism found that
  energizes a *stationary* joint. It displaced a held pose by exactly `ff/kp` (−0.017 rad measured
  against 0.0174 predicted) and doubled mean tracking error at gentle speed — worse than the
  full-speed unlimited replay. `calibrate_joint_limits.py` applies ff with a hard sign
  (`ff = args.ff * travel`); `hand_node` ramps it through a velocity deadzone and does not.

## The question this was built to answer, answered on the right hand

On 2026-08-23/24 the bench setup produced **loud constant noise** ("like a drone taking off"), and
on 2026-08-27 it could not be reproduced on `WH2JA01260810021` (left) loose on a bench by varying
setpoint rate, kp, amplitude, speed, held-vs-moving, or hard-sign feedforward. Recovered settings
from those sessions: `oscillate` 51 runs / `step` 4, kp 10 (38) or 3 (16), kd 0.2, speed
0.3 rad/s, ceiling 0.6 A, and a measured tick period of 1.0 ms (1 kHz).

Later the same day it **did** reproduce, on `WH2KA01260810003` (right) bolted to the cell, playing
a station clip at 0.5x through `hand_node`. The cause is the **effort ceiling**, and nothing else:

| kp | ceiling | result |
|---|---|---|
| 9 | 0.6 A | silent |
| 10 | 0.6 A | silent |
| 9 | **1.0 A** | **buzzes** |
| 10 (`hand_node`), full ROS path | 1.0 A → 0.6 A | **buzzes → silent** |

Alternating ONLY the ceiling four times in one session, kp fixed at 9, gave silent / buzz /
silent / buzz. `effort_limit_a` therefore defaulted to a value that buzzes; it is now 0.6 and is a
launch argument, which it previously was not.

What the noise actually is: a limit cycle that needs **exciting** before it sustains. It starts
partway into a clip and then persists through a held setpoint, but arriving at that same pose on a
gentle min-jerk ramp is silent at either ceiling. 1.0 A leaves enough current headroom for the
cycle to sustain; 0.6 A starves it. One burst was heard to stop mid-playback, which fits.

Free-air tracking does not pay for it: mean 0.0086 rad at 0.6 A against 0.0085 at 1.0 A, p95
0.0195 against 0.0203, over the same clip through `sdk_replay_as_driver.py`.

Ruled out by measurement while narrowing this, each on the hardware that buzzes:

- **The hand and its mounting.** `sdk_open_home.py` and `sdk_wave_fingers.py` are both silent on
  the mounted right hand with the arm powered, held and moving.
- **The arm.** Powered and holding position with the hand released is silent, so the arm's own
  servos are not the source.
- **ROS transport.** Commands reach the driver at 100.0 Hz, sd 0.17 ms, with *zero* ticks where
  the guard chain had no command to apply. The depth-1 `_pending` mailbox never starves.
- **The guard chain.** Never clamped and never rate-limited across the run (0/1493 ticks).
- **Feedforward.** Off in the ROS path: `hand.launch.py` sets no `friction_file`.
- **The 10 Hz velocity low-pass, and backward-vs-central velocity.** `sdk_replay_clip.py` and
  `sdk_replay_as_driver.py` apply the same filter, and swapping the derivative for the driver's
  backward delta at 0.6 A stays silent.
- **The trajectory.** Cubic resampling instead of `np.interp`, and capping commanded acceleration
  at 20 rad/s² (from 52.8), both still buzz at 1.0 A. Playback speed does not matter either:
  1.21 rad/s is silent at 0.6 A.

Note the 08-23 sessions ran a **0.6 A** ceiling at **1 kHz**, so that noise may be a different
phenomenon from this one and is not closed by this finding.
