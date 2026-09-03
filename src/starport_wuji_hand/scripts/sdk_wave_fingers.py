"""Wave the Wuji hand2's fingers, one after the next, for 20 s. Sibling of open_home.py.

Same three smoothness rules: a profiled setpoint (never a step, because MIT impedance turns one
into a torque impulse), the profile's own velocity in field 2 so kd tracks the motion instead of
fighting it, and 100 Hz -- at 1 kHz this hand silently stops driving whole fingers mid-run.

It ramps to home FIRST, then waves from there: starting a wave from wherever the hand was resting
would step every joint at frame one. Each finger curls and returns on a raised cosine, so it
leaves and arrives at zero with zero velocity and the next finger starts from a standstill.
"""

import math
import time

from wuji_sdk import DeviceType, JointCommand, SdkManager

KP, KD, EFFORT_LIMIT_A = 9.0, 0.2, 0.6  # kp 8 clean / 15 buzzes; kd optimum ~0.2; device default 1.5 A
RATE_HZ = 100.0
HOME_S = 3.0  # min-jerk ramp from the resting pose to home before the wave starts
WAVE_S = 20.0  # total waving time
FINGER_S = 1.5  # seconds per finger, out and back
AMPLITUDE = 0.4  # rad, well inside every joint's range (wave_check's default)
HOLD_S = 2.0  # hold home after the wave before releasing
# Peak speed is AMPLITUDE * pi / FINGER_S = 0.84 rad/s here. Keep it under the driver's 2.0 rad/s
# slew budget and above the ~0.1 rad/s where stick-slip makes slow motion rough.

mgr = SdkManager.instance()
hand = mgr.connect(sn=next(d.sn for d in mgr.scan() if d.device_type == DeviceType.WujiHand2), device_name="wave")

sub = hand.joint_states().subscribe()
frame = None
for _ in range(50):  # recv() returns None until the stream warms up
    frame = sub.recv()
    if frame is not None and frame.joints:
        break
    time.sleep(0.05)
else:
    raise SystemExit("no joint_states frame arrived; refusing to ramp from an unknown pose")

start = [0.0] * 20
for e in frame.joints:
    bus, node = divmod(e.nid - 1, 5)  # every 5th nid is a tactile slot, not a joint
    if node < 4:
        start[bus * 4 + node] = e.position

hand.effort_limit().set(EFFORT_LIMIT_A)
hand.mit_params().set((KP, KD))
hand.enable()
time.sleep(0.5)  # enable() is an action, not a landed write; publishing early races it
pub = hand.joint_command().publish()

t0 = time.monotonic()
sent = 0


def tick(pose, vel):
    """Ship one frame and hold the schedule against an absolute clock.

    Absolute, not sleep(1/rate): over 20 s a fixed sleep lets each tick's own duration accumulate
    into the period, so the rate sags and every joint feels the jitter through kd.
    """
    global sent
    pub.send([JointCommand(float(p), float(v), 0.0) for p, v in zip(pose, vel, strict=True)])
    sent += 1
    wait = t0 + sent / RATE_HZ - time.monotonic()
    if wait > 0:
        time.sleep(wait)


try:
    # Ramp to home on a min-jerk profile: zero velocity AND zero acceleration at both ends.
    for i in range(1, int(HOME_S * RATE_HZ) + 1):
        t = i / (HOME_S * RATE_HZ)
        s = t**3 * (10.0 - 15.0 * t + 6.0 * t * t)
        sd = 30.0 * t * t * (1.0 - t) ** 2 / HOME_S
        tick([q * (1.0 - s) for q in start], [-q * sd for q in start])

    # The wave. One finger at a time, in hardware order (thumb, index, middle, ring, pinky), so an
    # observed motion is unambiguously attributable to the finger that was commanded.
    for i in range(int(WAVE_S * RATE_HZ)):
        t = i / RATE_HZ
        finger = int(t // FINGER_S) % 5
        u = (t % FINGER_S) / FINGER_S
        phase = AMPLITUDE * (1.0 - math.cos(2.0 * math.pi * u)) / 2.0
        rate = AMPLITUDE * math.pi * math.sin(2.0 * math.pi * u) / FINGER_S
        pose, vel = [0.0] * 20, [0.0] * 20
        for j in range(4):
            k = finger * 4 + j
            if k % 4 != 1:  # slot 1 of each finger is abduction; curl reads as a wave, splay does not
                pose[k], vel[k] = phase, rate
        tick(pose, vel)

    # Hold home. Zero velocity because the setpoint has stopped moving.
    for _ in range(int(HOLD_S * RATE_HZ)):
        tick([0.0] * 20, [0.0] * 20)
finally:
    hand.disable()  # the hand goes limp: unsupported fingers will settle
    hand.disconnect()
