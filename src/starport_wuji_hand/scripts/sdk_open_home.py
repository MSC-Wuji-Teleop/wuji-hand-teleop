"""Minimal: ramp the Wuji hand2 to its home pose (logical zero) and let go.

The three lines that matter for smoothness, and why:
  * min-jerk profile -- control is MIT impedance, so a stepped target is a torque impulse
  * the profile's own velocity in field 2 -- damping is kd*(commanded - measured), and sending
    zero asks a moving joint to be stationary (halved tracking error on the bench)
  * 100 Hz, not 1 kHz -- at 1 kHz this hand silently stops driving whole fingers mid-run
"""

import time

from wuji_sdk import DeviceType, JointCommand, SdkManager

KP, KD, EFFORT_LIMIT_A = 9.0, 0.2, 0.6  # kp 8 clean / 15 buzzes; kd optimum ~0.2; device default 1.5 A
DURATION_S, RATE_HZ = 2.0, 100.0  # keep peak speed above the ~0.1 rad/s stick-slip floor
HOLD_S = 20.0  # seconds to hold the open pose before releasing; 0 = let go at once

mgr = SdkManager.instance()
# StopIteration here means the scan found no hand. The scan is Zenoh multicast scouting, which does
# not cross WSL2's NAT -- from a box that can only reach the hand by unicast, skip discovery:
#   hand = mgr.connect(address="192.168.40.111:7447", device_name="open")   # ip:port from a scan
hand = mgr.connect(sn=next(d.sn for d in mgr.scan() if d.device_type == DeviceType.WujiHand2), device_name="open")

# Where the hand actually is. Ramping from a guess would step a hand resting flexed across its
# whole range. nid is a bus-node id with a tactile slot every 5th, so it is not a joint index.
sub = hand.joint_states().subscribe()
frame = None
for _ in range(50):  # recv() returns None until the stream warms up; a guess here would step the hand
    frame = sub.recv()
    if frame is not None and frame.joints:
        break
    time.sleep(0.05)
else:
    raise SystemExit("no joint_states frame arrived; refusing to ramp from an unknown pose")

start = [0.0] * 20
for e in frame.joints:
    bus, node = divmod(e.nid - 1, 5)
    if node < 4:
        start[bus * 4 + node] = e.position

hand.effort_limit().set(EFFORT_LIMIT_A)
hand.mit_params().set((KP, KD))
hand.enable()
time.sleep(0.5)  # enable() is an action, not a landed write; publishing early races it
pub = hand.joint_command().publish()

try:
    for i in range(1, int(DURATION_S * RATE_HZ) + 1):
        t = i / (DURATION_S * RATE_HZ)
        s = t**3 * (10.0 - 15.0 * t + 6.0 * t * t)  # min-jerk: zero velocity AND accel at both ends
        sd = 30.0 * t * t * (1.0 - t) ** 2 / DURATION_S
        # Home is zero, so every joint just scales toward it and cannot leave its envelope.
        pub.send([JointCommand(q * (1.0 - s), -q * sd, 0.0) for q in start])
        time.sleep(1.0 / RATE_HZ)
    # Hold home. Zero velocity because the setpoint has stopped moving -- and no feedforward,
    # which at a standstill would only make the held pose creep.
    for _ in range(int(HOLD_S * RATE_HZ)):
        pub.send([JointCommand(0.0, 0.0, 0.0)] * 20)
        time.sleep(1.0 / RATE_HZ)
finally:
    hand.disable()  # the hand goes limp: unsupported fingers will settle
    hand.disconnect()
