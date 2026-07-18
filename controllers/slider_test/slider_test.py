#!/usr/bin/env python3
"""Bare minimum: just move the slider. Writes log to file."""
import os
from controller import Robot
LOG = os.path.join(os.path.dirname(__file__), "slider_log.txt")
def log(msg):
    print(msg)
    with open(LOG, "a") as f: f.write(msg + "\n")

open(LOG, "w").close()
r = Robot()
dt = int(r.getBasicTimeStep())
log(f"dt={dt}")

lin  = r.getDevice("0x10::slider")
rot  = r.getDevice("0x10::motor")
log(f"linear={'OK' if lin else 'MISSING'}, rotary={'OK' if rot else 'MISSING'}")

if lin: lin.setVelocity(0.02)  # slow for visibility
if rot: rot.setVelocity(10.0)

for phase in range(5):
    target_m = -0.050 * (phase + 1)  # -0.05, -0.10, -0.15, -0.20, -0.25
    target_r = target_m * 625.0
    log(f"Phase{phase}: lin={target_m:.3f}m rot={target_r:.0f}rad")
    if lin: lin.setPosition(target_m)
    if rot: rot.setPosition(target_r)
    for i in range(300):  # 1.2s per phase at dt=4
        r.step(dt)

log("DONE")
while r.step(dt) != -1: pass
