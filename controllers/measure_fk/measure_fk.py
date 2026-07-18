#!/usr/bin/env python3
"""Measure slider-to-suction-plane Z mapping."""
import os
from controller import Supervisor

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
OUT = os.path.join(os.path.dirname(__file__), "z_measure.txt")
open(OUT,"w").close()

m = robot.getFromDef("EE_MARKER")
if not m:
    with open(OUT,"a") as f: f.write("FATAL:no marker\n")
    while robot.step(dt)!=-1: pass

motors = [robot.getDevice("0x10::slider"),
          robot.getDevice("0x11"), robot.getDevice("0x12"), robot.getDevice("0x13")]
motors[0].setVelocity(0.05)
for i in (1,2,3): motors[i].setVelocity(2.0)

def step(t):
    for _ in range(int(t*1000/dt)): robot.step(dt)

def measure(label, s):
    motors[0].setPosition(s)
    for i in (1,2,3): motors[i].setPosition(0.0)
    step(3.0)
    p = m.getPosition()
    msg = f"{label}|s={s:.4f}|x={p[0]:.6f}|y={p[1]:.6f}|z={p[2]:.6f}"
    print(msg)
    with open(OUT,"a") as f: f.write(msg+"\n")

# Test slider positions from 0 down to -0.45
for s in [0.0, -0.05, -0.10, -0.15, -0.20, -0.25, -0.30, -0.35, -0.40]:
    step(1.0)
    measure(f"S{s:.2f}", s)

with open(OUT,"a") as f: f.write("DONE\n")
while robot.step(dt)!=-1: pass
