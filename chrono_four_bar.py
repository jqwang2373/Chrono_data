# -*- coding: utf-8 -*-
"""PyChrono simulation of a four-bar linkage.

This script builds a planar four-bar mechanism and runs a simple
dynamic simulation using the PyChrono physics engine. The mechanism
geometry matches the example used in ``four_bar.py``.

Because visualization libraries are not included, the script prints
angles of each link during the simulation instead of rendering a
window.
"""
import math

# PyChrono is typically imported as ``import pychrono as chrono``.
# The package is large and may require manual installation from
# https://github.com/projectchrono/chrono.
try:
    import pychrono as chrono
except ImportError as exc:  # pragma: no cover - optional dependency
    raise SystemExit("PyChrono is required to run this script") from exc

from four_bar import four_bar_angles

# Link lengths used in the example
L1, L2, L3, L4 = 4.0, 3.0, 5.0, 4.0


def build_four_bar(theta2=math.radians(30)):
    """Create a Chrono system with a four-bar mechanism."""
    system = chrono.ChSystemNSC()

    # Compute initial configuration of the coupler and rocker
    theta3, theta4 = four_bar_angles(L1, L2, L3, L4, theta2)
    xb = L2 * math.cos(theta2)
    yb = L2 * math.sin(theta2)
    xc = xb + L3 * math.cos(theta3)
    yc = yb + L3 * math.sin(theta3)

    # Ground body
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetCollide(False)
    system.Add(ground)

    # Helper for slender links aligned with the x-axis
    def make_link(length, angle, cx, cy):
        body = chrono.ChBodyEasyBox(length, 0.1, 0.1, 1.0, True, True)
        body.SetPos(chrono.ChVectorD(cx, cy, 0))
        body.SetRot(chrono.Q_from_AngZ(angle))
        body.SetCollide(False)
        system.Add(body)
        return body

    # Create links at the computed positions
    crank = make_link(L2, theta2, xb / 2, yb / 2)
    coupler = make_link(L3, theta3, (xb + xc) / 2, (yb + yc) / 2)
    rocker = make_link(L4, theta4, (xc + L1) / 2, yc / 2)

    # Revolute joints
    def rev(body1, body2, x, y):
        joint = chrono.ChLinkLockRevolute()
        joint.Initialize(body1, body2,
                         chrono.ChCoordsysD(chrono.ChVectorD(x, y, 0)))
        system.Add(joint)

    rev(crank, ground, 0, 0)
    rev(coupler, crank, xb, yb)
    rev(rocker, coupler, xc, yc)
    rev(rocker, ground, L1, 0)

    # Drive the crank with a constant angular speed motor
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(crank, ground,
                     chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor.SetSpeedFunction(chrono.ChFunction_Const(math.radians(30)))
    system.Add(motor)

    return system, crank, coupler, rocker


def simulate(duration=1.0, step=1e-3):
    """Run the simulation and print link angles."""
    system, crank, coupler, rocker = build_four_bar()
    while system.GetChTime() < duration:
        system.DoStepDynamics(step)
        t = system.GetChTime()
        a2 = chrono.Q_to_Euler123(crank.GetRot()).z
        a3 = chrono.Q_to_Euler123(coupler.GetRot()).z
        a4 = chrono.Q_to_Euler123(rocker.GetRot()).z
        print(
            f"{t:.3f} {math.degrees(a2):.2f} "
            f"{math.degrees(a3):.2f} {math.degrees(a4):.2f}"
        )


if __name__ == "__main__":  # pragma: no cover - manual run
    simulate()
