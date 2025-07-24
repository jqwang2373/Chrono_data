import math
from typing import Tuple


def _circle_intersections(x0: float, y0: float, r0: float,
                          x1: float, y1: float, r1: float):
    """Return intersection points of two circles.

    Returns a tuple of two points (x, y) for the intersections. If no
    intersection exists, returns None.
    """
    dx = x1 - x0
    dy = y1 - y0
    d = math.hypot(dx, dy)
    # no solution cases
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h_sq = r0**2 - a**2
    if h_sq < 0:
        # numerical rounding
        h_sq = 0.0
    h = math.sqrt(h_sq)
    xm = x0 + a * dx / d
    ym = y0 + a * dy / d
    rx = -dy * (h / d)
    ry = dx * (h / d)
    p1 = (xm + rx, ym + ry)
    p2 = (xm - rx, ym - ry)
    return p1, p2


def four_bar_angles(l1: float, l2: float, l3: float, l4: float,
                    theta2: float, elbow: str = 'open') -> Tuple[float, float]:
    """Solve a planar four-bar linkage.

    Parameters
    ----------
    l1 : float
        Ground link length.
    l2 : float
        Input crank length.
    l3 : float
        Coupler link length.
    l4 : float
        Output rocker length.
    theta2 : float
        Input crank angle in radians.
    elbow : str, optional
        Configuration, either ``'open'`` or ``'cross'``.

    Returns
    -------
    (theta3, theta4) : Tuple[float, float]
        Coupler and rocker angles in radians.

    Raises
    ------
    ValueError
        If the mechanism cannot be assembled with the given geometry.
    """
    x_b = l2 * math.cos(theta2)
    y_b = l2 * math.sin(theta2)
    intersection = _circle_intersections(x_b, y_b, l3, l1, 0.0, l4)
    if intersection is None:
        raise ValueError('No solution for given geometry and input angle')
    if elbow == 'open':
        x_c, y_c = max(intersection, key=lambda p: p[1])
    else:
        x_c, y_c = min(intersection, key=lambda p: p[1])
    theta3 = math.atan2(y_c - y_b, x_c - x_b)
    theta4 = math.atan2(y_c, x_c - l1)
    return theta3, theta4


def demo():
    """Demonstrate solving a four-bar linkage."""
    # example lengths and input angle
    l1, l2, l3, l4 = 4.0, 3.0, 5.0, 4.0
    theta2 = math.radians(30)
    t3, t4 = four_bar_angles(l1, l2, l3, l4, theta2)
    print(f"Theta3: {math.degrees(t3):.2f} deg")
    print(f"Theta4: {math.degrees(t4):.2f} deg")


if __name__ == '__main__':
    demo()
