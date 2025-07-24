import math
import unittest
from four_bar import four_bar_angles


class FourBarTest(unittest.TestCase):
    def test_geometry_closure(self):
        l1, l2, l3, l4 = 4.0, 3.0, 5.0, 4.0
        theta2 = math.radians(30)
        theta3, theta4 = four_bar_angles(l1, l2, l3, l4, theta2)

        # position of B
        x_b = l2 * math.cos(theta2)
        y_b = l2 * math.sin(theta2)

        # position of C from coupler
        x_c1 = x_b + l3 * math.cos(theta3)
        y_c1 = y_b + l3 * math.sin(theta3)

        # position of C from rocker
        x_c2 = l1 + l4 * math.cos(theta4)
        y_c2 = l4 * math.sin(theta4)

        self.assertAlmostEqual(x_c1, x_c2, places=7)
        self.assertAlmostEqual(y_c1, y_c2, places=7)


if __name__ == '__main__':
    unittest.main()
