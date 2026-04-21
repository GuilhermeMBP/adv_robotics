

from ar_py_utils.LocalFrameWorldFrameTransformations import Point2D

beacons_wpos = \
    [Point2D(7.82, 0.0),  # 1
     Point2D(7.82, 4.0),  # 2
     Point2D(7.81, 7.76),  # 3
     Point2D(6.26, 7.76),  # 4
     Point2D(5.99, 7.76),  # 5
     Point2D(2.57, 7.76),  # 6
     Point2D(2.32, 7.76),  # 7
     Point2D(-1.07, 7.76),  # 8
     Point2D(-1.34, 7.76),  # 9
     Point2D(-4.77, 7.76),  # 10
     Point2D(-5.02, 7.76),  # 11
     Point2D(-7.81, 7.76),  # 12
     Point2D(-7.82, 4.0),  # 13
     Point2D(-7.82, 0.0),  # 14
     Point2D(-7.82, -4.0),  # 15
     Point2D(-7.81, -7.86),  # 16
     Point2D(-6.16, -7.86),  # 17
     Point2D(-5.93, -7.27),  # 18
     Point2D(-3.80, -7.27),  # 19
     Point2D(-3.56, -7.27),  # 20
     Point2D(-1.42, -7.27),  # 21
     Point2D(-1.17, -7.27),  # 22
     Point2D(1.01, -7.27),  # 23
     Point2D(1.25, -7.27),  # 24
     Point2D(3.39, -7.27), # 25
     Point2D(3.64, -7.27),  # 26
     Point2D(5.92, -7.27),  # 27
     Point2D(6.16, -7.86),  # 28
     Point2D(7.81, -7.86),  # 29
     Point2D(7.82, -4.0),  # 30
     Point2D(-4.98, 3.08),  # 31
     Point2D(-1.20, 4.78),  # 32
     Point2D(2.5, 3.08),  # 33
     Point2D(5.38, 4.78),  # 34
     Point2D(-5.92, 2.38),  # 35
     Point2D(-3.79, 2.38),  # 36
     Point2D(-3.57, 2.38),  # 37
     Point2D(-1.41, 2.38),  # 38
     Point2D(-1.17, 2.38),  # 39
     Point2D(1.05, 2.38),  # 40
     Point2D(1.28, 2.38),  # 41
     Point2D(3.41, 2.38),  # 42
     Point2D(3.65, 2.38),  # 43
     Point2D(5.93, 2.38),  # 44
     Point2D(-4.84, -2.62),  # 45
     Point2D(-4.13, -3.36),  # 46
     Point2D(-4.84, -4.07),  # 47
     Point2D(-5.56, -3.36),  # 48
     Point2D(-0.05, -1.09),  # 49
     Point2D(0.64, -1.81),  # 50
     Point2D(-0.05, -2.52),  # 51
     Point2D(-0.77, -1.80),  # 52
     Point2D(4.73, -2.38),  # 53
     Point2D(5.43, -3.07),  # 54
     Point2D(4.73, -3.83),  # 55
     Point2D(4.04, -3.07)]  # 56

# Print all the beacons' values.
for i, pos in enumerate(beacons_wpos):
    print(f'Beacon {i+1}: ({pos.x}, {pos.y}) [m]')
