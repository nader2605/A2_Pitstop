# Pit lane scenery: lane surface, markings, pit box, pit wall, garages, gantry, crew
# 41013 Industrial Robotics - Lab Assignment 2
#
# Taken from F1_Race_Pit_Simulation.py (make_circuit and make_pit_garages), keeping
# only the pit lane parts. The coordinates are the original ones from that file;
# the helpers below shift them so the silver car's pit box at (21, -37.2) lands on
# this scene's origin, where the car is parked. Both scenes have the car facing +x
# with the garages on the -y side, which is where the robots stand.

import spatialgeometry as sg
from spatialmath import SE3

# Silver car's pit box in F1_Race_Pit_Simulation.py coordinates
Pit_box_x = 21.0
Pit_y = -37.2

# Colours from F1_Race_Pit_Simulation.py (RGBA, 0-1)
silver = [0.77, 0.80, 0.86, 1]
red = [0.91, 0.10, 0.12, 1]
black = [0.075, 0.080, 0.090, 1]
pit_asphalt = [0.27, 0.29, 0.32, 1]
white = [0.96, 0.96, 0.94, 1]
yellow = [1.00, 0.82, 0.10, 1]
barrier = [0.72, 0.75, 0.77, 1]
dark = [0.11, 0.14, 0.18, 1]
glass = [0.28, 0.47, 0.59, 1]


def place(x, y, z):
    # F1 simulation coordinates -> this scene (pit box centre at the origin)
    return SE3(x - Pit_box_x, y - Pit_y, z)


def box(size, x, y, z, color):
    return sg.Cuboid(size, pose=place(x, y, z), color=color)


def cylinder(radius, length, x, y, z, color):
    return sg.Cylinder(radius=radius, length=length, pose=place(x, y, z), color=color)


def sphere(radius, x, y, z, color):
    return sg.Sphere(radius, pose=place(x, y, z), color=color)


def lane_and_markings():
    shapes = [
        # Pit lane and the wide apron in front of the garages (road top at z=0)
        box([75, 7.5, 0.10], 18, Pit_y, -0.05, pit_asphalt),
        box([77, 7.6, 0.10], 18, -44.2, -0.05, pit_asphalt),
        # Solid white edge line on the garage side
        box([46, 0.12, 0.014], 29, Pit_y - 3.63, 0.014, white),
    ]
    # Yellow speed-limit dashes on the pit wall side
    shapes += [box([3.0, 0.13, 0.014], x, Pit_y + 3.5, 0.016, yellow) for x in range(15, 54, 6)]
    # White dashed centre line
    shapes += [box([2.5, 0.09, 0.014], x, Pit_y, 0.014, white) for x in range(0, 52, 6)]
    return shapes


def pit_box():
    # Yellow outline of the car's box, and a red line marking where the nose stops
    shapes = [box([0.12, 6.8, 0.016], Pit_box_x + dx, Pit_y, 0.017, yellow) for dx in (-4.0, 4.0)]
    shapes += [box([8.0, 0.12, 0.016], Pit_box_x, Pit_y + dy, 0.017, yellow) for dy in (-3.4, 3.4)]
    shapes.append(box([0.12, 3.4, 0.016], Pit_box_x + 3.3, Pit_y, 0.018, red))
    return shapes


def pit_wall():
    # Wall between the pit lane and the race track (the +y side)
    return [
        box([38.0, 0.30, 0.9], 34, -30.8, 0.45, barrier),
        box([38.0, 0.13, 0.14], 34, -30.8, 0.96, dark),
    ]


def garages():
    shapes = []
    # Five garages along the lane, doors facing the lane; the silver team's is fourth
    for index, x in enumerate((-17.0, -3.0, 11.0, 25.0, 39.0)):
        team_color = silver if index == 3 else (red if index == 1 else [0.20, 0.37, 0.70, 1])
        shapes += [
            box([12.6, 11.0, 5.1], x, -51.8, 2.55, [0.70, 0.72, 0.74, 1]),
            box([12.9, 12.2, 0.40], x, -51.9, 5.3, dark),
            # Dark door opening, team panel above, metal door frame
            box([9.8, 0.16, 3.9], x, -46.22, 1.98, dark),
            box([10.0, 0.22, 0.70], x, -46.10, 4.55, team_color),
        ]
        shapes += [box([0.22, 0.22, 3.8], x + dx, -46.04, 1.95, white) for dx in (-4.9, 4.9)]

    # Pit control tower with glass windows
    shapes += [
        box([17.0, 10.0, 8.0], 53, -51.9, 4.0, [0.63, 0.66, 0.69, 1]),
        box([17.4, 10.4, 0.5], 53, -51.9, 8.4, dark),
    ]
    shapes += [box([3.2, 0.13, 1.8], x, -46.80, 6.1, glass) for x in (47, 51, 55, 59)]
    return shapes


def gantry():
    # Red gantry over the pit box, posts outside the lane
    shapes = [box([0.50, 0.50, 6.4], 21, y, 3.20, white) for y in (-45.8, -32.5)]
    shapes += [
        box([1.1, 14.1, 0.52], 21, -39.2, 6.35, red),
        box([1.15, 14.1, 1.0], 21, -39.2, 5.65, dark),
    ]
    return shapes


def equipment_and_crew():
    shapes = []
    # Equipment carts beside the box
    for x, y in ((15.2, -43.0), (27.5, -43.0), (17.0, -31.9)):
        shapes += [
            box([1.0, 0.7, 1.0], x, y, 0.50, red),
            box([0.8, 0.15, 0.25], x, y - 0.45, 0.82, black),
        ]
    # Standing pit crew (static figures)
    for x, y in ((17.5, -43.0), (23.5, -43.0), (15.0, -32.7), (28.0, -35.0)):
        shapes += [
            cylinder(0.23, 1.00, x, y, 0.50, red),
            sphere(0.25, x, y, 1.22, white),
        ]
    return shapes


def build_pit_lane():
    """All pit lane shapes, positioned around this scene's pit box at the origin."""
    return (lane_and_markings() + pit_box() + pit_wall()
            + garages() + gantry() + equipment_and_crew())
