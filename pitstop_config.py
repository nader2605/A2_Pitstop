# Pit Stop constants
# 41013 Industrial Robotics - Lab Assignment 2

dt = 0.02
steps = 50
sequence_transits = 12  # joint moves (move_to / move_joint) in run_sequence
random_obstacle_count = 3  # extra boxes spawned at random places with the dropped obstacle
# Car body and wheels seen from above (x low/high, y low/high) with a margin; no boxes spawn here
car_footprint = ((-2.6, -1.2), (2.9, 1.2))

Kuka_base_posXY = (4.0, -1.5)

standHeight = 0.25
standRadius = 0.3
standOffset = {
    "old": (-0.65, -0.1),
    "new": (-0.1, -0.65)
}

tyre_stand_grey = [0.55, 0.55, 0.6, 1.0]
