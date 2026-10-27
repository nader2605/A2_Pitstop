# Pit Stop scene objects (pit lane, stands, obstacle, light curtain)
# 41013 Industrial Robotics - Lab Assignment 2

import numpy as np
import spatialgeometry as sg
from spatialmath import SE3

from pitstop_config import (
    standHeight, standRadius, standOffset,
    tyre_stand_grey,
    random_obstacle_count
)
from pit_lane import build_pit_lane


class SceneMixin:
    # Each robot's end effector is fitted in its own robot file
    # (KukaKR300.py, ABB_irb140.py, Staubli.py).

    def build_floor_and_stands(self):
        staubli = self.staubli.Robot

        # Pit lane, garages and pit box from the F1 race scene; its asphalt is the floor.
        # (Robots are still checked against the floor plane in collision.py.)
        self.pit_lane = build_pit_lane()

        # Wheel stands
        self.stands = {}
        self.stand_shapes = []

        for name, (dx, dy) in standOffset.items():
            pose = SE3(staubli.base.t[0] + dx, staubli.base.t[1] + dy, standHeight)
            self.stands[name] = pose

            self.stand_shapes.append(
                sg.Cylinder(
                    standRadius,
                    standHeight,
                    pose=pose * SE3(0, 0, -standHeight / 2),
                    color=tyre_stand_grey
                )
            )

    def build_obstacle_and_curtain(self):
        # Collision demo object
        self.obstacle_size = np.array([0.14, 0.14, 0.18])
        self.obstacle_pose = SE3(0, 0, -5)
        self.obstacle_active = False
        self.obstacle_drop_at = None  # transit number the armed obstacle drops on
        self.transit_index = None     # transits done so far; None outside the sequence

        self.obstacle = sg.Cuboid(
            self.obstacle_size,
            pose=self.obstacle_pose,
            color=[0.9, 0.1, 0.1, 1.0]
        )

        # Extra boxes spawned at random places when the obstacle drops
        self.random_boxes = []  # poses of the boxes in play
        self.random_box_shapes = [
            sg.Cuboid(
                self.obstacle_size,
                pose=self.obstacle_pose,
                color=[0.9, 0.45, 0.1, 1.0]
            )
            for _ in range(random_obstacle_count)
        ]

        # Light curtain
        self.light_curtain = [
            sg.Cuboid(
                [0.08, 0.08, 1.2],
                pose=SE3(-1.8, -2.7, 0.6),
                color=[0.9, 0.7, 0.0, 1]
            ),
            sg.Cuboid(
                [0.08, 0.08, 1.2],
                pose=SE3(1.8, -2.7, 0.6),
                color=[0.9, 0.7, 0.0, 1]
            ),
            sg.Cuboid(
                [3.6, 0.02, 0.03],
                pose=SE3(0, -2.7, 0.65),
                color=[1.0, 0.0, 0.0, 0.35]
            )
        ]

    def add_to_env(self):
        if self.env is None:
            return

        objects = [
            *self.pit_lane,
            self.kuka.base_mesh,
            self.abb.base_mesh,
            self.staubli.base_mesh,
            *self.stand_shapes,
            *self.props.values(),
            self.obstacle,
            *self.random_box_shapes,
            *self.light_curtain,
            self.kuka.Robot,
            self.abb.Robot,
            self.staubli.Robot
        ]

        for obj in objects:
            self.env.add(obj)

        self.ur3e.add_to_env(self.env)
