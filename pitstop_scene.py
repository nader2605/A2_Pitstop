# Pit Stop scene objects (pit lane, stands, obstacle, light curtain)
# 41013 Industrial Robotics - Lab Assignment 2

import numpy as np
from pathlib import Path
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

    def build_emergency_stop_table(self):
        """Visible, stationary E-stop near the UR3e; B remains the actual stop input."""
        # Keep clear of the UR3e pedestal and its sweeping arm.
        # The tabletop is 0.76 m above the floor.
        centre_x, centre_y = 4.40, 1.45
        table_height = 0.76
        top_thickness = 0.05
        grey = [0.37, 0.40, 0.44, 1]
        self.estop_table_shapes = [
            sg.Cuboid(
                [0.58, 0.46, top_thickness],
                pose=SE3(centre_x, centre_y, table_height - top_thickness / 2),
                color=grey
            )
        ]
        for dx in (-0.24, 0.24):
            for dy in (-0.18, 0.18):
                self.estop_table_shapes.append(
                    sg.Cuboid(
                        [0.045, 0.045, table_height - top_thickness],
                        pose=SE3(centre_x + dx, centre_y + dy,
                                 (table_height - top_thickness) / 2),
                        color=grey
                    )
                )

        # The OBJ files are split by material because STL doesn't retain colour
        # and Swift/SpatialGeometry can render separate meshes with explicit colours.
        mesh_dir = Path(__file__).resolve().parent / 'Assets' / 'EmergencyStop'
        button_pose = SE3(centre_x, centre_y, table_height)
        self.estop_button_shapes = [
            sg.Mesh(str(mesh_dir / f'Emergency_stop_{name}.obj'),
                    pose=button_pose, color=color)
            for name, color in (
                ('gray', [0.40, 0.42, 0.46, 1]),
                ('yellow', [1.0, 0.80, 0.04, 1]),
                ('red', [0.90, 0.05, 0.05, 1]),
            )
        ]

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
            *self.estop_table_shapes,
            *self.estop_button_shapes,
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
