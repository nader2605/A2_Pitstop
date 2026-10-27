import swift
import spatialgeometry as sg
from spatialmath import SE3
from pathlib import Path
import numpy as np


class CarModel:
    # Folder containing W07.STL, Wheel.STL and WheelNut.STL
    folder = Path(__file__).resolve().parent

    # Change STL dimensions to metres.
    # Based on the tyre STL file, 39 is the diameter of the tyre and we need the tyre to be 0.66m
    mesh_conv = 0.66 / 39.0

    # Points measured from the car STL file.
    centre = np.array([141.2, 0.0, 500.0])
    frontHub_x = 239.9 #x distance of the front axel from the origin
    rearHub_x = 42.5 #x distance of the rear axel from the origin
    axel_y = 19.5 #y distance of the axels from the ground
    axel_z = {"front": 39.0, "rear": 38.0} #track of the tyres
    jackPoint = np.array([285.0, 7.0, 500.0]) #where the Kuka bot needs to lift the car from, measured from the origin of the STL file

    # Wheel dimensions.
    wheel_centre = 19.5
    nut_seat = 22.0
    wheel_outer_z = 23.0
    # Wheel nut mesh (WheelNut.STL, M12x1.5 19 mm hex): drawn in mm, axis along +Z,
    # base face at z=0, 25 mm long.
    nut_scale = 0.001
    nut_length = 0.025

    # Colours.
    silver = [0.75, 0.77, 0.80, 1.0]
    tyre_black = [0.08, 0.08, 0.08, 1.0]
    new_tyre_red = [0.35, 0.05, 0.05, 1.0]
    nut_red = [0.85, 0.1, 0.1, 1.0]

    # The front-right wheel is changed.
    work_wheel = "FR"
    wheels = {
        "FL": ("front", 1),
        "FR": ("front", -1),
        "RL": ("rear", 1),
        "RR": ("rear", -1)
    }

    # The car tilts slightly when the front is lifted.
    lift_pitch = np.deg2rad(1.2)

    def __init__(self):
        self.pitch = 0.0

        # Offsets of each mesh from its parent frame.
        # The car STL is Y-up. Rotate it into the Swift Z-up world.
        self.body_offset = SE3.Rx(np.pi / 2) * SE3(*(-self.centre * self.mesh_conv))
        self.wheel_offset = SE3(
            -self.wheel_centre * self.mesh_conv,
            -self.wheel_centre * self.mesh_conv,
            0
        )
        # The nut mesh origin is its base face, so it sits straight on the nut seat.
        self.nut_offset = SE3(0, 0, self.nut_seat * self.mesh_conv)
        self.jack_offset = SE3(self.mesh_point(self.jackPoint)) * SE3.Ry(-np.pi / 2)
        self.rear_pivot = self.mesh_point([self.rearHub_x, self.axel_y, self.centre[2]])

        # Meshes: body, four wheels, the work wheel's nut and the replacement wheel.
        self.body = self.make_mesh("W07.STL", self.silver)
        self.wheel_meshes = {name: self.make_mesh("Wheel.STL", self.tyre_black) for name in self.wheels}
        self.nut = sg.Mesh(str(self.folder / "WheelNut.STL"), scale=[self.nut_scale] * 3, color=self.nut_red)
        self.new_wheel = self.make_mesh("Wheel.STL", self.new_tyre_red)

    def make_mesh(self, filename, color):
        s = self.mesh_conv
        return sg.Mesh(str(self.folder / filename), scale=[s, s, s], color=color)

    def mesh_point(self, point):
        # Convert a point from the STL coordinates to metres.
        difference = (np.array(point) - self.centre) * self.mesh_conv
        return np.array([difference[0], -difference[2], difference[1]])

    def hub_pose(self, axle, side):
        # Position of a wheel hub relative to the un-tilted car.
        x = self.frontHub_x if axle == "front" else self.rearHub_x
        z = self.centre[2] - side * self.axel_z[axle]
        position = self.mesh_point([x, self.axel_y, z])
        return SE3(position) * SE3.Rx(-side * np.pi / 2)

    def pose(self):
        # Tilt the car around the rear axle when lifting its front.
        return SE3(self.rear_pivot) * SE3.Ry(-self.pitch) * SE3(-self.rear_pivot)

    def hub_world(self, wheel):
        # Find the wheel hub's position in the world.
        axle, side = self.wheels[wheel]
        return self.pose() * self.hub_pose(axle, side)

    def jack_target(self):
        return self.pose() * self.jack_offset

    def nut_target(self):
        # Pose for a tool pressed against the work wheel's nut.
        target = self.hub_world(self.work_wheel) * self.nut_offset
        return target * SE3(0, 0, self.nut_length) * SE3.Rx(np.pi)

    def wheel_target(self, hub=None):
        # Pose for a gripper against the outer face of a wheel on a hub (or any frame).
        if hub is None:
            hub = self.hub_world(self.work_wheel)
        return hub * SE3(0, 0, self.wheel_outer_z * self.mesh_conv) * SE3.Rx(np.pi)


if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)

    car = CarModel()
    car.body.T = car.body_offset.A
    env.add(car.body)
    for name, mesh in car.wheel_meshes.items():
        mesh.T = (car.hub_world(name) * car.wheel_offset).A
        env.add(mesh)
    env.hold()
