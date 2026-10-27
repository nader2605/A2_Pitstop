# Pit Stop object attachment system
# 41013 Industrial Robotics - Lab Assignment 2


class AttachmentMixin:

    def setup_attachments(self):
        car = self.car

        self.mounts = {
            "car": ("car", car.body_offset),
            "nut": ("hub_" + car.work_wheel, car.nut_offset),
            "new_wheel": ("stand_new", car.wheel_offset)
        }

        for name in car.wheels:
            self.mounts[name] = ("hub_" + name, car.wheel_offset)

        self.props = {
            "car": car.body,
            "nut": car.nut,
            "new_wheel": car.new_wheel,
            **car.wheel_meshes
        }

        self.initial_mounts = dict(self.mounts)

    def parent_pose(self, parent):
        if parent == "car":
            return self.car.pose()
        if parent == "stand_old":
            return self.stands["old"]
        if parent == "stand_new":
            return self.stands["new"]
        if parent == "kuka":
            return self.kuka.Robot.fkine(self.kuka.Robot.q)
        if parent == "abb":
            return self.abb.Robot.fkine(self.abb.Robot.q)
        if parent == "staubli":
            return self.staubli.Robot.fkine(self.staubli.Robot.q)
        if parent.startswith("hub_"):
            return self.car.hub_world(parent[4:])

        raise ValueError("Unknown attachment: " + parent)

    def prop_pose(self, name):
        parent, offset = self.mounts[name]
        return self.parent_pose(parent) * offset

    def attach(self, name, parent):
        world_pose = self.prop_pose(name)
        parent_pose = self.parent_pose(parent)
        self.mounts[name] = (parent, parent_pose.inv() * world_pose)
        self.recovery.record()

    def update_props(self):
        for name, prop in self.props.items():
            prop.T = self.prop_pose(name).A
