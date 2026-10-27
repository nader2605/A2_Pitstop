"""Collision checking using the Lab 5 (Question 2) method.

Each robot link is treated as a line segment between consecutive joint
frames. Obstacles are triangle meshes (a rectangular prism, as in Lab 5).
A link collides when its segment crosses one of the prism's face planes
(line-plane intersection, Lab 3/5) at a point inside that face's triangle.
The floor is a single plane checked with the same line-plane test.

Avoidance: a blocked joint move searches for a random joint-space waypoint
that goes around the obstacle (find_detour). Any other move is stop on
contact: the robot is halted before the step that would collide (see
MotionMixin.follow_path).
"""
from itertools import combinations
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3


def line_plane_intersection(plane_normal, point_on_plane, point1, point2):
    """Same as ir_support.line_plane_intersection (Labs 3, 5 and 6).
    check: 0 = parallel, 1 = intersects within the segment,
           2 = segment lies in the plane, 3 = intersects outside the segment."""
    u = point2 - point1
    w = point1 - point_on_plane
    D = np.dot(plane_normal, u)
    N = -np.dot(plane_normal, w)
    if abs(D) < 1e-12:
        return None, 2 if abs(N) < 1e-12 else 0
    s = N / D
    intersection_point = point1 + s * u
    return intersection_point, 1 if 0 <= s <= 1 else 3


def is_intersection_point_inside_triangle(intersect_p, triangle_verts):
    # Lab 5 Question 2 solution
    u = triangle_verts[1, :] - triangle_verts[0, :]
    v = triangle_verts[2, :] - triangle_verts[0, :]

    uu = np.dot(u, u)
    uv = np.dot(u, v)
    vv = np.dot(v, v)

    w = intersect_p - triangle_verts[0, :]
    wu = np.dot(w, u)
    wv = np.dot(w, v)

    D = uv * uv - uu * vv

    s = (uv * wv - vv * wu) / D
    if s < 0.0 or s > 1.0:
        return False

    t = (uv * wu - uu * wv) / D
    if t < 0.0 or (s + t) > 1.0:
        return False

    return True


def rectangular_prism(size, pose):
    """Vertices, faces and face normals of a box, like ir_support.RectangularPrism
    in Lab 5, but placed with a full pose so it can be rotated."""
    half = np.asarray(size) / 2
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
    vertices = corners @ pose.R.T + pose.t
    # Each face: four corner indices, with its outward normal below.
    faces = np.array([
        [0, 1, 3, 2], [4, 5, 7, 6],  # -x, +x
        [0, 1, 5, 4], [2, 3, 7, 6],  # -y, +y
        [0, 2, 6, 4], [1, 3, 7, 5],  # -z, +z
    ])
    normals = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0],
                        [0, 1, 0], [0, 0, -1], [0, 0, 1]]) @ pose.R.T
    return vertices, faces, normals


def get_link_points(robot, q):
    """Joint frame origins (start and end of every link), plus the tool tip."""
    points = [T[:3, 3] for T in robot.fkine_all(q).A]
    points.append(robot.fkine(q).t)
    return points


def is_collision(robot, q_matrix, faces, vertex, face_normals):
    """Lab 5 Question 2: True if any link of any pose in q_matrix crosses the mesh."""
    for q in q_matrix:
        if links_cross_mesh(get_link_points(robot, q), faces, vertex, face_normals):
            return True
    return False


def links_cross_mesh(points, faces, vertex, face_normals):
    """The inner loops of Lab 5 Q2 is_collision, for link points already worked out."""
    for i in range(len(points) - 1):
        for j, face in enumerate(faces):
            vert_on_plane = vertex[face][0]
            intersect_p, check = line_plane_intersection(
                face_normals[j], vert_on_plane, points[i], points[i + 1])
            if check == 1:
                for triangle in combinations(face, 3):
                    if is_intersection_point_inside_triangle(intersect_p, vertex[list(triangle)]):
                        return True
    return False


def below_floor(robot, q, floor_z=-.005):
    """Line-plane test of each link against the floor plane."""
    points = get_link_points(robot, q)
    for i in range(len(points) - 1):
        _, check = line_plane_intersection(
            np.array([0, 0, 1]), np.array([0, 0, floor_z]), points[i], points[i + 1])
        if check == 1:
            return True
    return False


def link_boxes(points, mover_points, thickness=.15):
    """Another robot's links as obstacles: a box around each link segment, so the
    Lab 5 Q2 check can test one robot's links against another robot. Boxes that no
    link of the moving robot can reach are skipped (bounding spheres don't overlap)."""
    mover_points = np.array(mover_points)
    mids = (mover_points[:-1] + mover_points[1:]) / 2
    halves = np.linalg.norm(mover_points[1:] - mover_points[:-1], axis=1) / 2
    boxes = []
    for a, b in zip(points[:-1], points[1:]):
        length = np.linalg.norm(b - a)
        if length < 1e-6:
            continue
        reach = halves + np.sqrt(2 * thickness ** 2 + length ** 2) / 2
        if np.all(np.linalg.norm(mids - (a + b) / 2, axis=1) > reach):
            continue
        z = (b - a) / length
        x = np.cross(z, [0, 0, 1])
        if np.linalg.norm(x) < 1e-6:
            x = np.cross(z, [1, 0, 0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        pose = SE3.Rt(np.column_stack([x, y, z]), (a + b) / 2, check=False)
        boxes.append((pose, [thickness, thickness, length]))
    return boxes


class CollisionManager:
    """Checks any of the robots (Staubli, Kuka or ABB) against the floor, boxes and each other."""

    def __init__(self):
        self.reason = ""
        self.extra_obstacles = None  # function returning a list of (name, pose, size)
        self.robots = []  # every robot in the cell, checked against each other

    def obstacles(self, obstacle_pose, obstacle_size):
        boxes = []
        if obstacle_pose is not None:
            boxes.append(("obstacle", obstacle_pose, obstacle_size))
        if self.extra_obstacles:
            boxes += self.extra_obstacles()
        return boxes

    def check(self, robot, obstacle_pose, obstacle_size, q):
        if below_floor(robot, q):
            return True, "floor"
        points = get_link_points(robot, q)
        for name, pose, size in self.obstacles(obstacle_pose, obstacle_size):
            vertices, faces, normals = rectangular_prism(size, pose)
            if links_cross_mesh(points, faces, vertices, normals):
                return True, name
        # Robot vs robot, both ways round: the moving robot's links against boxes around
        # the other's links, then the other's links against boxes around the moving robot's
        # (a link lying wholly inside a box crosses none of its faces, so one way can miss it).
        for other in self.robots:
            if other is robot:
                continue
            other_points = get_link_points(other, other.q)
            for mover_points, boxes in ((points, link_boxes(other_points, points)),
                                        (other_points, link_boxes(points, other_points))):
                for pose, size in boxes:
                    vertices, faces, normals = rectangular_prism(size, pose)
                    if links_cross_mesh(mover_points, faces, vertices, normals):
                        return True, other.name
        return False, None

    def path_clear(self, robot, path, obstacle_pose, obstacle_size):
        for q in path:
            hit, name = self.check(robot, obstacle_pose, obstacle_size, q)
            if hit:
                self.reason = f"{robot.name} would hit the {name}"
                return False
        return True

    def random_obstacle_pose(self, robot, path, obstacle_size, keep_clear=(), others=(), rng=None):
        """Put the obstacle around the tool tip at a random point 30-70% along the path.
        It must block the path, but not touch the robot's start or goal, the other
        robots (others: list of (robot, q)) or come near the task targets (keep_clear)."""
        rng = rng or np.random.default_rng()
        start, goal = path[0], path[-1]
        for k in rng.permutation(np.arange(int(.3 * len(path)), int(.7 * len(path)) + 1)):
            pose = SE3(robot.fkine(path[k]).t)
            if any(np.linalg.norm(pose.t - p) < .45 for p in keep_clear):
                continue
            if any(self.check(r, pose, obstacle_size, q)[0]
                   for r, q in [(robot, start), (robot, goal), *others]):
                continue
            if not self.path_clear(robot, path, pose, obstacle_size):
                return pose
        return None

    def random_extra_obstacles(self, robot, path, size, count, keep_clear=(), avoid=(), others=(), existing=(), clear=.7):
        """Spawn up to count boxes at uniformly random centres (np.random.rand, as in
        Lab 5 Q3) in a region around the robot's path. A box is rejected if it touches
        the robot's start or goal, the other robots (others: list of (robot, q)), comes
        comes within clear (m) of a task target (keep_clear), lands in an avoid area (list of (xy_low, xy_high))
        or overlaps another box (existing: poses already placed)."""
        tips = np.array([robot.fkine(q).t for q in path])
        low = tips.min(axis=0) - [.7, .7, 0]
        high = tips.max(axis=0) + [.7, .7, 0]
        low[2] = size[2] / 2 + .05
        high[2] = max(high[2], low[2] + .3)
        boxes = []
        for _ in range(50 * count):
            if len(boxes) == count:
                break
            centre = low + np.random.rand(3) * (high - low)
            pose = SE3(centre)
            if any(np.linalg.norm(centre - p) < clear for p in keep_clear):
                continue
            if any(np.all(centre[:2] > a) and np.all(centre[:2] < b) for a, b in avoid):
                continue
            if any(np.linalg.norm(centre - b.t) < np.linalg.norm(size) for b in [*existing, *boxes]):
                continue
            if any(self.check(r, pose, size, q)[0]
                   for r, q in [(robot, path[0]), (robot, path[-1]), *others]):
                continue
            boxes.append(pose)
        return boxes

    def find_detour(self, robot, start, goal, obstacle_pose, obstacle_size, steps, tries=200, rng=None):
        """Random joint-space waypoint q_mid where start -> q_mid -> goal is collision-free.
        Samples around the middle of the move and searches wider after each failure."""
        rng = rng or np.random.default_rng()
        lower, upper = robot.qlim
        middle = (start + goal) / 2
        for i in range(tries):
            spread = .3 + 1.5 * i / tries
            q_mid = np.clip(middle + rng.normal(0, spread, len(start)), lower, upper)
            if self.check(robot, obstacle_pose, obstacle_size, q_mid)[0]:
                continue
            if (self.path_clear(robot, rtb.jtraj(start, q_mid, steps).q, obstacle_pose, obstacle_size)
                    and self.path_clear(robot, rtb.jtraj(q_mid, goal, steps).q, obstacle_pose, obstacle_size)):
                return q_mid
        self.reason = f"No way around the obstacle for the {robot.name}"
        return None
