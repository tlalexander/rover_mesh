"""Utilities for procedurally generating meshes and manipulating them."""

import math
import numpy as np
from opensimplex import OpenSimplex
from PIL import Image
import queue
import sys
import pickle


_MESH_NAME = "obj/generated.obj"
MESH_DETAILS_NAME = "obj/generated.pkl"
_FREQUENCY_MULTIPLIER = 0.05
_WATER_RANGE_MAX_PERCENT = 10
_SAND_RANGE_MAX_PERCENT = 50
_NEIGHBOR_DIST_TOLERANCE = 2
_MIN_ALLOWED_PATH_ANGLE = 160


class Mesh(object):
    """A class for generating meshes."""

    def __init__(self, seed):
        """Create the mesh object with these default parameters."""
        self.seed = seed
        self.height_factor = 5
        self.step = 0.1
        self.width = 50
        self.depth = 50
        self.width_steps = int(self.width/self.step)
        self.depth_steps = int(self.depth/self.step)
        self.zero_offset = -0.1
        self.lacunarity = 3
        self.persistence = 0.5
        self.octaves = 3
        self.total_vertices = self.width_steps * self.depth_steps
        self.mesh = None
        self.simplex = OpenSimplex(seed=seed)
        self.path_width = 20
        self.decay_distance = 6

    def save_mesh_details(self):
        path_meters = []
        for p in self.path:
            path_meters.append(((p[0] - self.width_steps/2) * self.step, (p[1] - self.depth_steps/2) * self.step))
        with open(MESH_DETAILS_NAME, 'wb') as f:
            pickle.dump(path_meters, f)

    def save_mesh(self):
        """Save the mesh to disk with _MESH_NAME for the filename."""
        vertices = []
        faces = []
        uv = []
        for x_coord in range(len(self.mesh)):
            for y_coord in range(len(self.mesh[0])):
                vertices.append(self.mesh[x_coord][y_coord])

        for y_idx in range(int(self.depth_steps)):
            for x_idx in range(int(self.width_steps)):
                row_offset = y_idx * self.width_steps
                start = int(x_idx + row_offset)
                if x_idx < self.depth_steps-1 and y_idx < self.width_steps-1:
                    # Note that order of these values affects normal
                    # orientation. That is, a face [1,2,3] will have a
                    # different normal than [3,2,1].
                    faces.append(
                      [int(start + self.width_steps),
                       start+1,
                       start])
                    faces.append(
                        [start + 1,
                         int(start + self.width_steps),
                         int(start + self.width_steps + 1)])
                    # The six vertices involved need six texture values, so we
                    # add them here.
                    for _ in range(6):
                        uv.append([y_idx/int(self.width_steps),
                                   x_idx/int(self.depth_steps)])

        vertices = np.array(vertices)
        faces = np.array(faces)
        uv = np.array(uv)

        with open(_MESH_NAME, 'w') as meshfile:
            meshfile.write("mtllib generated.obj.mtl\nusemtl material_0\n")
            for u in uv:
                meshfile.write('vt {} {}\n'.format(u[0], u[1]))
            for vert in vertices:
                meshfile.write(
                    'v {} {} {}\n'.format(vert[0], vert[1], vert[2]))
            facenum = 1
            for f in faces:
                meshfile.write(
                    'f {}/{} {}/{} {}/{}\n'.format(
                        f[0]+1, facenum, f[1]+1, facenum+1, f[2]+1, facenum+2))
                facenum += 3

    def get_noise(self, x_val, y_val, octaves):
        """Get a sample of noise at a point."""
        factor = 2
        height = 0
        for octave in range(octaves):
            frequency = self.lacunarity ** octave * _FREQUENCY_MULTIPLIER
            scale = self.persistence ** octave
            height += self.simplex.noise2d(x=x_val * frequency,
                                           y=y_val * frequency) * scale
        return height * factor

    def clip_mesh(self, min=-math.inf, max=math.inf):
        """Flatten out the top or bottom of the mesh."""
        for x_coord in range(self.width_steps):
            for y_coord in range(self.depth_steps):
                coord = self.mesh[x_coord][y_coord]
                if coord[2] > max:
                    coord[2] = max
                if coord[2] < min:
                    coord[2] = min

    def generate(self, octaves):
        """Generate the basic mesh geometry - the rolling hills."""
        # ------width-----
        #  -step-
        # |-0---1---4---6--
        # d | / | / | / |
        # e 2---3---5---7--
        # p | / | / | / |
        # t 8--- --- --- --
        # h
        # |

        width_range = np.linspace(
                -self.width/2, self.width/2, num=self.width_steps)
        depth_range = np.linspace(
                -self.depth/2, self.depth/2, num=self.depth_steps)

        self.mesh = np.zeros((self.depth_steps, self.depth_steps, 3))

        for x_coord in range(len(width_range)):
            for y_coord in range(len(depth_range)):
                x_val = width_range[x_coord]
                y_val = depth_range[y_coord]
                self.mesh[x_coord][y_coord][0] = x_val
                self.mesh[x_coord][y_coord][1] = y_val
                self.mesh[x_coord][y_coord][2] = self.get_noise(
                    x_val, y_val, octaves)

    def calculate_min_max(self):
        """Calculate the minimum and maximum mesh heights."""
        self.mesh_max = np.amax(self.mesh[:, :, 2])
        self.mesh_min = np.amin(self.mesh[:, :, 2])

    def init_texture(self):
        """Create the texture with colors based on height."""
        self.tex = np.zeros(shape=self.mesh.shape)
        for x_coord in range(self.width_steps):
            for y_coord in range(self.depth_steps):
                mesh_val = self.mesh[x_coord][y_coord][2]
                height_pct = (mesh_val-self.mesh_min) / (
                              self.mesh_max-self.mesh_min) * 100
                if height_pct < _WATER_RANGE_MAX_PERCENT:
                    self.tex[x_coord][y_coord] = (0, 0, 255)
                elif height_pct < _SAND_RANGE_MAX_PERCENT:
                    self.tex[x_coord][y_coord] = (222, 184, 135)
                else:
                    self.tex[x_coord][y_coord] = (34, 139, 34)

    def save_texture(self):
        """Save the texture as PNG."""
        # scale = 32
        im_array = np.rot90(self.tex)
        im = Image.fromarray(im_array.astype('uint8'))
        # size = im.width * scale, im.height * scale
        # im = im.resize(size, Image.BILINEAR)
        im.save(_MESH_NAME.split('.')[0] + '.png')

    def get_neighbors(self, last_coord, current_coord, dist):
        """Get the neighbor nodes from the graph with some criteria."""
        offsets = range(-1-dist, dist+2)
        neighbors = []
        rejected_points = []
        for x_coord in offsets:
            for y_coord in offsets:
                if x_coord == 0 and y_coord == 0:
                    continue
                this_neighbor = (
                    current_coord[0] + x_coord), (current_coord[1] + y_coord)
                dist_neighbor = self.get_distance(current_coord, this_neighbor)
                if dist_neighbor > dist + 1:
                    continue
                if last_coord:
                    if abs(dist - dist_neighbor) > _NEIGHBOR_DIST_TOLERANCE:
                        rejected_points.append(this_neighbor)
                        continue
                    angle = self.get_angle(
                        last_coord, current_coord, this_neighbor)
                    if angle < _MIN_ALLOWED_PATH_ANGLE:
                        rejected_points.append(this_neighbor)
                        continue

                if 0 < this_neighbor[0] < self.width_steps and \
                   0 < this_neighbor[1] < self.depth_steps:
                    neighbors.append(this_neighbor)
        return rejected_points, neighbors

    def get_cost(self, last, current, nextp):
        """Calculate the cost to the next point."""
        first_val = self.mesh[current[0]][current[1]][2]
        second_val = self.mesh[nextp[0]][nextp[1]][2]
        height_pct = (second_val - self.mesh_min) / (
            self.mesh_max - self.mesh_min) * 100
        difference = first_val - second_val
        difference *= 1000
        cost = math.pow(abs(difference), 1.3)
        if height_pct <= _WATER_RANGE_MAX_PERCENT:
            cost += 200
        return cost

    def get_angle(self, last, current, nextp):
        """Get the 2D planar angle between three grid points."""
        #  (l) *
        #      a  c
        #  (c) * b  * (n)

        dist_a = self.get_distance(last, current)
        dist_b = self.get_distance(current, nextp)
        dist_c = self.get_distance(last, nextp)

        angle = math.degrees(
            np.arccos(
             (dist_a ** 2 + dist_b ** 2 - dist_c ** 2) / (2 * dist_a * dist_b))
            )
        if math.isnan(angle):
            # Check for colinearity by confirming the distances sum as
            # expected.
            distances = [dist_a, dist_b, dist_c]
            distances.sort()
            calculated_diff = distances[0] + distances[1] - distances[2]
            if math.isclose(0, calculated_diff, abs_tol=0.001):
                # Points are colinear.
                if last[0] < current[0] < nextp[0] or \
                  last[0] > current[0] > nextp[0] or \
                  last[1] < current[1] < nextp[1] or \
                  last[1] > current[1] > nextp[1]:
                    return 180
                else:
                    return 0
        return angle

    def get_distance(self, p1, p2):
        """Get the linear distance between two 2d coordinates."""
        if not all((p1, p2)):
            return 0
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def interpolate(self, first, second, ratio):
        """Calculate a value as a ratio of the first and second values.

        Zero means return the second value, one means return the first.
        Values outside of that range are allowed.
        """
        return second + (first-second) * ratio

    def get_height(self, coords):
        """Get mesh height at a coordinate."""
        return self.mesh[coords[0]][coords[1]][2]

    def is_this_section(self, points, subject):
        """Determine if subject point lies in section p1-2 of p0-4."""
        angle_2_1_S = self.get_angle(points[2], points[1], subject)
        angle_0_1_S = self.get_angle(points[0], points[1], subject)
        angle_1_2_S = self.get_angle(points[1], points[2], subject)
        angle_3_2_S = self.get_angle(points[3], points[2], subject)

        if angle_2_1_S < angle_0_1_S and angle_1_2_S < angle_3_2_S:
            return True
        elif math.isclose(angle_1_2_S, angle_3_2_S, abs_tol=0.0001):
            return True
        else:
            return False

    def test_is_this_section(self):
        """A lonely test, to be moved to a test file."""
        points = [(0, 0), (2, 0), (4, 0), (6, 0)]
        success = True
        success &= self.is_this_section(points, (3, 0.1))
        success |= self.is_this_section(points, (5, 0.1))
        if not success:
            raise RuntimeError("Is_this_section test fail")

    # self.test_is_this_section()

    def get_path_values(self, points, subject):
        """Calculate the percent along the path and distance from it.

        In particular, this function returns values for all points present on
        the wide segment between points[1] and points[2] while respecting the
        angle at which segments meet.

        Returns:
            A tuple. The first value is the distance along the section the
              point is found (as a fraction of 1), and the second value is the
              distance from the path the point lies.

        """
        if not self.is_this_section(points, subject):
            return None
        theta = self.get_angle(points[1], points[2], subject)
        if theta > 90.001:
            a_angle = theta - 90
        else:
            a_angle = 90 - theta
        hypotenuse = self.get_distance(points[2], subject)
        dist_points = self.get_distance(points[1], points[2])
        if math.isclose(a_angle, 0, abs_tol=0.00001):
            percent_along = 1.0
            dist_away = hypotenuse
        else:
            dist_along = hypotenuse * math.sin(math.radians(a_angle))
            dist_away = hypotenuse * math.cos(math.radians(a_angle))
            if theta > 90.001:
                percent_along = (dist_along + dist_points)/dist_points
            else:
                percent_along = (dist_points-dist_along)/dist_points
        return percent_along, dist_away

    def visualize_search(self):
        """Paint the points involved in pathfinding red."""
        for c in self.cost_so_far.keys():
            self.tex[c[0]][c[1]] = (255, 0, 0)

    def find_path_costs(self, start, goal):
        """Find the cost for possible routes to the goal."""
        # Path finding tutorial:
        # https://www.redblobgames.com/pathfinding/a-star/introduction.html
        self.goal = goal
        self.start = start
        frontier = queue.PriorityQueue()
        frontier.put((0, self.start))
        self.came_from = {}
        self.came_from[self.start] = None
        self.cost_so_far = {}
        self.cost_so_far[self.start] = 0
        current = None

        count = 0
        last = None
        while not frontier.empty():
            current = frontier.get()
            current = current[1]
            last = self.came_from[current]
            rejected, neighbors = self.get_neighbors(last, current, 4)

            if current == self.goal or self.goal in rejected or \
                    self.goal in neighbors:
                print()
                print("Found Goal!")
                self.goal = current  # Just call where we are the goal.
                break
            for next in neighbors:
                new_cost = self.cost_so_far[current] + \
                    self.get_cost(last, current, next)
                if next not in self.cost_so_far.keys() or new_cost < (
                  0.8 * self.cost_so_far[next]):
                    self.cost_so_far[next] = new_cost
                    self.came_from[next] = current

                    priority = new_cost/30 + self.get_distance(next, goal)

                    # This still slightly favors smaller coordinates.
                    frontier.put((priority, next))
                    last = current
                    count += 1
                    if count % 1000:
                        sys.stdout.write(
                            "\rVisited: {:0.2f}%, Total: {:0.2f}%".format(
                              100 * count/self.total_vertices,
                              100 * len(self.cost_so_far)/self.total_vertices))
                        sys.stdout.flush()
                    if count > 5000000:
                        raise RuntimeError("Exceeded 5000000 count")

    def reconstruct_path(self):
        """Construct the path from the start to the goal."""
        # Reconstruct path.
        current = self.goal
        self.path = []
        count = 0
        while current != self.start:
            self.path.append(current)
            current = self.came_from[current]
            count += 1
            if count > 250000:
                print(self.path[:-10])
                print(current)
                raise RuntimeError("Exceeded 250000 count on path calculation")
        self.path.append(self.start)  # optional
        self.path.reverse()  # optional

    def build_path(self):
        """Add the path to the 3D geometry and texture."""
        white = [255, 255, 255]
        black = [0, 0, 0]
        visited = []

        for index in range(0, len(self.path)-5):
            points = self.path[index:index+4]
            last_value = points[1]
            value = points[2]
            self.tex[value[0]][value[1]] = black
            height = self.get_height(value)
            last_height = self.get_height(last_value)
            rejects, neighbors = self.get_neighbors(
                            None, value, self.path_width * 2)
            for neighbor in neighbors:
                if neighbor in points:
                    continue
                ret = self.get_path_values(points, neighbor)
                if ret and neighbor not in visited:
                    dist = abs(ret[1])
                    if dist < self.path_width/2-2:
                        self.tex[neighbor[0]][neighbor[1]] = white
                    neighbor_height = self.get_height(neighbor)
                    trail_height = self.interpolate(
                        height, last_height, ret[0])
                    if dist < self.path_width/2 + self.decay_distance:
                        if dist > self.path_width/2:
                            trail_height = self.interpolate(
                                neighbor_height, trail_height,
                                (dist - self.path_width/2)/self.decay_distance)
                        self.mesh[neighbor[0]][neighbor[1]][2] = trail_height
                        visited.append(neighbor)
