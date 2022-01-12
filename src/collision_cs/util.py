# import math
from numbers import Real
from typing import Any, Union
import casadi.casadi as cs

from cs_tools import cs2bool


LEFT_VORONOI_REGION = -1
MIDDLE_VORONOI_REGION = 0
RIGHT_VORONOI_REGION = 1
ALLOWED_NUM_TYPES = (int, float, cs.SX)


class Vector:
    __slots__ = ['x', 'y']

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other: Any):
        if isinstance(other, ALLOWED_NUM_TYPES):
            return Vector(self.x+other, self.y+other)

        return Vector(self.x+other.x, self.y+other.y)

    def __mul__(self, other: Any):
        if isinstance(other, ALLOWED_NUM_TYPES):
            return Vector(self.x*other, self.y*other)

        return Vector(self.x*other.x, self.y*other.y)

    def __sub__(self, other: Any):
        if isinstance(other, ALLOWED_NUM_TYPES):
            return Vector(self.x-other, self.y-other)

        return Vector(self.x-other.x, self.y-other.y)

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __truediv__(self, other: Any):
        if isinstance(other, ALLOWED_NUM_TYPES):
            return Vector(self.x/other, self.y/other)

        return Vector(self.x/other.x, self.y/other.y)

    def __floordiv__(self, other: Any):
        if isinstance(other, ALLOWED_NUM_TYPES):
            return Vector(self.x//other, self.y//other)

        return Vector(self.x//other.x, self.y//other.y)

    def __mod__(self, other: Any):
        if isinstance(other, ALLOWED_NUM_TYPES):
            return Vector(self.x % other, self.y % other)

        return Vector(self.x % other.x, self.y % other.y)

    def __eq__(self, other: Any):
        if not isinstance(other, Vector):
            return False
        return self.x == other.x and self.y == other.y

    def __ne__(self, other: Any):
        if not isinstance(other, Vector):
            return True

        return self.x != other.x or self.y != other.y

    def __getitem__(self, index: int):
        return [self.x, self.y][index]

    def __contains__(self, value):
        return value == self.x or value == self.y

    def __len__(self):
        return 2

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Vector [{x}, {y}]".format(x=self.x, y=self.y)

    def copy(self):
        return Vector(self.x, self.y)

    def set(self, other):
        self.x = other.x
        self.y = other.y

    def perp(self):
        return Vector(self.y, -self.x)

    # def rotate(self, angle: Union[int, float, Real]):
    #     return Vector(self.x * math.cos(angle) - self.y * math.sin(angle), self.x * math.sin(angle) + self.y * math.cos(angle))

    def reverse(self):
        return Vector(-self.x, -self.y)

    def int(self):
        return Vector(int(self.x), int(self.y))

    def normalize(self):
        dot = self.ln()
        return self / dot

    def project(self, other):
        amt = self.dot(other) / other.ln2()

        return Vector(amt * other.x,  amt * other.y)

    def project_n(self, other):
        amt = self.dot(other)

        return Vector(amt * other.x, amt * other.y)

    def reflect(self, axis):
        v = Vector(self.x, self.y)
        v = v.project(axis) * 2
        v = -v

        return v

    def reflect_n(self, axis):
        v = Vector(self.x, self.y)
        v = v.project_n(axis) * 2
        v = -v

        return v

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def ln2(self):
        return self.dot(self)

    def ln(self):
        return cs.sqrt(self.ln2())


def flatten_points_on(points, normal, result):
    minpoint = 1e10 # cs.inf
    maxpoint = -1e10 # cs.inf

    for i in range(len(points)):
        dot = points[i].dot(normal)
        # Changed for cs
        minpoint = cs.fmin(minpoint, dot)
        maxpoint = cs.fmax(maxpoint, dot)

    result[0] = minpoint
    result[1] = maxpoint


def is_separating_axis(a_pos, b_pos, a_points, b_points, axis, response=None):
    range_a = [0, 0]
    range_b = [0, 0]

    offset_v = b_pos-a_pos

    projected_offset = offset_v.dot(axis)

    flatten_points_on(a_points, axis, range_a)
    flatten_points_on(b_points, axis, range_b)

    range_b[0] += projected_offset
    range_b[1] += projected_offset

    #### Changed for cs ####
    # FOR READABILITY CHECK ORIGINAL util.py in collision library (https://github.com/qwertyquerty/collision/blob/master/collision/util.py)
    # collision: Boolean Flag with 0: no collision -> output is surpressed to 0 / 1: collision possible (at least regarding this axis)   
    # Firstly, for first boolean (range_a[0] > range_b[1])
    collision1 = cs2bool(- (range_a[0] - range_b[1]))   # here -residual because we want 0 when True
    # Then, for the second boolean (range_b[0] > range_a[1])
    collision2 = cs2bool(- (range_b[0] - range_a[1]))   # here -residual because we want 0 when True
    # Combine in "or"
    collision = cs.fmin(1.0,collision1+collision2)  # only 1 if at least one is 1
    # flag to surpress output if no collision 
    # flag = (no_collision - 1)**no_collision  # map 1 to 0 and vice versa

    # Measure overlap
    overlap = 0

    # if range_a[0] < range_b[0]:
    (b1,nb1) = cs2bool(range_b[0] - range_a[0], with_else=True)
    # if range_a[1] < range_b[1]:
    (b2,nb2) = cs2bool(range_b[1] - range_a[1], with_else=True)
    # Add overlap (just one term remains without being canceled out)
    overlap += b1*b2*(range_a[1] - range_b[0])

    option_11 = range_a[1] - range_b[0]
    option_12 = range_b[1] - range_a[0]
    # overlap = option_1 if option_1 < option_2 else -option_2
    (b3,nb3) = cs2bool(option_12 - option_11, with_else=True)
    # Add overlap (just one term remains without being canceled out)
    overlap += b1*nb2*b3*option_11
    overlap += b1*nb2*nb3*(-option_12)

    # if range_a[1] > range_b[1]:
    (b4,nb4) = cs2bool(range_a[1] - range_b[1], with_else=True)
    overlap += nb1*b4*(range_a[0] - range_b[1])

    option_21 = range_a[1] - range_b[0]
    option_22 = range_b[1] - range_a[0]
    # overlap = option_1 if option_1 < option_2 else -option_2
    (b5,nb5) = cs2bool(option_22 - option_21, with_else=True)
    overlap += nb1*nb4*b5*option_21
    overlap += nb1*nb4*nb5*(-option_22)

    # If no collision set overlap to 0
    overlap *= collision

    # Take absolut value
    abs_overlap = cs.sqrt(overlap**2)
    # if abs_overlap < response.overlap:
    # TODO: why no checking in the other direction???
    b_abs = cs2bool(response.overlap - abs_overlap)

    # Update overlap
    response.overlap += b_abs*(abs_overlap-response.overlap)  # increase if necessary

    # DONT NEED NORMAL VECTOR
    # response.overlap_n.set(axis)
    # if overlap < 0:
    #     response.overlap_n = response.overlap_n.reverse()

    # DONT NEED RETURN
    # return no_collision    # 1 if no collision, 0 if collision possible (no seperating axis found)
    ####


def voronoi_region(line, point):
    dp = point.dot(line)

    if dp < 0:
        return LEFT_VORONOI_REGION
    elif dp > line.ln2():
        return RIGHT_VORONOI_REGION
    return MIDDLE_VORONOI_REGION
