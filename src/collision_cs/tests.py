import math

from .circle import Circle
from .poly import Poly, Concave_Poly
from .response import Response
from .util import Vector, flatten_points_on, voronoi_region, is_separating_axis


LEFT_VORONOI_REGION = -1
MIDDLE_VORONOI_REGION = 0
RIGHT_VORONOI_REGION = 1
RESPONSE = Response()
TEST_POINT = Poly(Vector(0, 0), [Vector(0, 0), Vector(0.0000001, 0.0000001)])


def test_aabb(b1,b2):
    return b1[0][0] <= b2[1][0] and b2[0][0] <= b1[1][0] and b1[0][1] <= b2[2][1] and b2[0][1] <= b1[2][1]

def test_poly_poly(a, b, response):
    a_points = a.rel_points
    b_points = b.rel_points
    a_pos = a.pos
    b_pos = b.pos

    # NO NEED FOR RETURN BOOLEAN, response.overlap has all information needed
    # Check that for all normals in a and b there is no seperating axis found
    # boolean = 1     # 1: collision, 0: no collision 

    # for n in a.normals:
    #     sep_axis_found = is_separating_axis(a_pos, b_pos, a_points, b_points, n, response):
    #     # sep_axis_found = 1, when no collision -> map to 0 to give back correct signal
    #     boolean *= (sep_axis_found - 1)**sep_axis_found     # map 1 to 0 and vice versa

    for n in a.normals:
        is_separating_axis(a_pos, b_pos, a_points, b_points, n, response)

    for n in b.normals:
        is_separating_axis(a_pos, b_pos, a_points, b_points, n, response)

    response.a = a
    response.b = b
    # response.overlap_v = response.overlap_n * response.overlap

def collide(a, b, response=None):
    # TODO: implement bounding box checking -> even useful?
    # if not test_aabb(a.aabb, b.aabb):
    #     return False
    test_poly_poly(a, b, response)
    