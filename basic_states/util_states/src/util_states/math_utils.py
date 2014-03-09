import copy
import math


def xy_with_angle(alfa, distance):
    """
Returns the x and y components for a 2D vector that
has that angle ( radians) reffered to the horizontal and has the
magnitude distance.
"""

    x = distance * math.cos(alfa)
    y = distance * math.sin(alfa)

    return x, y


def vector_magnitude(vec):
    """
Returns the magnitude of the given vector.
"""
    return math.sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z)


def normalize_vector(vec):
    """
Returns a normalized (unitary) copy of *vec*.
"""
    magnitude = vector_magnitude(vec)
    unit = copy.deepcopy(vec)
    if magnitude > 0.0:
        unit.x /= magnitude
        unit.y /= magnitude
        unit.z /= magnitude
    return unit


def multiply_vector(vec, factor):
    """
Multiplies each component of *vec* by the given *factor*.
There is the factor zero condition, to aviod having variables
that were negative having a value or -0.0.
"""
    result = copy.deepcopy(vec)
    if factor == 0.0:
        result.x = 0.0
        result.y = 0.0
        result.z = 0.0
    else:
        result.x *= factor
        result.y *= factor
        result.z *= factor
    return result


def add_vectors(vec1, vec2):
    """
Element-wise addition of vec1 and vec2.
"""
    result = copy.deepcopy(vec1)
    result.x += vec2.x
    result.y += vec2.y
    result.z += vec2.z
    return result


def substract_vector(vec1, vec2):
    """
Element-wise substraction of vec2 from vec1.
"""
    result = copy.deepcopy(vec1)
    result.x -= vec2.x
    result.y -= vec2.y
    result.z -= vec2.z
    return result


def multiply_quaternions(a, b):
    """
Multiplies quaternions `a' and `b`. Remember that this operation
is noncommutative.
"""
    result = copy.deepcopy(a)
    # From "3D Math Primer for Graphics and Game Development", pag. 165
    result.w = a.w*b.w - a.x*b.x - a.y*b.y - a.y*b.z
    result.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y
    result.y = a.w*b.y + a.y*b.w + a.x*b.z - a.z*b.x
    result.z = a.w*b.z + a.z*b.w + a.y*b.x - a.x*b.y
    return result


def cross_product(a, b):
    """
Makes the cross product of two vectors
"""
    result = copy.deepcopy(a)
    result.x = a.y*b.z - a.z*b.y
    result.y = a.z*b.x - a.x*b.z
    result.z = a.x*b.y - a.y*b.x

    return result


def dot_product(a, b):
    """
Makes a dot product of two vectors
Returns a number
"""
    result = a.x*b.x + a.y*b.y + a.z*b.z

    return result


def euclidean_distance(pose1, pose2):
    dist = math.sqrt(((pose2.position.x - pose1.position.x)**2) + ((pose2.position.y - pose1.position.y)**2))
    return dist

def euclidean_distance_3d(pose1, pose2):
    dist = math.sqrt(((pose2.position.x - pose1.position.x)**2) + ((pose2.position.y - pose1.position.y)**2) + ((pose2.position.z - pose1.position.z)**2))
    return dist

# vim: expandtab ts=4 sw=4