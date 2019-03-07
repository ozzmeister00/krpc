"""
Extension to the krpc drawing api
"""
from __future__ import absolute_import, print_function, division

import krpc

from . import utils

def draw_cube(connection, center, size, referenceFrame):
    """

    :param connection: the connection on which to draw
    :param center: the center of the cube in the given reference frame
    :param size: the size of the cube (width, length, and height)
    :param referenceFrame: the reference frame in which to draw the cube
    """
    size /= 2
    x = center[0] + size
    y = center[1] + size
    z = center[2] + size

    # TODO nX, nY, nZ = centerX - size, not negative X

    topFrontLeft = (x, y, z)
    topFrontRight = (-x, y, z)
    topBackLeft = (x, -y, z)
    topBackRight = (-x, -y, z)
    bottomFrontLeft = (x, y, -z)
    bottomFrontRight = (-x, y, -z)
    bottomBackLeft = (x, -y, -z)
    bottomBackRight = (-x, -y, -z)

    faces = [[topFrontLeft, topFrontRight, topBackRight, topBackLeft],
             [bottomFrontLeft, bottomBackLeft, bottomBackRight, bottomFrontRight],
             [topFrontLeft, topBackLeft, bottomBackLeft, bottomFrontLeft],
             [topFrontLeft, topFrontRight, bottomFrontRight, bottomFrontLeft],
             [topFrontRight, topBackRight, bottomBackRight, bottomFrontRight],
             [topBackRight, topBackLeft, bottomBackLeft, bottomBackRight]]

    for face in faces:
        print(face)
        connection.drawing.add_polygon(face, referenceFrame)

