"""
Extension to the krpc drawing api
"""
from __future__ import absolute_import, print_function, division


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
    nX = center[0] - size
    nY = center[1] - size
    nZ = center[2] - size

    topFrontLeft = (x, y, z)
    topFrontRight = (nX, y, z)
    topBackLeft = (x, nY, z)
    topBackRight = (nX, nY, z)
    bottomFrontLeft = (x, y, nZ)
    bottomFrontRight = (nX, y, nZ)
    bottomBackLeft = (x, nY, nZ)
    bottomBackRight = (nX, nY, nZ)

    facesCoords = [[topFrontLeft, topFrontRight, topBackRight, topBackLeft],
             [bottomFrontLeft, bottomBackLeft, bottomBackRight, bottomFrontRight],
             [topFrontLeft, topBackLeft, bottomBackLeft, bottomFrontLeft],
             [topFrontLeft, topFrontRight, bottomFrontRight, bottomFrontLeft],
             [topFrontRight, topBackRight, bottomBackRight, bottomFrontRight],
             [topBackRight, topBackLeft, bottomBackLeft, bottomBackRight]]

    faces = []

    for verts in facesCoords:
        faces.append(connection.drawing.add_polygon(verts, referenceFrame))
