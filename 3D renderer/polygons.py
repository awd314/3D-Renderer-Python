class Object3D:
    """
    A collection of vertices and a mesh indicating how to construct
    a solid.

    The vertices are a list of tuples with 3 coordinates and the mesh is
    a list of tuples with 4 coordinates. The first three are indices pointing
    to the vertices, to know which points to use to make a triangle. It also
    reduces redundancy as some triangles share the same vertices. The fourth
    coordinate indicates the color in which the triangle needs to be drawn.

    The color is an integer and is a direct conversion from hexadecimal to decimal
    ex : "#125C1F" = 120000 + 5C00 + 1F = 1203231
         "#0001FF" = 100 + FF = 256 + 255 = 511
    """
    def __init__(self, vertices=None, mesh=None):
        self.vertices = vertices
        self.mesh = mesh
