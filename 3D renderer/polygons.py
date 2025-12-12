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
    def __init__(self, vertices=None, mesh=None, pos=(0, 0, 0)):
        self.vertices = vertices
        self.mesh = mesh
        self.pos = pos
    

    def CopyStructure(self, obj3d):
        """
        Copies the vertices and the mesh of an object to the current one.

        Arguments : 
            obj3d : Object3D
        
        Return Type : 
            None
        """
        self.vertices = []
        self.mesh = []

        for vertex in obj3d.vertices:
            v = [0, 0, 0]
            for i in range(3):
                v[i] = vertex[i] + self.pos[i]
            self.vertices.append((v[0], v[1], v[2]))

        for element in obj3d.mesh:
            self.mesh.append(element)