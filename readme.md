# Simple 3D Renderer in Python

## Requirements

The following libraries are required to run the program : 

* `pygame`
* `numpy`

## Description

This is a simple 3D renderer capable of displaying a few solids at once. It doesn't allow texture rendering and only works with convex polyhedrons. It is a small project to experiment with rotation matrices, perspective projection, hidden surface determination, etc.

## How to configure the scene

Configuring the objects to display is done in the `objects.txt` file. By default, there's a dodecagonal prism. The format of this file is as follows : 

Reserved characters are `{} [] () # , - and digits`. To include comments, begin a line with `?`. To declare a new object, use `{}`. Each object contains two lists, for the vertices and for the mesh, both declared with `[]`.

### Vertices

**The first list is always for the vertices** and it contains tuples with three coordinates in the format `(x, y, z,)`. Each tuple is declared with `()` and each coordinate is separated with a `,` even the last one. **No separator is needed for the tuples**, simply open and close brackets to specify them. **Do not add seperators from the reserved characters**, as it will cause issues when reading the file.

### Mesh

**The second list is always for the mesh of the object**. It contains tuples with four elements representing triangles, in the same format as the vertices. The first three elements are indices pointing to vertices to form the triangle. The fourth element is the color of the triangle, provided in hexadecimal format.

-  For instance, to create a blue triangle with the first three vertices, it will be `(0, 1, 2, #0000FF)` as 0, 1, and 2 point to the first three vertices.
 
 **All vertices within a triangle must be specified in an anticlockwise order to be rendered**, otherwise the triangle will not be facing the correct way due to back face culling.

 Additional information is available on Wikipedia about [**rotation matrices**](https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions) and [**hidden surface determination algorithms**](https://en.wikipedia.org/wiki/Hidden-surface_determination#Algorithms).
