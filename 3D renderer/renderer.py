from settings import *
import numpy as np
import pygame as pg


class Renderer:
    """
    Both calculates objects positions and displays the objects on a given
    surface. All the rotations and translations (camera position) are kept
    in memory to be able to display everything.
    """
    def __init__(self, screen, objectToRender):
        self.screen = screen # Surface to draw on
        self.objectToRender = objectToRender

        self.rotations = [0, 0] # [y rotation : theta, x rotation : phi]
        self.camera = [0, 0, 0] # Camera position
        self.acceleration = [0, 0, 0] # Camera acceleration
        self.rotationAcceleration = [0, 0] # Angular acceleration of the camera
        self.base = np.array([ # x, y, and z axis scaled to CAM_SPEED parameter
            [CAM_SPEED, 0, 0],
            [0, CAM_SPEED, 0],
            [0, 0, CAM_SPEED]
        ])
        self.lightRay = (1, 0, 0) # Direction of the light ray
    

    def Normalize(self, v):
        """
        Returns a normalized version of a given vector. Ex : (3, 4, 0) -> 
        (3/5, 2/5, 0). The vector is first scaled to a smaller version because for 
        large values of the coordinates, we get integer overflows that mess up with
        the sqrt and the norm.
        """
        coef = max([abs(v[i]) for i in range(3)]) # Gets max coordinate (in abs) of the given vector
        normalizedVector = [v[i] / coef for i in range(3)] # Scales the vector by that coordinate
        norm = np.sqrt(sum([normalizedVector[i] * normalizedVector[i] for i in range(3)])) # Calculates the norm
        normalizedVector = [normalizedVector[i] / norm for i in range(3)] # Divides each component by the norm to normalize the vector
        return normalizedVector
    

    def ApplyShadows(self):
        """
        Sets up a light source on the z axis behind the camera. It doesn't follow
        the camera movements. Since everything in the scene is static, the light
        rays are calculated once and not every frame.

        Light rays are all supposed parallel and colinear to vector (0, 0, 1). As
        such, to know how dark a surface has to be, we first calculate the normal
        vector to that surface. Then, we apply a dot product with the light rays
        to deduce the angle of incidence of the ray upon the surface. From that angle,
        we can come up with a coefficient indicating how much a surface needs to be darkened.
        """
        if self.objectToRender.mesh is not None:
            shadowMesh = []
            vertices = self.objectToRender.vertices
            for triangle in self.objectToRender.mesh:
                # Gets two vectors of the current triangle and calculates the normal with a cross product
                vector1 = [vertices[triangle[1]][i] - vertices[triangle[0]][i] for i in range(3)]
                vector2 = [vertices[triangle[2]][i] - vertices[triangle[0]][i] for i in range(3)]
                normal = self.Normalize(np.cross(vector1, vector2))

                # Calculates the dot product and deduces the angle of incidence of the light rays
                incidenceAngle = np.arccos(np.dot(normal, self.lightRay))
                shadowCoef = max((incidenceAngle) / pi, MAX_SHADOW_COEF) # Deduces a shadow coefficient based on the angle
                color = hex(triangle[3])[2:] # Gets the current color of the triangle and converts it to hexadecimal
                while len(color) < 6:
                    color = '0' + color # Adding '0' if necessary to get in the proper format of length 6 (#XXXXXX)
                rgb = [hex(int(int(color[i*2:i*2+2], 16) * shadowCoef))[2:] for i in range(3)] # Scales each component by the shadow coefficient and gets the new darkened color
                for i in range(3):
                    if len(rgb[i]) == 1:
                        rgb[i] = '0' + rgb[i] # Formats new color
                newColor = int("".join(rgb), 16)
                shadowMesh.append((triangle[0], triangle[1], triangle[2], newColor)) # Updates the mesh
            self.objectToRender.mesh = shadowMesh
    

    def RenderObjects(self):
        """
        Calculates the position of the objects to render and displays them on the
        screen. First, all vertices are updated to their new position. Then, the 
        mesh is sorted by distance to the camera. 

        This is called the painter's algorithm and it makes sure that all surfaces are
        rendered in the correct order. However, the painter's algorithm still requires
        to draw all surfaces, one by one, even the ones that are hidden.

        This is why back face culling is applied afterwards. It determines which surfaces are
        facing the camera and only them are displayed. It reduces the number of surfaces to draw
        but doesn't work with non-convex solids.
        """
        if self.objectToRender.mesh is not None:
            updatedVertices = self.GetUpdatedObjectVertices(self.objectToRender)
            # Painter's algorithm - sorts triangles by distance to the camera (reversed)
            self.objectToRender.mesh.sort(key=lambda vertices : max(updatedVertices[vertices[0]][2], max(updatedVertices[vertices[1]][2], updatedVertices[vertices[2]][2])), reverse=True)

            for triangle in self.objectToRender.mesh:
                if abs(updatedVertices[triangle[0]][2]) < RENDER_DISTANCE:
                    # Back face culling - all triangles facing the wrong way are not drawn
                    # Here we try to determine if the current triangle if facing the correct way. For that, we take two vectors of said triangle and calculate a cross product, 
                    # giving us the normal vector to the surface
                    vector1 = [updatedVertices[triangle[1]][i] - updatedVertices[triangle[0]][i] for i in range(2)]
                    vector2 = [updatedVertices[triangle[2]][i] - updatedVertices[triangle[0]][i] for i in range(2)]
                    if vector1[0] * vector2[1] - vector1[1] * vector2[0] < 0: # Normal points towards the camera / z is negative
                        self.DrawTriangle(triangle, updatedVertices)
        

    def GetUpdatedObjectVertices(self, obj3d):
        """
        Returns an array of all the vertices after they've been updated to the new
        rotations and camera positions.

        obj3d is Object3D type.
        """
        newVertices = []

        if obj3d.vertices is not None:
            for vertex in obj3d.vertices:
                newVertices.append(self.GetUpdatedVector(vertex))
        
        return newVertices


    def UpdateBase(self):
        """
        Resets the base (x, y, and z axis) vectors and rotates them to update to the
        current rotation.
        """
        self.base = np.array([
            [CAM_SPEED, 0, 0],
            [0, CAM_SPEED, 0],
            [0, 0, CAM_SPEED]
        ])

        for i in range(3):
            # The rotation is performed backwards to compensate the objects rotation ('reversed' flag set to -1). 
            # Basically, all objects' vectors are rotated, including the camera, which depends on the base vectors.
            # The resulting effect is that when you move right or left, the base vectors are following the new 
            # rotated axis. To compensate, it is necessary to turn the base the other way around.
            self.base[i] = self.RotateVector(self.base[i], reversed=-1, baseFlag=1)


    def GetUpdatedVector(self, v):
        """
        Returns a given vector after all transformations : translations,
        rotations, perspective.

        v is a array with three components

        All transformatoins use numpy operations to speed things up.
        """
        return self.ApplyPerspective(self.RotateVector(self.TranslateVector(v)))


    def TranslateVector(self, v):
        """
        Returns a new vector whose coordinates are shifted by the current camera
        position.
        """
        translatedVector = np.array(v)
        
        for i in range(3):
            # Moves the vector along the camera position
            translatedVector[i] += self.camera[i]
        
        return translatedVector
    

    def RotateVector(self, v, reversed=1, baseFlag=-1):
        """
        Returns a rotated version of a given vector. For that, it uses rotation 
        matrices along theta and phi angles.

        The 'reversed' flag indicates if the rotation has to be calculated backwards
        The 'baseFlag' flag indicates a specific rotation for the base vectors, x, y and
        z, as they recquire a backwards rotation only on the phi rotation.
        """
        rotatedVector = np.array(v)

        # Gets rotation angles
        theta = reversed * self.rotations[0]
        phi = reversed * self.rotations[1]

        # Rotation along y axis (theta)
        rotatedVector = np.dot(rotatedVector, np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ]))

        if baseFlag == -1:
            # Rotation along x axis (phi)
            rotatedVector = np.dot(rotatedVector, np.array([
                [1, 0, 0],
                [0, np.cos(phi), -np.sin(phi)],
                [0, np.sin(phi), np.cos(phi)]
            ]))

        return rotatedVector
    

    def ApplyPerspective(self, v):
        """
        Gets the depth of a vector and scales it depending on that depth. Since the screen is a 
        rectangle and the focal point is smaller than the screen, we can't simply make an orthogonal
        projection (i.e drop the z coordinate) to be accurate. What is done here instead is a perspective
        projection.

        For each vector, we calculate the dimensions of the plane it's inscribed in. The vector's x and
        y coordinates are then appropriately scaled on the screen according to those dimensions.
        """
        pV = np.array(v)
        if v[2] != 0:
            # Perspective projection, calculates width and height of the plane the vector is inscribed in
            vectorSurfaceWidth = (SCREEN_DST + v[2]) * WIDTH / SCREEN_DST
            vectorSurfaceHeigth = (SCREEN_DST + v[2]) * HEIGHT / SCREEN_DST

            # Scaling coefficient
            pV[0] *= WIDTH / vectorSurfaceWidth
            pV[1] *= HEIGHT / vectorSurfaceHeigth
        
        return pV


    def UpdateCam(self):
        """
        Moves the camera in the direction and instensity of its acceleration.
        This acceleration decreases with time. Also rotates the camera by current
        rotations.
        """
        for i in range(3):
            self.camera[i] += self.acceleration[i]
            self.acceleration[i] *= 0.9
        for i in range(2):
            self.rotations[i] += self.rotationAcceleration[i]
            self.rotationAcceleration[i] *= 0.9

    

    def MoveCamX(self, reversed=1):
        """
        Moves the camera along the x axis. Takes into account rotations
        of the base. Setting the 'reversed' flag to -1 moves the camera
        into negative x values.
        """
        for i in range(3):
            self.acceleration[i] += reversed * self.base[0][i]
    

    def MoveCamY(self, reversed=1):
        """
        Moves the camera along the y axis. Takes into account rotations
        of the base.
        """
        for i in range(3):
            self.acceleration[i] += reversed * self.base[1][i]
    

    def MoveCamZ(self, reversed=1):
        """
        Moves the camera along the z axis. Takes into account rotations
        of the base.
        """
        for i in range(3):
            self.acceleration[i] += reversed * self.base[2][i]
     

    def DrawLine(self, v1, v2):
        """
        Displays a line between two given vectors indicating beginning and ending points. It cuts the line
        when it crosses the screen, i.e when their z coordinate is less than or equals 0.
        """
        if v1[2] >= 0 and v2[2] >= 0:
            pg.draw.line(self.screen, "#FFFFFF", (v1[0] + WIDTH / 2, v1[1] + HEIGHT / 2), (v2[0] + WIDTH / 2, v2[1] + HEIGHT / 2))
        elif v1[2] >= 0: # Avoids drawing vectors in -z values
            coef = - v1[2] / (v2[2] - v1[2])
            newV = [v1[0] + coef * (v2[0] - v1[0]), v1[1] + coef * (v2[1] - v1[1]), v1[2] + coef * (v2[2] - v1[2])] # Instead it cuts them when they hit the screen
            self.DrawLine(v1, newV)
        elif v2[2] >= 0: # Other case
            coef = - v2[2] / (v1[2] - v2[2])
            newV = [v2[0] + coef * (v1[0] - v2[0]), v2[1] + coef * (v1[1] - v2[1]), v2[2] + coef * (v1[2] - v2[2])]
            self.DrawLine(v2, newV)
    
    
    def DrawTriangle(self, triangle, vertices):
        """
        Displays a given triangle on the screen. All three vertices are extracted from
        the triangle provided. The triangle has a format (p1, p2, p3, color) where the color
        is the hex value converted into an integer ("#0001FF" -> 511).
        """
        # Reads points coordinates
        p1 = vertices[triangle[0]]
        p2 = vertices[triangle[1]]
        p3 = vertices[triangle[2]]

        # Reads color code in hex
        color = hex(triangle[3])[2:]
        while len(color) < 6:
            color = '0' + color
        color = '#' + color

        if p1[2] >= 0 and p2[2] >= 0 and p3[2] >= 0: # Avoids drawing triangles if they have negative z values
            pg.draw.polygon(self.screen, color, [(p1[0] + WIDTH / 2, p1[1] + HEIGHT / 2), (p2[0] + WIDTH / 2, p2[1] + HEIGHT / 2), (p3[0] + WIDTH / 2, p3[1] + HEIGHT / 2)])