from renderer import *
from polygons import *


class Main:
    """
    This class manages the different objects to render. It contains a surface to draw onto,
    a renderer (see the appropriate class for more info), and a loop to run the engine.
    It also reads key and mouse events to update in real time what appears on the screen.
    """
    def __init__(self):
        pg.init()

        # Display initialization & variables
        self.screen = pg.display.set_mode([WIDTH, HEIGHT])
        pg.display.set_caption("3D project")
        self.clock = pg.time.Clock()
        self.running = True
        pg.mouse.set_visible(False)
        pg.mouse.set_pos([WIDTH/2, HEIGHT/2])

        # Test objects
        self.objects = []
        self.GetObjects()
        self.MergeObjects()

        objectCopy = Object3D(pos=(5000, 0, 7000))
        objectCopy.CopyStructure(self.objects[0])
        self.objects.append(objectCopy)

        self.MergeObjects()
        
        self.objects = self.objects[0]
            
        self.rend = Renderer(self.screen, self.objects)
        self.rend.ApplyShadows()


    def Loop(self):
        """
        Runs the methods necessary to update the scene. First it reads all key
        and mouse events, then it calls the renderer to update the vertices positions,
        and then it displays the objects on the screen and loops.
        """
        while self.running:
            self.clock.tick(FPS)
            self.screen.fill("#000000") # Erases the screen
            self.ReadKeyEvents()
            self.ReadMouseMovements()

            ####  TEST ZONE

            self.rend.UpdateBase()
            self.rend.RenderObjects()
            self.rend.UpdateCam()

            ####  END TEST ZONE

            print(self.clock.get_fps())
            pg.display.flip()
    

    def GetObjects(self):
        """
        Opens the objects.txt file to read vertices and mesh of the objects and adds
        them to the 'self.objects' list.
        """
        with open("./objects.txt", 'r') as file:
            currVertices = []
            currMesh = []
            currTuple = ()
            currValue = ""
            flag = False
            commentFlag = False

            for line in file:
                for c in line: # Reads the file and catches special characters
                    if c == "?":
                        commentFlag = True
                    elif c == "\n":
                        commentFlag = False
                    if not commentFlag:
                        if c == '[':
                            if flag:
                                currMesh = []
                            else:
                                currVertices = []
                        elif c == ']':
                            flag = not flag
                        elif c == '(':
                            currTuple = ()
                        elif c == ')':
                            if not flag:
                                currVertices.append(currTuple)
                            else:
                                currMesh.append(currTuple)
                        elif c == ',':
                            if len(currTuple) < 3:
                                currTuple += (int(currValue),)
                            else:
                                currTuple += (int(currValue, 16),) # Converts hex color to decimal
                            currValue = ""
                        elif c == '}':
                            self.objects.append(Object3D(currVertices, currMesh))
                        elif c.upper() in "-0123456789ABCDEF.":
                            currValue += c
    

    def MergeObjects(self):
        """
        Merges the properties (vertices and mesh) of the objects to render in one big
        object. This is done so that the renderer can sort all triangles by distance 
        to the camera in one operation, which is required to display the surfaces in the 
        correct order.
        """
        mergedVertices = []
        mergedMeshes = []

        for objects in self.objects:
            for i in range(len(objects.mesh)):
                # The mesh consists of indices pointing to the vertices
                # Since all the vertices are merged in one array, the mesh
                # must be updated, which is done here ; all indices are shifted
                # by the current length of the vertices array.
                objects.mesh[i] = (objects.mesh[i][0] + len(mergedVertices),
                                   objects.mesh[i][1] + len(mergedVertices),
                                   objects.mesh[i][2] + len(mergedVertices),
                                   objects.mesh[i][3])
            mergedMeshes += objects.mesh
            mergedVertices += objects.vertices

        self.objects = [Object3D(mergedVertices, mergedMeshes)]
    

    def ReadMouseMovements(self):
        """
        Gets the position of the mouse pointer. From that position, it deduces the
        change in x (dx) and y (dy) axis. These distances are then converted to angular
        difference.
        """
        dx, dy = pg.mouse.get_rel()

        self.rend.rotations[0] += dx * MOUSE_SENSITIVITY * pi / WIDTH
        if abs(self.rend.rotations[1] + dy * MOUSE_SENSITIVITY * pi / HEIGHT) < pi / 2: 
            # Prevents from looking 360° on the y axis, essentially doing saltos or breaking your neck
            # Instead it caps when you look directly up or down
            self.rend.rotations[1] -= dy * MOUSE_SENSITIVITY * pi / HEIGHT

        # Resets mouse position at the center of the screen
        pg.mouse.set_pos([WIDTH/2, HEIGHT/2])
    

    def ReadKeyEvents(self):
        """
        Reads all the keys being pressed and call the binded methods. It reads
        both one time and continous key presses.
        """
        # Keys pressed continously
        keys = pg.key.get_pressed()
        if keys[pg.K_a]: # Looking left
            self.rend.rotationAcceleration[0] -= ROTATION_SPEED
        if keys[pg.K_e]: # Looking right
            self.rend.rotationAcceleration[0] += ROTATION_SPEED
        if keys[pg.K_DOWN]: # Looking down
            self.rend.rotationAcceleration[1] -= ROTATION_SPEED
        if keys[pg.K_UP]: # Looking up
            self.rend.rotationAcceleration[1] += ROTATION_SPEED
        if keys[pg.K_z]: # Moving forward
            self.rend.MoveCamZ(-1)
        if keys[pg.K_s]: # Moving backwards
            self.rend.MoveCamZ()
        if keys[pg.K_d]: # Moving right
            self.rend.MoveCamX(-1)
        if keys[pg.K_q]: # Moving left
            self.rend.MoveCamX()
        if keys[pg.K_LCTRL]: # Moving up
            self.rend.MoveCamY(-1)
        if keys[pg.K_SPACE]: # Moving down
            self.rend.MoveCamY()

        # One-time key press
        for event in pg.event.get():
            if event.type == pg.QUIT or event.type == pg.KEYDOWN and event.key == pg.K_ESCAPE:
                # Pressing the 'X' button or 'esc' kills the window
                self.running = False


# ---- MAIN PROGRAM ---- #


if __name__ == "__main__":
    m = Main()
    m.Loop()
    pg.quit() # Makes sure the window closes with the program