from numpy import tan, pi


WIDTH, HEIGHT = 1920, 1200 # Window dimensions
FPS = 60
FOV = pi/3 # FOV in radians
SCREEN_DST = WIDTH / (2 * tan(FOV / 2)) # Distance between the focal point and the screen onto which the scene is projected
MOUSE_SENSITIVITY = 1
CAM_SPEED = 120 * 30 / FPS