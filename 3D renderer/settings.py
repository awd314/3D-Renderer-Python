from numpy import tan, pi


WIDTH, HEIGHT = 1920, 1200 # Window dimensions
FPS = 60
FOV = pi/2 # FOV in radians
SCREEN_DST = WIDTH / (2 * tan(FOV / 2)) # Distance between the focal point and the screen onto which the scene is projected
MOUSE_SENSITIVITY = 1
CAM_SPEED = 15 * 30 / FPS
ROTATION_SPEED = pi / 1000
RENDER_DISTANCE = 1e12

MAX_SHADOW_COEF = 0.1 # How dark a surface can get at maximum