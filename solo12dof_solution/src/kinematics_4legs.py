import numpy as np
import math
angls = np.array[math.pi/4, math.pi/6, math.pi/12] #[roll pitch yaw] Cardanian angles
p = random_foots_pos(angls)
def ikin(p, angls):