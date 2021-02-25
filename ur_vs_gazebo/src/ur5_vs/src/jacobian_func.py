# The jacobian for the UR5, using the standard DH parameters. Generated from Peter Corke's Robotics Toolbox in MATLAB, copied to python.

import numpy as np
from math import *
import math as m

d1 = 0.0895
d2 = 0
d3 = 0
d4 = 0.1092
d5 = 0.0947
d6 = 0.0823

a1 = 0
a2 = -0.425
a3 = -0.3923
a4 = 0
a5 = 0
a6 = 0


def calc_jack(th1,th2,th3,th4,th5,th6):
    
    j11=d5*m.sin(th2 + th3 + th4)*m.sin(th1) - d4*m.cos(th1) - a2*m.cos(th2)*m.sin(th1) - d6*(m.cos(th1)*m.cos(th5) + m.cos(th2 + th3 + th4)*m.sin(th1)*m.sin(th5)) - a3*m.cos(th2)*m.cos(th3)*m.sin(th1) + a3*m.sin(th1)*m.sin(th2)*m.sin(th3)

    j12=-m.cos(th1)*(a3*m.sin(th2 + th3) - d5*(m.sin(th2 + th3)*m.sin(th4) - m.cos(th2 + th3)*m.cos(th4)) + a2*m.sin(th2) + d6*m.sin(th5)*(m.cos(th2 + th3)*m.sin(th4) + m.sin(th2 + th3)*m.cos(th4)))

    j13=-m.cos(th1)*(a3*m.sin(th2 + th3) + d5*m.cos(th2 + th3 + th4) + d6*m.sin(th2 + th3 + th4)*m.sin(th5))

    j14=-m.cos(th1)*(d5*m.cos(th2 + th3 + th4) + d6*m.sin(th2 + th3 + th4)*m.sin(th5))

    j15=d6*m.sin(th1)*m.sin(th5) - d6*m.cos(th1)*m.cos(th2)*m.cos(th5)*m.sin(th3)*m.sin(th4) - d6*m.cos(th1)*m.cos(th3)*m.cos(th5)*m.sin(th2)*m.sin(th4) - d6*m.cos(th1)*m.cos(th4)*m.cos(th5)*m.sin(th2)*m.sin(th3) + d6*m.cos(th1)*m.cos(th2)*m.cos(th3)*m.cos(th4)*m.cos(th5)

    j16=0



    j21=a2*m.cos(th1)*m.cos(th2) - d4*m.sin(th1) - d6*m.cos(th5)*m.sin(th1) - d5*m.sin(th2 + th3 + th4)*m.cos(th1) + a3*m.cos(th1)*m.cos(th2)*m.cos(th3) - a3*m.cos(th1)*m.sin(th2)*m.sin(th3) + d6*m.cos(th2 + th3 + th4)*m.cos(th1)*m.sin(th5)

    j22=-m.sin(th1)*(a3*m.sin(th2 + th3) - d5*(m.sin(th2 + th3)*m.sin(th4) - m.cos(th2 + th3)*m.cos(th4)) + a2*m.sin(th2) + d6*m.sin(th5)*(m.cos(th2 + th3)*m.sin(th4) + m.sin(th2 + th3)*m.cos(th4)))

    j23=-m.sin(th1)*(a3*m.sin(th2 + th3) + d5*m.cos(th2 + th3 + th4) + d6*m.sin(th2 + th3 + th4)*m.sin(th5))

    j24=-m.sin(th1)*(d5*m.cos(th2 + th3 + th4) + d6*m.sin(th2 + th3 + th4)*m.sin(th5))

    j25=d6*m.cos(th2)*m.cos(th3)*m.cos(th4)*m.cos(th5)*m.sin(th1) - d6*m.cos(th1)*m.sin(th5) - d6*m.cos(th2)*m.cos(th5)*m.sin(th1)*m.sin(th3)*m.sin(th4) - d6*m.cos(th3)*m.cos(th5)*m.sin(th1)*m.sin(th2)*m.sin(th4) - d6*m.cos(th4)*m.cos(th5)*m.sin(th1)*m.sin(th2)*m.sin(th3)

    j26=0

	
    j31=0

    j32=(d6*m.sin(th2 + th3 + th4 - th5))/2 - a3*m.cos(th2 + th3) - a2*m.cos(th2) - (d6*m.sin(th2 + th3 + th4 + th5))/2 + d5*m.sin(th2 + th3 + th4)

    j33=(d6*m.sin(th2 + th3 + th4 - th5))/2 - a3*m.cos(th2 + th3) - (d6*m.sin(th2 + th3 + th4 + th5))/2 + d5*m.sin(th2 + th3 + th4)

    j34=(d6*m.sin(th2 + th3 + th4 - th5))/2 - (d6*m.sin(th2 + th3 + th4 + th5))/2 + d5*m.sin(th2 + th3 + th4)

    j35=-d6*(m.sin(th2 + th3 + th4 - th5)/2 + m.sin(th2 + th3 + th4 + th5)/2)

    j36=0


    j41=0

    j42=-m.sin(th1)

    j43=-m.sin(th1)

    j44=-m.sin(th1)

    j45=-m.sin(th2 + th3 + th4)*m.cos(th1)

    j46=m.cos(th2 + th3 + th4)*m.cos(th1)*m.sin(th5) - m.cos(th5)*m.sin(th1)


    j51=0

    j52=m.cos(th1)

    j53=m.cos(th1)

    j54=m.cos(th1)

    j55=-m.sin(th2 + th3 + th4)*m.sin(th1)

    j56=m.cos(th1)*m.cos(th5) + m.cos(th2 + th3 + th4)*m.sin(th1)*m.sin(th5)


    j61=1

    j62=0

    j63=0

    j64=0

    j65=-m.cos(th2 + th3 + th4)

    j66=-m.sin(th2 + th3 + th4)*m.sin(th5)
	
	
    jacobian = [[j11,j12,j13,j14,j15,j16],[j21,j22,j23,j24,j25,j26],[j31,j32,j33,j34,j35,j36],[j41,j42,j43,j44,j45,j46],[j51,j52,j53,j54,j55,j56],[j61,j62,j63,j64,j65,j66]]	
	
    jacobian = np.linalg.pinv(jacobian)


    return jacobian

