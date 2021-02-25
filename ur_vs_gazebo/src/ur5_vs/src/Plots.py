
import time
import math as m
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc
Varm0 = []
Varm1 = []
Varm2 = []
Varm3 = []
Varm4 = []
Varm5 = []
er = []
th0 = []
th1 = []
th2 = []
th3 = []
th4 = []
th5 = []
r11 = []
c11 = []
r22 = []
c22 = []
r33 = []
c33 = []
r44 = []
c44 = []
X = []
Y = []
Z = []
time1 = []
i = 0
u_des1 = 261
v_des1 = 203
u_des2 = 318
v_des2 = 203
u_des3 = 262
v_des3 = 259
u_des4 = 317
v_des4 = 259
initial_time=time.time()
def plot(Varm,error,th,r1,c1,r2,c2,r3,c3,r4,c4,trans):
    global i
    global Varm0,Varm1,Varm2,Varm3,Varm4,Varm5
    global th0,th1,th2,th3,th4,th5
    global r11,r22,r33,r44,c11,c22,c33,c44
    global er
    global X,Y,Z
    global time1
    er.append(m.sqrt((error.item(0)*error.item(0)+error.item(1)*error.item(1)+error.item(2)*error.item(2)+error.item(3)*error.item(3)+error.item(4)*error.item(4)+error.item(5)*error.item(5))/6))
    th0.append(th.item(0))
    th1.append(th.item(1))
    th2.append(th.item(2))
    th3.append(th.item(3))
    th4.append(th.item(4))
    th5.append(th.item(5))
    Varm0.append(Varm.item(0))
    Varm1.append(Varm.item(1))
    Varm2.append(Varm.item(2))
    Varm3.append(Varm.item(3))
    Varm4.append(Varm.item(4))
    Varm5.append(Varm.item(5))
    r11.append(r1)
    r22.append(r2)
    r33.append(r3)
    r44.append(r4)
    c11.append(c1)
    c22.append(c2)
    c33.append(c3)
    c44.append(c4)

    X.append(trans.item(0,3))
    Y.append(trans.item(1,3))
    Z.append(trans.item(2,3))    
    time1.append(time.time()-initial_time)
    if i == 700:
        
    	fig1 =  plt.figure()
        #plt.rc('font', family='serif')
        #plt.rc('xtick', labelsize='x-small')
        #plt.rc('ytick', labelsize='x-small')
        plt.plot(time1,Varm0, 'r',ls='solid', label='$\dot \Theta_1$')
    	plt.plot(time1,Varm1, 'g', label='$\dot \Theta_2$')
    	plt.plot(time1,Varm2, 'b', label='$\dot \Theta_3$')
    	plt.plot(time1,Varm3, 'c', label='$\dot \Theta_4$')
    	plt.plot(time1,Varm4, 'y', label='$\dot \Theta_5$')
    	plt.plot(time1,Varm5, 'k', label='$\dot \Theta_6$')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Velocities (rad/s)')
    	plt.legend(frameon=False)
    	plt.autoscale(enable=True, axis = 'both',tight = None)
        
        plt.savefig('Joint_vel.png', transperent=True)


        fig2 = plt.figure()
        plt.plot(time1,er,'b', label= 'mean error')
        plt.xlabel('Time (s)')
        plt.ylabel('Norm Error (Pixels)')
        
        plt.savefig('Mean_err.png', transperent=True)


        fig3 = plt.figure()
        plt.plot(time1,th0,'r', label='$\Theta_1$')
    	plt.plot(time1,th1,'g', label='$\Theta_2$')
        plt.plot(time1,th2,'b', label='$\Theta_3$')
        plt.plot(time1,th3,'c', label='$\Theta_4$')
        plt.plot(time1,th4,'y', label='$\Theta_5$')
        plt.plot(time1,th5,'k', label='$\Theta_6$')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Angle (rad)')
        plt.legend(frameon=False)
        
        plt.savefig('Joint_Ang.png', transperent=True)


        fig4 = plt.figure()
        plt.plot(u_des2,v_des2, 'o' ,markersize=10)
        plt.plot(u_des4,v_des4, 'o', markersize=10)
        plt.plot(u_des1,v_des1, 'o', markersize=10)
        plt.plot(u_des3,v_des3, 'o', markersize=10)
        plt.plot(r11,c11,'r')
        plt.plot(r22,c22,'b')
        plt.plot(r33,c33,'y')
        plt.plot(r44,c44,'g')
        plt.xlabel('x-axis (Pixels)')
        plt.ylabel('y-axis (Pixels)')
        
        plt.savefig('Image_feat.png', transperent=True)

        fig5 = plt.figure()
        ax = fig5.gca(projection='3d')
        ax.plot(X,Y,Z)
        plt.title('Camera Position')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        
        plt.savefig('Camera_Position.png', transperent=True)
        print('SAVED')

    i = i+1