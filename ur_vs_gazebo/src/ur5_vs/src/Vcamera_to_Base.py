
import numpy as np
import math as m


d = np.array([0.089459,0,0,0.10915,0.09465,0.0823])
a = np.array([0,0.42500,0.39225,0,0,0])
alp = np.array([3.142/2, 0, 0, 3.142/2, -3.142/2, 0])

def Vc_2_Base(Vcamera,th):
    i1 = 0
	
    T = np.identity(4)
    trans = np.identity(4)
    for i1 in range (0,6):
    	T = np.array([[m.cos(th[i1]) , -m.sin(th[i1])*m.cos(alp[i1]) , m.sin(th[i1])*m.sin(alp[i1]) , a[i1]*m.cos(th[i1])] , [m.sin(th[i1]) ,m.cos(th[i1])*m.cos(alp[i1]) , -m.cos(th[i1])*m.sin(alp[i1]) , a[i1]*m.sin(th[i1])] , [0 , m.sin(alp[i1]) , m.cos(alp[i1]) , d[i1]] , [0 , 0 , 0 , 1]])
		
    	trans = np.matmul(trans,T)
		
    Rot = np.array([[trans.item(0,0), trans.item(0,1), trans.item(0,2)], [trans.item(1,0), trans.item(1,1), trans.item(1,2)], [trans.item(2,0), trans.item(2,1), trans.item(2,2)]])
	
    Rot = np.linalg.inv(Rot)
	
    V1 = np.array([Vcamera[0], Vcamera[1], Vcamera[2]])
    V2 = np.array([Vcamera[3], Vcamera[4], Vcamera[5]])
	
    V11 = np.matmul(Rot,V1)
    V22 = np.matmul(Rot,V2)
	
    Vbase = np.array([V11[0], V11[1],  V11[2], V22[0], V22[1], V22[2]]) 

    return Vbase