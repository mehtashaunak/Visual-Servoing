import numpy as np
import cv2
import math as m


def grad(Ugx,Ugy,ux,uy,var):
    x = Ugx
    y = Ugy
    mu_x = ux 
    mu_y = uy

    Sigma1 = var
    Sigma2 = 0.01
    Sigma3 = 0.01
    Sigma4 = var

    Sigma = np.array([[Sigma1,Sigma2],[Sigma3,Sigma4]])
    nu = 2*var/(var-1)
    d = 2
    A1 = m.gamma((nu+d)/2)
    A2 = np.linalg.det(Sigma)

    A = A1*m.pow(A2,-0.5)

    B1 = m.pi*nu
    B2 = m.gamma(nu/2)
    B = m.pow(B1,d/2)*B2

    C = (nu+d)/2
    u = np.array([x-mu_x,y-mu_y])
    mahal = np.matmul((np.matmul(u,np.linalg.inv(Sigma))),u.T)

    Fxy = 1 + mahal/nu

    dfx = -2*Sigma1*(mu_x-x) - Sigma2*(mu_y-y) - Sigma3*(mu_y-y)
    dfy = -Sigma2*(mu_x-x) - Sigma3*(mu_x-x) - 2*Sigma4*(mu_y-y)


    grad_x = (-A*C/B)*m.pow(Fxy,-C-1)*dfx
    grad_y = (-A*C/B)*m.pow(Fxy,-C-1)*dfy

    G = np.array([grad_x,grad_y])
    
    return G