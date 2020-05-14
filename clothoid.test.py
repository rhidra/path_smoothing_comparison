import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import math



def main():
    x0,y0,theta0 = 0,0, 3.14/2
    x1,y1,theta1 = 1,1, 3.14*3/2

    c0, c1 = theta0, theta1
    s = np.linspace(0, 1, 1000)

    def clothoid_ode_rhs(state, s):
        return np.array([np.cos(state[2]), np.sin(state[2]), theta1*s])
    def eval_clothoid(x0,y0,theta0, s):
        return odeint(clothoid_ode_rhs, np.array([x0,y0,theta0]), s)


    sol = eval_clothoid(x0, y0, theta0, s)

    xs, ys, thetas = sol[:,0], sol[:,1], sol[:,2]
    plt.plot(xs, ys)
    print(thetas)

    plt.gcf().gca().add_artist(plt.Circle((0,0), 1, color='b', fill=False))
    plt.xlim(-1.25,1.25)
    plt.ylim(-1.25,1.25)
    plt.show()

if __name__=='__main__':
    main()
