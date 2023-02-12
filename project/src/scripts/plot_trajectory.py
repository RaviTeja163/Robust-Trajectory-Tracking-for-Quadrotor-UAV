import numpy as np
import matplotlib.pyplot as plt

def traj_evaluate(start_pos, des_pos, T0, Tf, t):
    
    x0 = start_pos[0]
    xd0 = 0
    xdd0 = 0

    y0 = start_pos[1]
    yd0 = 0
    ydd0 = 0

    z0 = start_pos[2]
    zd0 = 0
    zdd0 = 0

    xf = des_pos[0]
    xdf = 0
    xddf = 0

    yf = des_pos[1]
    ydf = 0
    yddf = 0

    zf = des_pos[2]
    zdf = 0
    zddf = 0 
    
    A = np.array(
        [[1, T0, T0**2, T0**3, T0**4, T0**5],
            [0, 1, 2*T0, 3*T0**2, 4*T0**3, 5*T0**4],
            [0, 0, 2, 6*T0, 12*T0**2, 20*T0**3],
            [1, Tf, Tf**2, Tf**3, Tf**4, Tf**5],
            [0, 1, 2*Tf, 3*Tf**2, 4*Tf**3, 5*Tf**4],
            [0, 0, 2, 6*Tf, 12*Tf**2, 20*Tf**3],
        ])

    X = np.array(
        [[x0],
            [xd0],
            [xdd0],
            [xf],
            [xdf],
            [xddf]
        ])

    Y = np.array(
        [[y0],
            [yd0],
            [ydd0],
            [yf],
            [ydf],
            [yddf]
        ])

    Z = np.array(
        [[z0],
            [zd0],
            [zdd0],
            [zf],
            [zdf],
            [zddf]
        ])

    xc = np.linalg.solve(A, X)
    yc = np.linalg.solve(A, Y)
    zc = np.linalg.solve(A, Z)

    x = xc[5]*t**5 + xc[4]*t**4 + xc[3]*t**3 + xc[2]*t**2 + xc[1]*t + xc[0]
    xd = 5 * xc[5] *t**4 + 4 * xc[4]*t**3 + 3 * xc[3]*t**2 + 2 * xc[2]*t + xc[1]
    xdd = 20 * xc[5] *t**3 + 12 * xc[4]*t**2 + 6 * xc[3]*t + 2 * xc[2]

    y = yc[5]*t**5 + yc[4]*t**4 + yc[3]*t**3 + yc[2]*t**2 + yc[1]*t + yc[0]
    yd = 5 * yc[5] *t**4 + 4 * yc[4]*t**3 + 3 * yc[3]*t**2 + 2 * yc[2]*t + yc[1]
    ydd = 20 * yc[5] *t**3 + 12 * yc[4]*t**2 + 6 * yc[3]*t + 2 * yc[2]

    z = zc[5]*t**5 + zc[4]*t**4 + zc[3]*t**3 + zc[2]*t**2 + zc[1]*t + zc[0]
    zd = 5 * zc[5] *t**4 + 4 * zc[4]*t**3 + 3 * zc[3]*t**2 + 2 * zc[2]*t + zc[1]
    zdd = 20 * zc[5] *t**3 + 12 * zc[4]*t**2 + 6 * zc[3]*t + 2 * zc[2]

    return x, xd, xdd, y, yd, ydd, z, zd, zdd

if __name__ == '__main__':
    t1 = np.linspace(0,5)
    x1, xd1, xdd1, y1, yd1, ydd1, z1, zd1, zdd1 = traj_evaluate([0,0,0], [0,0,1], 0, 5, t1)

    t2 = np.linspace(5,20)
    x2, xd2, xdd2, y2, yd2, ydd2, z2, zd2, zdd2 = traj_evaluate([0,0,1], [1,0,1], 5, 20, t2)

    t3 = np.linspace(20,35)
    x3, xd3, xdd3, y3, yd3, ydd3, z3, zd3, zdd3 = traj_evaluate([1,0,1], [1,1,1], 20, 35, t3)

    t4 = np.linspace(35,50)
    x4, xd4, xdd4, y4, yd4, ydd4, z4, zd4, zdd4 = traj_evaluate([1,1,1], [0,1,1], 35, 50, t4)

    t5 = np.linspace(50,65)
    x5, xd5, xdd5, y5, yd5, ydd5, z5, zd5, zdd5 = traj_evaluate([0,1,1], [0,0,1], 50, 65, t5)

    t = np.concatenate((t1, t2, t3, t4, t5))
    x = np.concatenate((x1, x2, x3, x4, x5))
    xd = np.concatenate((xd1, xd2, xd3, xd4, xd5))
    xdd = np.concatenate((xdd1, xdd2, xdd3, xdd4, xdd5))

    y = np.concatenate((y1, y2, y3, y4, y5))
    yd = np.concatenate((yd1, yd2, yd3, yd4, yd5))
    ydd = np.concatenate((ydd1, ydd2, ydd3, ydd4, ydd5))

    z = np.concatenate((z1, z2, z3, z4, z5))
    zd = np.concatenate((zd1, zd2, zd3, zd4, zd5))
    zdd = np.concatenate((zdd1, zdd2, zdd3, zdd4, zdd5))



    plt.figure()
    plt.plot(t,x)
    plt.xlabel('time (s)')
    plt.ylabel('x Position')

    plt.figure()
    plt.plot(t,y)
    plt.xlabel('time (s)')
    plt.ylabel('y Position')

    plt.figure()
    plt.plot(t,z)
    plt.xlabel('time (s)')
    plt.ylabel('z Position')

    plt.figure()
    plt.plot(t,xd)
    plt.xlabel('time (s)')
    plt.ylabel('x Velocity')

    plt.figure()
    plt.plot(t,yd)
    plt.xlabel('time (s)')
    plt.ylabel('y Velocity')

    plt.figure()
    plt.plot(t,zd)
    plt.xlabel('time (s)')
    plt.ylabel('z Velocity')

    plt.figure()
    plt.plot(t,xdd)
    plt.xlabel('time (s)')
    plt.ylabel('x Acceleration')

    plt.figure()
    plt.plot(t,ydd)
    plt.xlabel('time (s)')
    plt.ylabel('y Acceleration')

    plt.figure()
    plt.plot(t,zdd)
    plt.xlabel('time (s)')
    plt.ylabel('z Acceleration')
    plt.show()
