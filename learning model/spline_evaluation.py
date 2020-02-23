def spline_evalution(M,tx_i,x_i,vx_i,ax_i,jx_i,y_i,vy_i,ay_i,jy_i):
    import pylab as plt

    # Curve between the sampled points is approximated with a quintic spline: x(t) = a + b*t+c*t2+d*t3+e*t4+f*t5
    # This is used to increase the amount of samples used in each signal and enhance the integration of the features.
    # M-1 is the number of extra points put in the (tk-ti) time interval
    dt = tx_i[1] - tx_i[0]
    DT = dt/M

    x = []
    vx = []
    ax = []
    jx = []
    y = []
    vy = []
    ay = []
    jy = []

    # Evaluation of the coefficients x:
    for j in plt.arange(1, len(x_i), 1):
        i = j - 1
        k = j
        ti = i * dt
        tk = k * dt
        pi = x_i[i]
        pk = x_i[k]
        vi = vx_i[i]
        vk = vx_i[k]
        ai = ax_i[i]
        ak = ax_i[k]
        ji = jx_i[i]
        jk = jx_i[k]

        x.append(pi)
        vx.append(vi)
        ax.append(ai)
        jx.append(ji)
        # solution of coefficiënts in closed form
        a = -(2 * pi * tk ** 5 - 2 * pk * ti ** 5 + 10 * pk * ti ** 4 * tk - 2 * ti * tk ** 5 * vi + 2 * ti ** 5 * tk * vk + 20 * pi * ti ** 2 * tk ** 3 + ai * ti ** 2 * tk ** 5 - 2 * ai * ti ** 3 * tk ** 4 + ai * ti ** 4 * tk ** 3 - ak * ti ** 3 * tk ** 4 + 2 * ak * ti ** 4 * tk ** 3 - ak * ti ** 5 * tk ** 2 - 20 * pk * ti ** 3 * tk ** 2 + 10 * ti ** 2 * tk ** 4 * vi - 8 * ti ** 3 * tk ** 3 * vi + 8 * ti ** 3 * tk ** 3 * vk - 10 * ti ** 4 * tk ** 2 * vk - 10 * pi * ti * tk ** 4) / (2 * (ti - tk) ** 2 * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        b = (2 * ti ** 5 * vk - 2 * tk ** 5 * vi + 2 * ai * ti * tk ** 5 - 2 * ak * ti ** 5 * tk + 10 * ti * tk ** 4 * vi - 10 * ti ** 4 * tk * vk + 60 * pi * ti ** 2 * tk ** 2 - ai * ti ** 2 * tk ** 4 - 4 * ai * ti ** 3 * tk ** 3 + 3 * ai * ti ** 4 * tk ** 2 - 3 * ak * ti ** 2 * tk ** 4 + 4 * ak * ti ** 3 * tk ** 3 + ak * ti ** 4 * tk ** 2 - 60 * pk * ti ** 2 * tk ** 2 + 16 * ti ** 2 * tk ** 3 * vi - 24 * ti ** 3 * tk ** 2 * vi + 24 * ti ** 2 * tk ** 3 * vk - 16 * ti ** 3 * tk ** 2 * vk) / (2 * (ti - tk) ** 2 * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        c = -(ai * tk ** 5 - ak * ti ** 5 + 4 * ai * ti * tk ** 4 + 3 * ai * ti ** 4 * tk - 3 * ak * ti * tk ** 4 - 4 * ak * ti ** 4 * tk - 60 * pk * ti * tk ** 2 - 60 * pk * ti ** 2 * tk + 36 * ti * tk ** 3 * vi - 24 * ti ** 3 * tk * vi + 24 * ti * tk ** 3 * vk - 36 * ti ** 3 * tk * vk - 8 * ai * ti ** 2 * tk ** 3 + 8 * ak * ti ** 3 * tk ** 2 - 12 * ti ** 2 * tk ** 2 * vi + 12 * ti ** 2 * tk ** 2 * vk + 60 * pi * ti * tk ** 2 + 60 * pi * ti ** 2 * tk) / (2 * (ti - tk) ** 2 * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        d = (20 * pi * ti ** 2 + 20 * pi * tk ** 2 + ai * ti ** 4 + 3 * ai * tk ** 4 - 3 * ak * ti ** 4 - ak * tk ** 4 - 20 * pk * ti ** 2 - 20 * pk * tk ** 2 - 8 * ti ** 3 * vi - 12 * ti ** 3 * vk + 12 * tk ** 3 * vi + 8 * tk ** 3 * vk + 4 * ai * ti ** 3 * tk - 4 * ak * ti * tk ** 3 + 28 * ti * tk ** 2 * vi - 32 * ti ** 2 * tk * vi + 32 * ti * tk ** 2 * vk - 28 * ti ** 2 * tk * vk - 8 * ai * ti ** 2 * tk ** 2 + 8 * ak * ti ** 2 * tk ** 2 + 80 * pi * ti * tk - 80 * pk * ti * tk) / (2 * (ti ** 2 - 2 * ti * tk + tk ** 2) * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        e = -(30 * pi * ti + 30 * pi * tk - 30 * pk * ti - 30 * pk * tk + 2 * ai * ti ** 3 + 3 * ai * tk ** 3 - 3 * ak * ti ** 3 - 2 * ak * tk ** 3 - 14 * ti ** 2 * vi - 16 * ti ** 2 * vk + 16 * tk ** 2 * vi + 14 * tk ** 2 * vk - 4 * ai * ti * tk ** 2 - ai * ti ** 2 * tk + ak * ti * tk ** 2 + 4 * ak * ti ** 2 * tk - 2 * ti * tk * vi + 2 * ti * tk * vk) / (2 * (ti - tk) ** 2 * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        f = -(12 * pk - 12 * pi + 6 * ti * vi + 6 * ti * vk - 6 * tk * vi - 6 * tk * vk - ai * ti ** 2 - ai * tk ** 2 + ak * ti ** 2 + ak * tk ** 2 + 2 * ai * ti * tk - 2 * ak * ti * tk) / (2 * (ti ** 2 - 2 * ti * tk + tk ** 2) * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        # Building the signal
        for m in plt.arange(1, M, 1):
            time = ti + m*DT
            x_it = a + b*time+c*time**2+d*time**3+e*time**4+f*time**5
            vx_it = b + 2*c * time  + 3*d * time ** 2 + 4*e * time ** 3 + 5*f * time ** 4
            ax_it =  2 * c  + 6 * d * time + 12 * e * time ** 2 + 20 * f * time ** 3
            jx_it =  6 * d  + 24 * e * time  + 60 * f * time ** 2

            x.append(x_it)
            vx.append(vx_it)
            ax.append(ax_it)
            jx.append(jx_it)

    x.append(pk)
    vx.append(vk)
    ax.append(ak)
    jx.append(jk)



    # Evaluation of the coefficients y:
    for j in plt.arange(1, len(x_i), 1):
        i = j - 1
        k = j
        ti = i * dt
        tk = k * dt
        pi = y_i[i]
        pk = y_i[k]
        vi = vy_i[i]
        vk = vy_i[k]
        ai = ay_i[i]
        ak = ay_i[k]
        ji = jy_i[i]
        jk = jy_i[k]

        y.append(pi)
        vy.append(vi)
        ay.append(ai)
        jy.append(ji)

        # solution of coefficiënts in closed form
        a = -(2 * pi * tk ** 5 - 2 * pk * ti ** 5 + 10 * pk * ti ** 4 * tk - 2 * ti * tk ** 5 * vi + 2 * ti ** 5 * tk * vk + 20 * pi * ti ** 2 * tk ** 3 + ai * ti ** 2 * tk ** 5 - 2 * ai * ti ** 3 * tk ** 4 + ai * ti ** 4 * tk ** 3 - ak * ti ** 3 * tk ** 4 + 2 * ak * ti ** 4 * tk ** 3 - ak * ti ** 5 * tk ** 2 - 20 * pk * ti ** 3 * tk ** 2 + 10 * ti ** 2 * tk ** 4 * vi - 8 * ti ** 3 * tk ** 3 * vi + 8 * ti ** 3 * tk ** 3 * vk - 10 * ti ** 4 * tk ** 2 * vk - 10 * pi * ti * tk ** 4) / (2 * (ti - tk) ** 2 * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        b = (2 * ti ** 5 * vk - 2 * tk ** 5 * vi + 2 * ai * ti * tk ** 5 - 2 * ak * ti ** 5 * tk + 10 * ti * tk ** 4 * vi - 10 * ti ** 4 * tk * vk + 60 * pi * ti ** 2 * tk ** 2 - ai * ti ** 2 * tk ** 4 - 4 * ai * ti ** 3 * tk ** 3 + 3 * ai * ti ** 4 * tk ** 2 - 3 * ak * ti ** 2 * tk ** 4 + 4 * ak * ti ** 3 * tk ** 3 + ak * ti ** 4 * tk ** 2 - 60 * pk * ti ** 2 * tk ** 2 + 16 * ti ** 2 * tk ** 3 * vi - 24 * ti ** 3 * tk ** 2 * vi + 24 * ti ** 2 * tk ** 3 * vk - 16 * ti ** 3 * tk ** 2 * vk) / (2 * (ti - tk) ** 2 * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        c = -(ai * tk ** 5 - ak * ti ** 5 + 4 * ai * ti * tk ** 4 + 3 * ai * ti ** 4 * tk - 3 * ak * ti * tk ** 4 - 4 * ak * ti ** 4 * tk - 60 * pk * ti * tk ** 2 - 60 * pk * ti ** 2 * tk + 36 * ti * tk ** 3 * vi - 24 * ti ** 3 * tk * vi + 24 * ti * tk ** 3 * vk - 36 * ti ** 3 * tk * vk - 8 * ai * ti ** 2 * tk ** 3 + 8 * ak * ti ** 3 * tk ** 2 - 12 * ti ** 2 * tk ** 2 * vi + 12 * ti ** 2 * tk ** 2 * vk + 60 * pi * ti * tk ** 2 + 60 * pi * ti ** 2 * tk) / (2 * (ti - tk) ** 2 * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        d = (20 * pi * ti ** 2 + 20 * pi * tk ** 2 + ai * ti ** 4 + 3 * ai * tk ** 4 - 3 * ak * ti ** 4 - ak * tk ** 4 - 20 * pk * ti ** 2 - 20 * pk * tk ** 2 - 8 * ti ** 3 * vi - 12 * ti ** 3 * vk + 12 * tk ** 3 * vi + 8 * tk ** 3 * vk + 4 * ai * ti ** 3 * tk - 4 * ak * ti * tk ** 3 + 28 * ti * tk ** 2 * vi - 32 * ti ** 2 * tk * vi + 32 * ti * tk ** 2 * vk - 28 * ti ** 2 * tk * vk - 8 * ai * ti ** 2 * tk ** 2 + 8 * ak * ti ** 2 * tk ** 2 + 80 * pi * ti * tk - 80 * pk * ti * tk) / (2 * (ti ** 2 - 2 * ti * tk + tk ** 2) * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        e = -(30 * pi * ti + 30 * pi * tk - 30 * pk * ti - 30 * pk * tk + 2 * ai * ti ** 3 + 3 * ai * tk ** 3 - 3 * ak * ti ** 3 - 2 * ak * tk ** 3 - 14 * ti ** 2 * vi - 16 * ti ** 2 * vk + 16 * tk ** 2 * vi + 14 * tk ** 2 * vk - 4 * ai * ti * tk ** 2 - ai * ti ** 2 * tk + ak * ti * tk ** 2 + 4 * ak * ti ** 2 * tk - 2 * ti * tk * vi + 2 * ti * tk * vk) / (2 * (ti - tk) ** 2 * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        f = -(12 * pk - 12 * pi + 6 * ti * vi + 6 * ti * vk - 6 * tk * vi - 6 * tk * vk - ai * ti ** 2 - ai * tk ** 2 + ak * ti ** 2 + ak * tk ** 2 + 2 * ai * ti * tk - 2 * ak * ti * tk) / (2 * (ti ** 2 - 2 * ti * tk + tk ** 2) * (ti ** 3 - 3 * ti ** 2 * tk + 3 * ti * tk ** 2 - tk ** 3))
        # Building the signal
        for m in plt.arange(1, M, 1):
            time = ti + m * DT
            y_it = a + b * time + c * time ** 2 + d * time ** 3 + e * time ** 4 + f * time ** 5
            vy_it = b + 2 * c * time + 3 * d * time ** 2 + 4 * e * time ** 3 + 5 * f * time ** 4
            ay_it = 2 * c + 6 * d * time + 12 * e * time ** 2 + 20 * f * time ** 3
            jy_it = 6 * d + 24 * e * time + 60 * f * time ** 2

            y.append(y_it)
            vy.append(vy_it)
            ay.append(ay_it)
            jy.append(jy_it)

    y.append(pk)
    vy.append(vk)
    ay.append(ak)
    jy.append(jk)

    x = plt.array(x)
    vx = plt.array(vx)
    ax = plt.array(ax)
    jx = plt.array(jx)

    y = plt.array(y)
    vy = plt.array(vy)
    ay = plt.array(ay)
    jy = plt.array(jy)

    # Check of the calculated signals
    time_vector = plt.arange(0,tx_i[-1]+DT,DT)
    plt.figure("Comparinson of the calculated signals " , figsize=(10, 4))
    plt.subplot(2, 4, 1)
    plt.plot(tx_i,x_i)
    plt.plot(time_vector, x)
    plt.xlabel("Time [s]", fontsize=14)
    plt.grid(True)
    plt.title('x(t)', fontsize=14)

    plt.subplot(2, 4, 2)
    plt.plot(tx_i, vx_i)
    plt.plot(time_vector, vx)
    plt.xlabel("Time [s]", fontsize=14)
    plt.grid(True)
    plt.title('vx(t)', fontsize=14)

    plt.subplot(2, 4, 3)
    plt.plot(tx_i, ax_i)
    plt.plot(time_vector, ax)
    plt.xlabel("Time [s]", fontsize=14)
    plt.grid(True)
    plt.title('ax(t)', fontsize=14)

    plt.subplot(2, 4, 4)
    plt.plot(tx_i, jx_i)
    plt.plot(time_vector, jx)
    plt.xlabel("Time [s]", fontsize=14)
    plt.grid(True)
    plt.title('jx(t)', fontsize=14)

    plt.subplot(2, 4, 5)
    plt.plot(tx_i,y_i)
    plt.plot(time_vector, y)
    plt.xlabel("Time [s]", fontsize=14)
    plt.grid(True)
    plt.title('y(t)', fontsize=14)

    plt.subplot(2, 4, 6)
    plt.plot(tx_i, vy_i)
    plt.plot(time_vector, vy)
    plt.xlabel("Time [s]", fontsize=14)
    plt.grid(True)
    plt.title('vy(t)', fontsize=14)

    plt.subplot(2, 4, 7)
    plt.plot(tx_i, ay_i)
    plt.plot(time_vector, ay)
    plt.xlabel("Time [s]", fontsize=14)
    plt.grid(True)
    plt.title('ay(t)', fontsize=14)

    plt.subplot(2, 4, 8)
    plt.plot(tx_i, jy_i)
    plt.plot(time_vector, jy)
    plt.xlabel("Time [s]", fontsize=14)
    plt.grid(True)
    plt.title('jy(t)', fontsize=14)



    return x, vx, ax, jx, y, vy, ay, jy
