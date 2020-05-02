def derivative(a_list,dt):
    import pylab as plt
    a_list = plt.squeeze(a_list)
    j_list = []
    for i in plt.arange(0, len(a_list), 1):
        if i == 0:
            j_list.append((a_list[i + 1] - a_list[i]) / dt)
        elif i == len(a_list) - 1:
            j_list.append((a_list[i] - a_list[i - 1]) / dt)
        else:
            j_list.append((a_list[i + 1] - a_list[i - 1]) / (2 * dt))

    return plt.array(j_list)


