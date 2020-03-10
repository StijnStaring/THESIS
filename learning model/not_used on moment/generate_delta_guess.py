def generate_delta_guess(T_guess,samples):

    """
    Generation of a guess of delta in radians based on a perfect sinus.
    The advantage of this is that there will be no overshoot behavior.

    :return:
    """
    import pylab as plt
    time = plt.linspace(0,T_guess,samples)
    output = (0.75*plt.pi/180)*plt.sin(2*plt.pi/T_guess*time)

    return output
