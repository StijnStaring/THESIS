import pylab as plt
theta_iter = str(1)
plt.figure("Path vs Time: iter " + theta_iter,figsize=(10, 4))
plt.subplot(1, 2, 1)
ax1a = plt.gca()
plt.xlabel("Time [s]", fontsize=14)
plt.ylabel("Horizontal distance [m]", fontsize=14)
plt.grid(True)
plt.title('x(t) global',fontsize=14)

ax1a.plot([0,1],[0,1],'b-',)

plt.subplot(1, 2, 2)
ax1b = plt.gca()
plt.xlabel("Time [s]", fontsize=14)
plt.ylabel("Vertical distance [m]", fontsize=14)
plt.title('y(t) global',fontsize=14)
plt.grid(True)
ax1b.plot([0,1],[0,1],'r-',)
# path
plt.figure("Path: iter " + theta_iter)
ax2 = plt.gca()
plt.xlabel("x [m]", fontsize=14)
plt.ylabel("y [m]", fontsize=14)
plt.title('Path global [m] ', fontsize=14)
plt.grid(True)


#call figure again
plt.figure("Path vs Time: iter " + theta_iter,figsize=(10, 4))
fname = "results/test.png"
plt.savefig(fname, dpi=None, facecolor='w', edgecolor='w',orientation='portrait', papertype=None, format=None,transparent=False, bbox_inches=None, pad_inches=0.1,metadata=None)
plt.show()