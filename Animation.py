# import numpy as np
# from matplotlib import pyplot as plt
# from matplotlib import animation

# # First set up the figure, the axis, and the plot element we want to animate
# fig = plt.figure()
# ax = plt.axes(xlim=(0, 5), ylim=(0, 5))
# line, = ax.plot([], [], lw=2)


# def init():
#     line.set_data([], [])
#     return line,

# # animation function.  This is called sequentially
# def animate(i):
#     x = [0,1,2,3,4,5]
#     y_data = [[0,0,0,0,0,0][0,1,0,0,0,0][0,1,2,0,0,0][0,1,2,3,0,0][0,1,2,3,4,0][0,1,2,3,4,5]]
#     y = line.set_ydata(y_data[i, :])
#     line.set_data(x, y)
#     return line,

# if __name__ == '__main__':

#     # call the animator.  blit=True means only re-draw the parts that have changed.
#     anim = animation.FuncAnimation(fig, animate, init_func=init,frames=200, interval=1000, blit=True)

#     # save the animation as an mp4.  This requires ffmpeg or mencoder to be
#     # installed.  The extra_args ensure that the x264 codec is used, so that
#     # the video can be embedded in html5.  You may need to adjust this for
#     # your system: for more information, see
#     # http://matplotlib.sourceforge.net/api/animation_api.html

#     plt.show()

import numpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def update_line(num, data, line):
    line.set_data(data[..., :num])
    return line,

fig1 = plt.figure()

data = numpy.array([[[0.1,0.2],[0.2,0.3],[0.4,0.5]],[[0.1,0.2],[0.3,0.4],[0.4,0.5]]])#np.random.rand(2, 5)
print data
l, = plt.plot([], [], 'r-')
plt.xlim(0, 1)
plt.ylim(0, 1)
plt.xlabel('x')
plt.title('test')
line_ani = animation.FuncAnimation(fig1, update_line, 5, fargs=(data, l),interval=500, blit=True)


plt.show()