import matplotlib.pyplot as plt
import math

# Create sinewaves with sine and cosine
xs = [i / 5.0 for i in range(0, 50)]
y1s = [math.sin(x) for x in xs]
y2s = [math.cos(x) for x in xs]

# Explicitly create our figure and subplots
fig = plt.figure()
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)

# Draw our sinewaves on the different subplots
ax1.plot(xs, y1s)
ax2.plot(xs, y2s)

# Adding labels to subplots is a little different
ax1.set_title('sin(x)')
ax1.set_xlabel('Radians')
ax1.set_ylabel('Value')
ax2.set_title('cos(x)')
ax2.set_xlabel('Radians')
ax2.set_ylabel('Value')

# We can use the subplots_adjust function to change the space between subplots
plt.subplots_adjust(hspace=0.6)

# Draw all the plots!
plt.show()
