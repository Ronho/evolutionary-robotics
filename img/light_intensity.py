import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

fig, ax = plt.subplots()

# Define the center of the circle and its radius.
center = (40, 40)
radius = 30
inner_color = (0, 1, 0)  # Green

# Create a grid of points within the circle
x = np.linspace(-100, 100, 200)
y = np.linspace(-100, 100, 200)
X, Y = np.meshgrid(x, y)
Z = np.sqrt((X - center[0])**2 + (Y - center[1])**2)

# Inner color with full opacity, outer color with full transparency
colors = [(*inner_color, 1), (*inner_color, 0)]
cmap_name = 'circle_gradient'
cmap = LinearSegmentedColormap.from_list(cmap_name, colors)

im = ax.imshow(Z, cmap=cmap, interpolation='bilinear', origin='lower', extent=[-100, 100, -100, 100])

# Add a colorbar to show the gradient.
cbar = plt.colorbar(im)
ticks = np.arange(0, 201, 25)
labels = 1 - np.round((ticks / 200), 2)
cbar.ax.set_yticks(ticks, labels)
cbar.set_label('Intensity of Light')

ax.plot(center[0], center[1], marker='o', markersize=30, color="red")

plt.savefig('light_intensity.png')
plt.show()
