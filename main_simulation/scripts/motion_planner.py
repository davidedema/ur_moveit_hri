import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy import interpolate
from scipy.interpolate import Rbf

# Input data
x = np.array([1, 2, 2.3, 3, 4, 5, 5.5, 8, 9, 9.5])
y = np.arange(0, 10)
z = np.sin(x) * np.cos(y**2) + x  # Use element-wise power '**'

# Create figure and 3D axes
fig = plt.figure(figsize=(10, 6))
ax = Axes3D(fig, auto_add_to_figure=False)  # Prevent auto addition
fig.add_axes(ax)  # Manually add the axes to the figure

# Stem plot
ax.stem(x, y, z)

# Chord-length parameterization for spline interpolation
xyz = np.vstack([x, y, z]).T
u = np.cumsum(np.r_[[0], np.linalg.norm(np.diff(xyz, axis=0), axis=1)])

# Interpolation splines
sx = interpolate.InterpolatedUnivariateSpline(u, x)
sy = interpolate.InterpolatedUnivariateSpline(u, y)
sz = interpolate.InterpolatedUnivariateSpline(u, z)

# Generate interpolated points using spline interpolation
uu = np.linspace(u[0], u[-1], 100)
xx_spline = sx(uu)
yy_spline = sy(uu)
zz_spline = sz(uu)

# Plot spline interpolated curve
ax.plot(xx_spline, yy_spline, zz_spline, "b", label="Spline Interpolation")

# Rbf interpolation
rbfi_x = Rbf(u, x, function='cubic')
rbfi_y = Rbf(u, y, function='cubic')
rbfi_z = Rbf(u, z, function='cubic')

# Generate interpolated points using Rbf
xx_rbf = rbfi_x(uu)
yy_rbf = rbfi_y(uu)
zz_rbf = rbfi_z(uu)

# Plot Rbf interpolated curve
ax.plot(xx_rbf, yy_rbf, zz_rbf, "r", label="RBF Interpolation")

# Add labels and legend
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.legend()

# Show the plot
plt.show()
