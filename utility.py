import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def interpolate_coordinates(coordinates):
    # Ensure coordinates is a list or array
    if not isinstance(coordinates, (list, np.ndarray)):
        raise ValueError("Coordinates must be a list or array")

    # Ensure the number of coordinates is divisible by 3
    if len(coordinates) % 3 != 0:
        raise ValueError("The number of coordinates must be divisible by 3")

    # Reshape the coordinates into a 2D array with 3 columns (latitude, longitude, altitude)
    coordinates = np.array(coordinates).reshape(-1, 3)

    # Unpack the coordinates
    latitudes, longitudes, altitudes = coordinates.T

    # Create an array of indices
    indices = np.arange(len(latitudes))

    # Create cubic spline interpolators for each coordinate
    lat_interp = interpolate.CubicSpline(indices[:-2], latitudes[:-2])
    lon_interp = interpolate.CubicSpline(indices[:-2], longitudes[:-2])
    alt_interp = interpolate.CubicSpline(indices[:-2], altitudes[:-2])

    # Create linear interpolators for the last two points
    lat_interp_last = interpolate.interp1d(indices[-2:], latitudes[-2:])
    lon_interp_last = interpolate.interp1d(indices[-2:], longitudes[-2:])
    alt_interp_last = interpolate.interp1d(indices[-2:], altitudes[-2:])

    # Create an array of new indices with additional points
    new_indices = np.linspace(0, len(latitudes) - 3, (len(latitudes) - 2) * 7)
    new_indices_last = np.linspace(len(latitudes) - 2, len(latitudes) - 1, 7)

    # Interpolate the coordinates
    new_latitudes = np.concatenate([lat_interp(new_indices), lat_interp_last(new_indices_last)])
    new_longitudes = np.concatenate([lon_interp(new_indices), lon_interp_last(new_indices_last)])
    new_altitudes = np.concatenate([alt_interp(new_indices), alt_interp_last(new_indices_last)])

    # Combine the coordinates into a flat list
    new_coordinates = np.concatenate([new_latitudes, new_longitudes, new_altitudes])

    # Plot the original and interpolated coordinates
    '''fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(longitudes, latitudes, altitudes, 'bo-', label='Original')
    ax.plot(new_longitudes, new_latitudes , new_altitudes, 'ro-', label='Interpolated')
    ax.legend()
    plt.show()'''

    return new_coordinates.tolist()