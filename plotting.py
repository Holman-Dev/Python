import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from geopy import distance
from geopy import Point
plt.ion()

# Function to calculate the location of points on the circle
def points_on_circle(center, radius, num_points):
    circle_points = []
    for i in np.linspace(0, 2*np.pi, num_points):
        dist = distance.distance(miles=radius)
        point_on_circle = dist.destination(point=center, bearing=np.degrees(i))
        circle_points.append((point_on_circle.latitude, point_on_circle.longitude))
    return circle_points

def plot_course(center_lat, center_lon, radius, aircraft_lat, aircraft_lon):
    center = Point(center_lat, center_lon)
    circle_points = points_on_circle(center, radius, num_points=100)

    # Unpack the circle points into X and Y lists
    lat_list, lon_list = zip(*circle_points)

    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot the circle
    ax.plot(lon_list, lat_list, 'b-')  # blue circle
    ax.plot(center_lon, center_lat, 'ro')  # red center

    # Initial aircraft position
    aircraft_dot, = ax.plot(aircraft_lon, aircraft_lat, 'go')  # green aircraft

    def animate(i):
        # Update aircraft position
        aircraft_dot.set_data(aircraft_lon, aircraft_lat)
        plt.pause(0.001)
        return aircraft_dot,

    ani = animation.FuncAnimation(fig, animate, frames=100, interval=200, blit=True)

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Aircraft and Waypoints')
    plt.grid(True)
    plt.show()