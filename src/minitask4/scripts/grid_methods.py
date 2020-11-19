import operator

# Grid map variables
resolution = 1.0  # Size of each grid cell.
# origin = [0, 0, 0]  # [x, y, 0] defining the bottom left corner/origin of the grid.
size = [20, 20]  # No. of grid cells in [x, y] dimensions.

origin = [-10.0, -10.0]
# For mymap.yaml
# resolution: 0.050000
# origin: [-10.000000, -10.000000, 0.000000]

# TODO: add conditionals for size (so its not exceeded), deal with
# invalid values.


def to_grid(p_xy, origin_, size, resolution):
    # Convert a world co-ordinate [px, py] with respect to the origin,
    # grid size and resolution, to [gx, gy], a pair of integer grid
    # co-ordinates

    # Need to round or truncate real world co-ordinates - convert to Int

    g_xy = [int((a - b)/resolution) for a, b in zip(p_xy, origin_)]

    # If statements for exceeding 20x20 bounds, raise/print ValueError
    if g_xy[0] >= size[0] or g_xy[1] >= size[0] or g_xy[0] < 0 or g_xy[1] < 0:
        # raise ValueError("Exceeding grid size.")
        print('ValueError: Exceeding grid size.')

    return g_xy


def to_world(g_xy, origin, size, resolution):
    # Return [px, py], float values representing the center of the grid cell in
    # world co-ordinates.
    
    # Resolution needs to be a float here
    p_xy = [(((a + b) * resolution) + resolution/2) for a, b in zip(g_xy, origin)]
    return p_xy


# Convert grid coordinate to map index; size_x is expected to be size[0]
def to_index(gx, gy, size_x):
    return gy * size_x + gx



