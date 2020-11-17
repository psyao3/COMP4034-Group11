import operator

# Grid map variables
resolution = 1 # Size of each grid cell.
origin = [0, 0, 0] # [x, y, 0] defining the bottom left corner/origin of the grid. 
size = [20, 20] # No. of grid cells in [x, y] dimensions.

# For mymap.yaml
#resolution: 0.050000
#origin: [-10.000000, -10.000000, 0.000000]

def to_grid(p_xy, origin_, size, resolution):
    # Convert a world co-ordinate [px, py] with respect to the origin,
    # grid size and resolution, to [gx, gy], a pair of integer grid
    # co-ordinates
    return [(x - y)/resolution for x, y in zip(p_xy, origin_)]

def to_world(g_xy, origin, size, resolution):
    # Return [px, py], float values representing the center of the grid cell in
    # world co-ordinates.
    return 

# Convert grid coordinate to map index
def to_index(gx, gy, size_x):
  return gy * size_x + gx
    
def test():
    # Using examples from github
    print to_grid( [5,5], [0,0],   [20,20], 1) #should return (5,5)
    print to_grid( [5,5], [0,0],   [20,20], 0.5) #should return (10,10)
    print to_grid( [0,0], [-5,-5], [20,20], 1) #should return (5,5)
    print to_world( [9,9], [-5,-5], [20,20], 1) #should return (4.5, 4.5)

test()