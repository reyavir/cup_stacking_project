# Takes coordinates of the leftmost point and the number of cups to put on the pyramid and outputs a list of x,y,z coordinates for each cup
def plan_pyramid(num_cups, start_x, start_y, start_z, cup_diameter, cup_height):
    layers = pyramid_shape(num_cups)
    cup_coordinates = []
    x, y, z = start_x, start_y, start_z
    first_y = start_y
    for i in reversed(range(1, layers+1)):
        for j in range(1, i+1):
            cup_coordinates.append((x,y,z))
            y = y + cup_diameter
        first_y = first_y + cup_diameter/2
        y = first_y
        z = z + z + cup_height
    return cup_coordinates

def pyramid_shape(cups):
    layers = 0
    for i in range(1, cups):
        if i > cups:
            break
        cups = cups - i
        layers = layers + 1
    return layers





