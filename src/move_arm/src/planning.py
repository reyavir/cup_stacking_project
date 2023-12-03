# Takes coordinates of the leftmost point and the number of cups to put on the pyramid and outputs a list of x,y,z coordinates for each cup
def plan_pyramid(num_cups, start_x, start_y, start_z, cup_diameter, cup_height):
    layers = pyramid_shape(num_cups)
    print("num_cups", num_cups)
    print("layers", layers)
    cup_coordinates = []
    x, y, z = start_x, start_y, start_z
    print("start z", start_z)
    print("cup height", cup_height)
    first_y = start_y
    for i in reversed(range(1, layers+1)):
        for j in range(1, i+1):
            cup_coordinates.append((x,y,z))
            y = y + cup_diameter
        first_y = first_y + cup_diameter/2
        y = first_y
        z = z + cup_height
    print("cup_coordinates", cup_coordinates)
    return cup_coordinates

def pyramid_shape(cups):
    layers = 0
    if cups==1:
        return 1
    for i in range(1, cups):
        if i > cups:
            break
        cups = cups - i
        layers = layers + 1
    return layers

# print(pyramid_shape(1))
# print(pyramid_shape(2))
# print(pyramid_shape(3))
# print(pyramid_shape(4))
# print(pyramid_shape(5))
# print(pyramid_shape(6))
# print(pyramid_shape(7))



