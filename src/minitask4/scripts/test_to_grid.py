#!/usr/bin/env python

import grid_methods as grid


interactive = False


def interactive_test_to_grid():

    print("\nTesting to_grid() function \n\n")
    p_xy = input("Insert parameter: p_xy...\n")
    origin = input("Insert parameter: origin...\n")
    size = input("Insert parameter: size...\n")
    resolution = input("Insert parameter: resolution...\n")

    p_xy = [float(value) for value in p_xy]
    origin = [float(value) for value in origin]
    size = [float(value) for value in size]
    resolution = float(resolution)

    print("\nResult:")
    print(grid.to_grid(p_xy=p_xy, origin_=origin, size=size, resolution=resolution))


def interactive_test_to_world():

    print("\nTesting to_world() function \n\n")
    p_xy = input("Insert parameter: g_xy...\n")
    origin = input("Insert parameter: origin...\n")
    size = input("Insert parameter: size...\n")
    resolution = input("Insert parameter: resolution...\n")

    p_xy = [float(value) for value in p_xy]
    origin = [float(value) for value in origin]
    size = [float(value) for value in size]
    resolution = float(resolution)

    print("\nResult:")
    print(grid.to_world(g_xy=p_xy, origin=origin, size=size, resolution=resolution))


def static_test_to_grid():

    print("\nTesting to_grid() function \n\n")

    # Using examples from github
    test_values = [
        # p_xy | origin | size | resolution
        [[5, 5], [0, 0], [20, 20], 1, [5, 5]],
        [[5, 5], [0, 0], [20, 20], 0.5, [10, 10]],
        [[0,0], [-5, -5], [20, 20], 1, [5, 5]],
        [[9, 9], [-5, -5], [20, 20], 1, [4.5, 4.5]],
        [[10000, 0], [0, 0], [20, 20], 1, "Error"],
        [[5, 10000], [0, 0], [20, 20], 1, "Error"],
        [[-20, 5], [0, 0], [20, 20], 1, "Error"],
        [[5, -1], [0, 0], [20, 20], 1, "Error"]
    ]

    for value in test_values:

        print('\nTesting: p_xy: {}, origin: {}, size: {}, resolution: {}'.format(value[0], value[1], value[2], value[3]))
        print('Should return {}'.format(value[4]))
        print('Result: {}'.format(grid.to_grid(value[0], value[1], value[2], value[3])))


def static_test_to_world():

    print("\nTesting to_world() function \n\n")

    # Using examples from github
    test_values = [
        # g_xy | origin | size | resolution
        [[9, 9], [-5, -5], [20, 20], 1, [4.5, 4.5]],
        [[30, 30], [-5, -5], [20, 20], 1, "Error"],
        [[-10, 10], [-5, -5], [20, 20], 1, "Error"]
    ]

    for value in test_values:

        print('\nTesting: g_xy: {}, origin: {}, size: {}, resolution: {}'.format(value[0], value[1], value[2], value[3]))
        print('Should return {}'.format(value[4]))
        print('Result: {}'.format(grid.to_world(value[0], value[1], value[2], value[3])))


if __name__ == "__main__":

    mode = raw_input("Interactive or static testing? \n")
    function = raw_input("Testing to_grid() or to_world()? \n")
    if mode.lower() == 'interactive':
        if function.lower() == 'to_grid()' or function.lower() == 'to_grid':
            interactive_test_to_grid()
        else:
            interactive_test_to_world()
    else:
        if function.lower() == 'to_grid()' or function.lower() == 'to_grid':
            static_test_to_grid()
        else:
            static_test_to_world()

