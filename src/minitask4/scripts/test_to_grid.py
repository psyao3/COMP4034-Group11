#!/usr/bin/env python

import grid_methods as grid


interactive = False


def interactive_test_to_grid():

    print("\nTesting to_grid() function \n\n")
    p_xy = input("Insert parameter: p_xy...\n")
    origin = input("Insert parameter: origin...\n")
    size = input("Insert parameter: size...\n")
    resolution = input("Insert parameter: resolution...\n")

    p_xy = [float(value) for value in p_xy.split(',')]
    origin = [float(value) for value in origin.split(',')]
    size = [float(value) for value in size.split(',')]
    resolution = float(resolution)

    print("\nResult:")
    print(grid.to_grid(p_xy=p_xy, origin_=origin, size=size, resolution=resolution))


def static_test_to_grid():

    # Using examples from github
    test_values = [
        [[5,5], [0,0],   [20,20], 1, (5, 5)],
        [[5, 5], [0, 0], [20, 20], 1, (10,10)],
        [[5, 5], [0, 0], [20, 20], 1, (5, 5)],
        [[5, 5], [0, 0], [20, 20], 1, (4.5, 4.5)],
    ]

    for value in test_values:

        print(f'\nShould return {value[4]}')
        print(grid.to_grid(value[0], value[1], value[2], value[3]))


if __name__ == "__main__":

    if interactive:
        interactive_test_to_grid()
    else:
        static_test_to_grid()
