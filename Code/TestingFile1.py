import os

from SearchAlgorithm import expand, uniform_cost_search, breadth_first_search
from SubwayMap import Path
from utils import read_station_information, read_cost_table, read_information, print_list_of_path, \
    print_list_of_path_with_cost


def print_list_of_path_with_heu(path_list):
    for p in path_list:
        print("Route: {}, \t Cost: {}".format(p.route, round(p.h, 2)))


if __name__ == "__main__":
    ROOT_FOLDER = '../CityInformation/Lyon_SmallCity/'
    map = read_station_information(os.path.join(ROOT_FOLDER, 'Stations.txt'))
    connections = read_cost_table(os.path.join(ROOT_FOLDER, 'Time.txt'))
    map.add_connection(connections)

    infoVelocity_clean = read_information(os.path.join(ROOT_FOLDER, 'InfoVelocity.txt'))
    map.add_velocity(infoVelocity_clean)


    # example
    # example_path = expand(Path([5]), map)
    # print_list_of_path(example_path)
    example_path_2 = uniform_cost_search(9, 3, map, 1)
    # print_list_of_path_with_cost([example_path_2])

    example_path_3 = breadth_first_search(12, 14, map)
    # print_list_of_path([example_path_3])
    print_list_of_path_with_cost([Path([5, 4, 3, 2, 1, 7, 8])])