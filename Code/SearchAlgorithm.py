# This file contains all the required routines to make an A* search algorithm.
#
__author__ = '1718541'

# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2023 - 2024
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import copy


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    result_path_list = []

    for station_id in (map.connections[path.last]):
        new_path = copy.deepcopy(path)
        new_path.add_route(station_id)
        result_path_list.append(new_path)

    return result_path_list


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    result_path_list = []

    for path in path_list:
        if len(path.route) == len(set(path.route)):
            result_path_list.append(path)

    return result_path_list


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    return expand_paths + list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    list_of_paths = [Path(origin_id)]

    while len(list_of_paths) != 0:
        path = list_of_paths.pop(0)
        if path.last == destination_id:
            return path
        list_of_paths = insert_depth_first_search(remove_cycles(expand(path, map)), list_of_paths)

    return None


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    return list_of_path + expand_paths


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_paths = [Path(origin_id)]

    while len(list_of_paths) != 0:
        path = list_of_paths.pop(0)
        if path.last == destination_id:
            return path
        list_of_paths = insert_breadth_first_search(remove_cycles(expand(path, map)), list_of_paths)

    return None


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    for path in expand_paths:
        if type_preference == 0:
            path.update_g(1)
        elif type_preference == 1:
            path.update_g(map.connections[path.penultimate][path.last])
        elif type_preference == 2:
            if map.stations[path.last]['line'] == map.stations[path.penultimate]['line']:
                path.update_g(
                    map.connections[path.penultimate][path.last] * map.velocity[map.stations[path.penultimate]['line']])
        elif type_preference == 3:
            if map.stations[path.last]['line'] != map.stations[path.penultimate]['line']:
                path.update_g(1)
            else:
                path.update_g(0)
        else:
            print("Wrong argument: ", type_preference)

    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
    Insert expanded paths into the list of paths according to their cost.

    Args:
        expand_paths (list of Path Class): Expanded paths
        list_of_path (list of Path Class): The paths to be visited

    Returns:
        list of Path Class: Sorted list of paths where expanded paths are inserted according to cost.
    """
    sorted_paths = sorted((list_of_path + expand_paths), key=lambda x: x.g)

    return sorted_paths


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
    Uniform Cost Search algorithm
    Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_paths = [Path(origin_id)]

    while len(list_of_paths) != 0:
        path = list_of_paths.pop(0)
        if path.last == destination_id:
            return path
        list_of_paths = insert_cost(calculate_cost(remove_cycles(expand(path, map)), map, type_preference),
                                    list_of_paths)

    return None


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """

    for path in expand_paths:
        if path.last != destination_id:
            if type_preference == 0:
                path.update_h(1)
            elif type_preference == 1:
                distance = euclidean_dist([map.stations[path.last]['x'], map.stations[path.last]['y']],
                                          [map.stations[destination_id]['x'], map.stations[destination_id]['y']])

                velocity_values = map.stations.values()
                max_velocity = max(station['velocity'] for station in velocity_values)
                path.update_h(distance / max_velocity)

            elif type_preference == 2:
                path.update_h(euclidean_dist([map.stations[path.last]['x'], map.stations[path.last]['y']],
                                             [map.stations[destination_id]['x'], map.stations[destination_id]['y']]))
            elif type_preference == 3:
                if map.stations[path.last]['line'] != map.stations[destination_id]['line']:
                    path.update_h(1)
            else:
                print("Wrong argument: ", type_preference)
        else:
            path.update_h(0)

    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for path in expand_paths:
        path.update_f()

    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
    Removes redundant paths from the expanded paths.
    A path is redundant if it reaches a station with a higher or equal g-cost.

    Args:
        expand_paths (list of Path Class): Expanded paths
        list_of_path (list of Path Class): All the paths to be expanded
        visited_stations_cost (dict): All visited stations cost

    Returns:
        new_paths (list of Path Class): Expanded paths without redundant paths
        updated_list_of_path (list of Path Class): list_of_path without redundant paths
        updated_visited_stations_cost (dict): Updated visited stations cost
    """
    new_paths = []
    for path in expand_paths:
        if path.last in visited_stations_cost:
            if visited_stations_cost[path.last] > path.g:
                visited_stations_cost[path.last] = path.g

                updated_list_of_path = []
                for p in list_of_path:
                    if path.last not in p.route:
                        updated_list_of_path.append(p)
                list_of_path = updated_list_of_path
            else:
                continue
        else:
            visited_stations_cost[path.last] = path.g
        new_paths.append(path)

    return new_paths, list_of_path, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """

    sorted_paths = sorted((list_of_path + expand_paths), key=lambda x: x.f)

    return sorted_paths


def distance_to_stations(coord, map):
    """
        From coordinates, it computes the distance to all stations in map.
        Format of the parameter is:
        Args:
            coord (list): Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            (dict): Dictionary containing as keys, all the Indexes of all the stations in the map, and as values, the
            distance between each station and the coord point
    """
    distances = {}
    for station in map.stations:
        distances[station] = euclidean_dist(coord, [map.stations[station]['x'], map.stations[station]['y']])

    sorted_distances = sorted(distances.items(), key=lambda x: (x[1], x[0]))
    sorted_distances_dict = {k: v for k, v in sorted_distances}  # Convert list of tuples to dictionary

    return sorted_distances_dict


def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_paths = [Path(origin_id)]
    visited_stations_cost = {origin_id: 0}

    while len(list_of_paths) != 0:
        path = list_of_paths.pop(0)
        if path.last == destination_id:
            return path
        expanded_paths = calculate_cost(expand(path, map), map, type_preference)
        expanded_paths = calculate_heuristics(expanded_paths, map, destination_id, type_preference)
        expanded_paths = update_f(expanded_paths)
        expanded_paths, list_of_paths, visited_stations_cost = remove_redundant_paths(expanded_paths, list_of_paths,
                                                                                      visited_stations_cost)
        list_of_paths = insert_cost_f(expanded_paths, list_of_paths)

    return None


def Astar_improved(origin_coord, destination_coord, map):
    """
    A* Search algorithm

    Args:
        origin_coord (list): Two REAL values, which refer to the coordinates of the starting position
        destination_coord (list): Two REAL values, which refer to the coordinates of the final position
        map (object of Map class): All the map information

    Returns:
        list_of_path[0] (Path Class): The route that goes from origin_coord to destination_coord
    """
    list_of_paths = []
    origin_list = list(distance_to_stations(origin_coord, map).keys())
    destination_list = list(distance_to_stations(destination_coord, map).keys())

    for origin_station in origin_list:
        origin_distance = euclidean_dist(origin_coord, [map.stations[origin_station]['x'], map.stations[origin_station]['y']])
        for destination_station in destination_list:
            destination_distance = euclidean_dist(destination_coord, [map.stations[destination_station]['x'], map.stations[destination_station]['y']])
            walking_time = (origin_distance + destination_distance) / 5

            path = Astar(origin_station, destination_station, map, 1)
            path.update_g(walking_time)
            path.update_f()
            path.route.insert(0, 0)
            path.route.append(-1)
            list_of_paths.append(path)

    # Calculate walking time from origin to destination separately
    walking_distance = euclidean_dist(origin_coord, destination_coord)
    walking_time = walking_distance / 5
    walk = Path([0, -1])
    walk.update_g(walking_time)
    walk.update_f()
    list_of_paths.append(walk)

    list_of_paths = sorted(list_of_paths, key=lambda x: x.g)

    return list_of_paths[0]

def test_set_up():
    ROOT_FOLDER = '../CityInformation/Lyon_smallCity/'

    subway_map = read_station_information(os.path.join(ROOT_FOLDER, 'Stations.txt'))
    connections = read_cost_table(os.path.join(ROOT_FOLDER, 'Time.txt'))
    subway_map.add_connection(connections)

    info_velocity_clean = read_information(os.path.join(ROOT_FOLDER, 'InfoVelocity.txt'))
    subway_map.add_velocity(info_velocity_clean)

    return subway_map


if __name__ == '__main__':
    map = test_set_up()

    paths = [Path([13, 12, 8]), Path([13, 12, 11]), Path([13, 12, 13])]

    for station in map.stations:
        print(station, map.stations[station])
    print()
    for connection in map.connections:
        print(connection, map.connections[connection])

    print()
    print_list_of_path_with_cost(paths)
    print()
    print(distance_to_stations([3, 5], map))
    # route = uniform_cost_search(9, 3, map, 0)
