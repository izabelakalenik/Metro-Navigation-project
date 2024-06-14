from SearchAlgorithm import *
from SubwayMap import *
from utils import *

if __name__=="__main__":
    ROOT_FOLDER = '../CityInformation/Barcelona_City/'
    map = read_station_information(os.path.join(ROOT_FOLDER, 'Stations.txt'))
    connections = read_cost_table(os.path.join(ROOT_FOLDER, 'Time.txt'))
    map.add_connection(connections)

    infoVelocity_clean = read_information(os.path.join(ROOT_FOLDER, 'InfoVelocity.txt'))
    map.add_velocity(infoVelocity_clean)


    #This is an example of how to call some of the functions

    # example_path=uniform_cost_search(9, 3, map, 1)
    # print_list_of_path_with_cost([example_path])

    # print(distance_to_stations([169,190], map))

    # print_list_of_path(remove_cycles([Path([5,8,6,2,3]), Path([5,8,6,2,4]), Path([5,8,6,2,5]), Path([5,8,6,2,8]), Path([5,8,6,2,7])]))

    # print_list_of_path_with_cost([breadth_first_search(20,15,map)])

    # print_list_of_path_with_cost([Path([5, 4, 3, 2, 1, 7, 8])])

    # path = Astar(16,21,map, 1)
    # print_list_of_path([path])
    # path_2=calculate_heuristics(expand_paths([Path([16])]), map, 21, 1)
    # print_list_of_path(path_2)
    # print(path_2.h)

    # path = Path([10,23,22,21])
    # path_a = Astar(10,15,map, 1)
    # print_list_of_path_with_cost([path_a])
    # print_list_of_path_with_cost(calculate_cost([path], 15))
    # print_list_of_path([path])
    # print(path.f)

    # path1 = Path([2, 3, 5])
    # path1.update_g(31)
    # path2 = Path([2, 3, 8])
    # path2.update_g(43)
    # path3 = Path([2, 3, 6])
    # path3.update_g(8)
    # path4 = Path([2, 3, 7, 1])
    # path4.update_g(27)
    # new_paths, list_of_path, visited_stations_cost = remove_redundant_paths([path1, path2, path3, path4],[Path([2,3,7,5]), Path([2,3,7,8]), Path([2,3,7,6]), Path([2,3,7,4])], {3:39.86, 7:58.63, 5:42.78, 8:22.83, 6:37.17})
    # print_list_of_path(new_paths)
    # print()
    # print_list_of_path(list_of_path)


    # path = Astar(18,1,map, 3)
    # print_list_of_path([path])

    # path = Astar(4,14,map, 1)
    # print_list_of_path_with_cost([path])