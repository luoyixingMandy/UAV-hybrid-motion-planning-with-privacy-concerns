#!/usr/bin/env python
# main function
import numpy as np
from path import Path
from point import Point
import geneticAlgorithm as gA
from quickSort import quick_sort
from gridVisualization import grid_visualization
from mapTools import privacy_init, map_generate

path = []
grid_x = 5
grid_y = 5
grid_z = 5
thickness = 5
occ_grid = None
population = 500
max_generation = 30
selection_size = 50
objectives = []
no_of_objectives = 1
obstacles_per_axis = 1
starting_point = Point(0, 0, 0, 0)
end_point = Point(4, 4, 4, 0)
grid_map = [grid_x, grid_y, grid_z]
sigma = 0.00001
mutation_rate = 5
ts = 2
sr = 40
safety_threshold = 0.5
privacy_threshold = 0.1
privacy_radius = 1
# the tolerance of the times of camera off
Kca = 3


if __name__ == "__main__":
    # Create a 3D Occupancy Grid
    print('\033[94m Generating random occupancy grid and objectives... \033[0m')
    occ_grid, obstacle_num = map_generate(grid_x, grid_y, grid_z,
                                          starting_point, end_point, safety_threshold, privacy_threshold)
    print(occ_grid)
    pri_grid, privacy_sum = privacy_init(grid_x, grid_y, grid_z, occ_grid, privacy_radius)
    print(pri_grid)
    objectives = [end_point]
    alg = gA.GeneticAlgorithm(population, 0.00001, 5, 2, 40, grid_map)

    print('\033[94m Generating random initial solutions... \033[0m')
    paths = alg.init_population(starting_point, objectives, Kca)

    for p in range(len(paths)):
        paths[p].fitness = alg.get_fitness(paths[p], occ_grid, pri_grid,
                                           starting_point, end_point, privacy_sum, obstacle_num)

    max_p = max(paths, key=lambda x: x.fitness)

    max_f = -5
    count = 0
    fitnv = []
    print(len(paths))

    for i in range(max_generation):
        quick_sort(paths)
        if max_f < paths[0].fitness:
            max_f = paths[0].fitness
            print('\033[94m Current maximum fitness:\033[0m\033[92m ' + str(
                max_f) + '\033[0m\033[94m, Generation:\033[0m\033[92m ' + str(i) + ' \033[0m')
            for j in range(len(paths[0].points)):
                print(paths[0].points[j])
            print("the generation", i, len(paths[0].points))
            alg.get_fitness(paths[0], occ_grid, pri_grid, starting_point, end_point, privacy_sum, obstacle_num)
        '''
        p1 = alg.tournament_select(paths)
        p2 = alg.tournament_select(paths)

        new_path = []
        # Always crossover (cr = 1)
        new_path1 = alg.cross_over(paths[p1].points, paths[p2].points, objectives, Kca)
        new_path2 = alg.cross_over(paths[p2].points, paths[p1].points, objectives, Kca)

        new_path1_ = alg.mutate(new_path1, objectives, Kca)
        new_path1_.fitness = alg.get_fitness(new_path1_, occ_grid, pri_grid,
                                             starting_point, end_point, privacy_sum, obstacle_num)
        paths[-2] = new_path1_

        new_path2_ = alg.mutate(new_path2, objectives, Kca)
        new_path2_.fitness = alg.get_fitness(new_path2_, occ_grid, pri_grid,
                                             starting_point, end_point, privacy_sum, obstacle_num)
        paths[-1] = new_path2_
        '''

        # 选择
        selected_path_list = alg.select(paths, selection_size, occ_grid, pri_grid, starting_point,
                                        end_point, privacy_sum, obstacle_num)
        # 形成couple, 交叉， 变异
        new_path_list = []
        for j in range(selection_size):
            for k in range(j+1, selection_size):
                new_path1 = alg.cross_over(selected_path_list[i].points, selected_path_list[k].points, objectives, Kca)
                new_path2 = alg.cross_over(selected_path_list[k].points, selected_path_list[j].points, objectives, Kca)

                new_path1_mutated = alg.mutate(new_path1, objectives, Kca)
                new_path1_mutated.fitness = alg.get_fitness(new_path1_mutated, occ_grid, pri_grid,
                                                            starting_point, end_point, privacy_sum, obstacle_num)
                new_path_list.append(new_path1_mutated)

                new_path2_mutated = alg.mutate(new_path2, objectives, Kca)
                new_path2_mutated.fitness = alg.get_fitness(new_path2_mutated, occ_grid, pri_grid,
                                                            starting_point, end_point, privacy_sum, obstacle_num)
                new_path_list.append(new_path2_mutated)

        # 重插入
        paths = alg.reinsert(paths, new_path_list, population)

    # optimal path
    quick_sort(paths)
    for j in range(len(paths[0].points)):
        print(paths[0].points[j])
    print("optimal", len(paths[0].points))

    # initial path
    for j in range(len(max_p.points)):
        print(max_p.points[j])
    print("initial", len(max_p.points))

    grid_visualization(occ_grid, starting_point, objectives, paths[0].points, max_p.points)
