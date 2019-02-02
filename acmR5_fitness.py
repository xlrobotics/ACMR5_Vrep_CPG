import numpy as np

def fitness(start_x, start_y, end_x, end_y, start_z = 0.0,
                  end_z = 0.0,
                 avg_z = 0.0,
                 up_time = 1.0,
                 fitness_option = 1):

    fitness = 0.0

    x_distance = end_x - start_x
    y_distance = end_y - start_y

    x_vel = x_distance / up_time  # (metres/second)

    fitness = 80.0*x_vel - 100*abs(y_distance)

    return fitness