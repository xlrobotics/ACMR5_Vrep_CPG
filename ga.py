# Script with low level feedback for phase resetting of pacemaker oscillator without ROS

# import os
import random
# import vrep

from deap import base
from deap import creator
from deap import tools

# from matsuoka_walk import Logger, log
from acmR5_control import *

# Set the home directory
# home_dir = os.path.expanduser('~')

# Set the logging variables
# This also creates a new log file
# Logger(log_dir=os.path.join(home_dir, '.bio_walk/logs/'), log_flag=True)
#
# log('[GA] Running ga_5')
#
# # Create the position bounds of the individual
# log('[GA] Creating position bounds')

FLT_MIN_KP, FLT_MAX_KP = -1.0, 1.0

FLT_MIN_TR, FLT_MAX_TR = 0.2, 1.0
FLT_MIN_TA, FLT_MAX_TA = 0.2, 1.0
FLT_MIN_BETA, FLT_MAX_BETA = 0.2, 20.0
FLT_MIN_A, FLT_MAX_A = 0.2, 2.0

FLT_MIN_W1, FLT_MAX_W1 = 0.01, 10.0
FLT_MIN_W2, FLT_MAX_W2 = 0.01, 10.0
FLT_MIN_W3, FLT_MAX_W3 = 0.01, 10.0
FLT_MIN_W4, FLT_MAX_W4 = 0.01, 10.0
FLT_MIN_W5, FLT_MAX_W5 = 0.01, 10.0
FLT_MIN_W6, FLT_MAX_W6 = 0.01, 10.0
FLT_MIN_W7, FLT_MAX_W7 = 0.01, 10.0
FLT_MIN_W8, FLT_MAX_W8 = 0.01, 10.0

# Define a custom class named `FitnessMax`
# Single objective function is specified by the tuple `weights=(1.0,)`
creator.create("FitnessMax", base.Fitness, weights=(1.0,))

# Create a class named `Individual` which inherits from the class `list` and has `FitnessMax` as an attribute
creator.create("Individual", list, fitness=creator.FitnessMax)

# Now we will use our custom classes to create types representing our individuals as well as our whole population.
# All the objects we will use on our way, an individual, the population, as well as all functions, operators, and
# arguments will be stored in a DEAP container called `Toolbox`. It contains two methods for adding and removing
# content, register() and unregister().

toolbox = base.Toolbox()

# Attribute generator - specify how each single gene is to be created
# name, uniform sampling from [0, 1], lowerbound, upperbound
toolbox.register("kp_flt", random.uniform, FLT_MIN_KP, FLT_MAX_KP)
toolbox.register("w1_flt", random.uniform, FLT_MIN_W1, FLT_MAX_W1)
toolbox.register("w2_flt", random.uniform, FLT_MIN_W2, FLT_MAX_W2)
toolbox.register("w3_flt", random.uniform, FLT_MIN_W3, FLT_MAX_W3)
toolbox.register("w4_flt", random.uniform, FLT_MIN_W4, FLT_MAX_W4)
toolbox.register("w5_flt", random.uniform, FLT_MIN_W5, FLT_MAX_W5)
toolbox.register("w6_flt", random.uniform, FLT_MIN_W6, FLT_MAX_W6)
toolbox.register("w7_flt", random.uniform, FLT_MIN_W7, FLT_MAX_W7)
toolbox.register("w8_flt", random.uniform, FLT_MIN_W8, FLT_MAX_W8)

toolbox.register("a_flt", random.uniform, FLT_MIN_A, FLT_MAX_A)
toolbox.register("tr_flt", random.uniform, FLT_MIN_TR, FLT_MAX_TR)
toolbox.register("ta_flt", random.uniform, FLT_MIN_TA, FLT_MAX_TA)
toolbox.register("beta_flt", random.uniform, FLT_MIN_BETA, FLT_MAX_BETA)


# Specify the structure of an individual chromosome
N_CYCLES=1 # Number of times to repeat this pattern

# Specify the sequence of genes in an individual chromosome
toolbox.register("individual", tools.initCycle, creator.Individual,
                 (toolbox.kp_flt,
                  toolbox.w1_flt, toolbox.w2_flt, toolbox.w3_flt, toolbox.w4_flt,
                  toolbox.w5_flt, toolbox.w6_flt, toolbox.w7_flt, toolbox.w8_flt,
                  toolbox.a_flt ,toolbox.tr_flt, toolbox.ta_flt, toolbox.beta_flt),
                 n=N_CYCLES)

# Define the population to be a list of individuals
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Register the goal / fitness function
toolbox.register("evaluate", experiment)

# Register the crossover operator - 2 point crossover is used here
toolbox.register("mate", tools.cxTwoPoint)

# Register a mutation operator
# Mutation is done by adding a float to each gene. This float to be added is randomly selected from a Gaussian
# distribution with mu=0.0 and sigma=0.01
# Probability of mutation is 0.05
toolbox.register("mutate", tools.mutGaussian, mu=0.0, sigma=0.1, indpb=0.05)

# Operator for selecting individuals for breeding the next
# generation: each individual of the current generation
# is replaced by the 'fittest' (best) of 3 individuals
# drawn randomly from the current generation.
toolbox.register("select", tools.selTournament, tournsize=3)

# Size of the population
POP_SIZE = 36

# Maximum generations
MAX_GEN = 32

def main():
    random.seed(64)

    # Create an initial population of `POP_SIZE` individuals (where each individual is a list of floats)
    pop = toolbox.population(n=POP_SIZE)

    # CXPB  is the probability with which two individuals are crossed
    # MUTPB is the probability for mutating an individual
    CXPB, MUTPB = 0.8, 0.1

    # log('[GA] Starting genetic algorithm')

    # Evaluate the entire population and store the fitness of each individual
    # log('[GA] Finding the fitness of individuals in the initial generation')
    print("=======generation 0 =======")
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        print (ind, fit)
        ind.fitness.values = (fit,)

    # Extracting all the fitnesses
    fits = [ind.fitness.values[0] for ind in pop]

    # Variable keeping track of the number of generations
    g = 0

    best_ind_ever = None
    best_fitness_ever = 0.0

    # Begin the evolution
    while max(fits) < 100 and g < MAX_GEN:

        # A new generation
        g = g + 1
        print("=======generation ", g, " =======")
        # log('[GA] Running generation {0}'.format(g))

        # Select the next generation individuals
        # log('[GA] Selecting the next generation')
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            # cross two individuals with probability CXPB
            if random.random() < CXPB:
                toolbox.mate(child1, child2)

                # fitness values of the children
                # must be recalculated later
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            # mutate an individual with probability MUTPB
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Since the content of some of our offspring changed during the last step, we now need to
        # re-evaluate their fitnesses. To save time and resources, we just map those offspring which
        # fitnesses were marked invalid.
        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = (fit,)

        # log('[GA] Evaluated {0} individuals (invalid fitness)'.format(len(invalid_ind)))
        print ('[GA] Evaluated {0} individuals (invalid fitness)'.format(len(invalid_ind)))
        # The population is entirely replaced by the offspring
        pop[:] = offspring

        # Gather all the fitnesses in one list and print the stats
        fits = [ind.fitness.values[0] for ind in pop]

        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x * x for x in fits)
        std = abs(sum2 / length - mean ** 2) ** 0.5

        print('[GA] Results for generation {0}'.format(g))
        print('[GA] Min %s' % min(fits))
        print('[GA] Max %s' % max(fits))
        print('[GA] Avg %s' % mean)
        print('[GA] Std %s' % std)

        best_ind_g = tools.selBest(pop, 1)[0]

        # Store the best individual over all generations
        if best_ind_g.fitness.values[0] > best_fitness_ever:
            best_fitness_ever = best_ind_g.fitness.values[0]
            best_ind_ever = best_ind_g

    #     log('[GA] Best individual for generation {0}: {1}, {2}'.format(g, best_ind_g, best_ind_g.fitness.values[0]))
    #     log('[GA] Best individual ever till now: %s, %s' % (best_ind_ever, best_fitness_ever))
    #
    #     log('[GA] ############################# End of generation {0} #############################'.format(g))
    #
    # log('[GA] ===================== End of evolution =====================')

    best_ind = tools.selBest(pop, 1)[0]
    # log('[GA] Best individual in the population: %s, %s' % (best_ind, best_ind.fitness.values[0]))
    # log('[GA] Best individual ever: %s, %s' % (best_ind_ever, best_fitness_ever))

if __name__ == "__main__":
    main()
