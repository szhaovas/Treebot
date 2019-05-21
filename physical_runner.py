import cma
import genome2csv
import numpy as np
import pickle

def get_fitness( mutants, gen ):
    # return a vector of fitnesses by inputting into terminal the calculated fitness value of
    # each genome

    fitnesses = []
    for i, mutant in enumerate(mutants):
        # for each mutant manually input the average fitness value over the two tested genomes.
        # you can also use this to input and load a file if that is easier for you.
        # i.e. If you store the cylinder's is seen data in a file, you can modify this code to directly
        # take that file and calculate the fitness.
        print( '\n' )
        print( 'Please input fitness for genome ' + str( i ) + ' of generation ' + str( gen ) )
        fitness = float( input() )
        fitnesses.append( fitness )

    return fitnesses


if __name__ == "__main__":
    import sys

    # this is a test file name
    # DO NOT USE THIS FOR EXPERIMENTS
    input_genome = 'genome.txt'

    if len(sys.argv) > 1: # if there is an input file name, replace the input genome
        input_genome = sys.argv[1]

    # this is the starting genome for cma-es.
    starting_genome = genome2csv.load_genome( input_genome )

    # this is the number of new genomes you need to test each generation
    # increase this number if evaluating new genomes is quick
    # this will increase the number of genomes you need to test
    popsize = 10

    # these stay constant over generational time.
    # use them to determine which neurons are connected
    # and how much each motor can move
    expression_matrix = starting_genome['expression']
    joint_vector = starting_genome['joints']

    # this is what cma-es will update over generational time
    weight_matrix = starting_genome['weights']

    # we need to flatten the matrix for cma-es to us
    weight_input = weight_matrix.flatten()
    m, n = np.shape( weight_matrix )


    es = cma.CMAEvolutionStrategy( weight_input, 0.5, { 'popsize' : popsize } )

    should_end = False # flag for ending
    generation = 0
    max_gens = 10

    while should_end is not True:
        print( '\nStarting Generation', generation )

        mutants = es.ask() # gets mutant genomes

        print( '    saving genomes' )
        # save mutant genomes to output files
        for i, mutant_weight in enumerate( mutants ):
            mutant_genome = { 'weights'    : mutant_weight,
                              'expression' : expression_matrix,
                              'joints'     : joint_vector }

            # edit this to whatever is best for you
            # i.e. add a directory or whatever helps
            genome_save_name = 'gen-' + str( generation ) + '-' + str(i) + '.genome'

            # save the genome
            genome2csv.save_genome( mutant_genome, genome_save_name )
            print( '        Saved genome to', genome_save_name )
        # perform evaluations and input
        es.tell( mutants, get_fitness( mutants, generation ) )
        # save cma object
        pickle.dump(es, open('_saved-cma-object.pkl', 'wb'))
        generation += 1

        # stopping criteria, you can change this to suit your needs
        if generation > max_gens:
            should_end = True

    # whichever genome performed the best in two environments,
    # you should now evaluate the genome in all four environments
    # ideally your fitness should be close to 1 in two environments before you do this.
