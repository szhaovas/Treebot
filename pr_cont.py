import cma
import genome2csv
import physical_runner
import numpy as np
import pickle

if __name__ == "__main__":

    es = pickle.load(open('_saved-cma-object.pkl', 'rb'))
    max_gens = 10
    popsize = int(input('Please enter popsize:'))
    generation = input('Please enter generation:')

    mutants = []
    for i in range(popsize):
        filename = 'gen-'+generation+'-'+str(i)+'.genome'
        mutants.append(genome2csv.load_genome(filename)['weights'])

    es.tell(mutants, physical_runner.get_fitness(mutants, generation))
    generation += 1

    should_end = False
    if generation > max_gens:
        should_end = True

    starting_genome = genome2csv.load_genome('genome.txt')
    expression_matrix = starting_genome['expression']
    joint_vector = starting_genome['joints']

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
        es.tell( mutants, physical_runner.get_fitness( mutants, generation ) )
        generation += 1

        # stopping criteria, you can change this to suit your needs
        if generation > max_gens:
            should_end = True

    # whichever genome performed the best in two environments,
    # you should now evaluate the genome in all four environments
    # ideally your fitness should be close to 1 in two environments before you do this.
