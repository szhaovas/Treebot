import numpy as np


def save_genome(genome, filename):
    # saves the genome to a csv delimited by spaces
    # matrices are flattened in row-major style as
    # it is the default of numpy

    with open(filename, 'w') as f:
        # write weights first
        flattened_weights = genome['weights'].flatten()
        for weight in flattened_weights:
            f.write(str(weight) + ' ')
        f.write('\n')

        # expression second
        flattend_expression = genome['expression'].flatten()
        for exp in flattend_expression:
            f.write(str(exp) + ' ')
        f.write('\n')

        # joints third
        for joint in genome['joints']:
            f.write(str(joint) + ' ')
        f.write('\n')


def load_genome(filename, n_sensors=2, n_hidden=9, n_motors=3):
    # loads file at filename and creates genome

    matrix_shape = (n_sensors + n_hidden + n_motors, n_hidden + n_motors)

    with open(filename, 'r') as f:
        # read in weights
        weights = f.readline().split(' ')[:-1]  # removes \n from the end
        weights = [float(w) for w in weights]
        weights = np.array(weights).reshape(matrix_shape)

        # read in expression
        expression = f.readline().split(' ')[:-1]  # removes \n from the end
        expression = [int(e) for e in expression]
        expression = np.array(expression).reshape(matrix_shape)

        # read in joints
        joints = f.readline().split(' ')[:-1]
        joints = [float(j) for j in joints]
        joints = np.array(joints)

    genome = {'weights': weights,
              'expression': expression,
              'joints': joints}
    return genome


def create_random_genome(n_sensors, n_hidden, n_motors):
    # creates a random genome of desired size

    # each genome is made up of three components:
    # 1. a synaptic weight matrix ( 14 x 12 of floats )
    # 2. an expression matrix ( 14 x 12 of boolean values )
    # 3. a vector of joint ranges ( 3 floats )

    # there are 14 total neurons
    #   2 sensors sL, sR.
    #       sL is the left sensor, sR is the right sensor
    #   3 x 3 = 9 hidden neurons
    #       3 per branch of the tree h0,...,h8
    #   3 x 1 = 3 motor neurons, m0, mL, mR
    #       1 for each branch.
    #            m0 is for the root
    #            mL is for the left leaf
    #            mR is for the right leaf

    # the matrices are orginized as follows
    #          0 ... 8 , 9 , 10 , 11
    #         h0 ... h8, m0, mL , mR
    #        ---------------------
    # 0  h0 |
    #   .   |
    #   .   |
    # 8  h8 |
    # 9  m0 |
    # 10 mL |
    # 11 mR |
    # 12 sL |
    # 13 sR |

    # this way, for example, if we want the weight from the left sensor to the
    # right motor it would be weight_matrix[12, 11]
    n_neurons = n_sensors + n_hidden + n_motors

    # scaled between -1.5, +1.5
    weight_matrix = np.random.random((n_neurons, n_neurons - n_sensors)) \
        * 3.0 - 1.5
    expression_matrix = np.random.random_integers(low=0, high=1,
                                                  size=(n_neurons, n_neurons
                                                        - n_sensors))
    joint_vector = np.random.random(n_motors)

    genome = {'weights': weight_matrix,
              'expression': expression_matrix,
              'joints': joint_vector}

    return genome


if __name__ == '__main__':

    # random genome with set test seed
    np.random.seed(0)

    n_sensors = 2
    n_hidden = 9
    n_motors = 3

    genome = create_random_genome(n_sensors, n_hidden, n_motors)
    test_file_name = '2019-01-25-test.genome'
    save_genome(genome, test_file_name)
    loaded_genome = load_genome(test_file_name)

    print(genome, loaded_genome)
