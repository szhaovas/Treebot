import numpy as np
import genome2csv


# modified from the file readers in genome2csv.py
def read_activations(activations_filename):

    with open(activations_filename, 'r') as f:
        activations = f.readline().split(' ')
        activations = [float(a) for a in activations]
        activations = np.array(activations)

    return activations


# modified from the file readers in genome2csv.py
def read_sensors(sensors_filename):

    with open(sensors_filename, 'r') as f:
        sensors = f.readline().split(' ')
        sensors = [float(s) for s in sensors]
        sensors = np.array(sensors)

    return sensors


# save newly generated activations and motor parameters for other programs to
# read
def save(activations, parameters, activations_filename='activations',
         parameters_filename='parameters'):

    with open(activations_filename, 'w+') as a, open(parameters_filename,
                                                     'w+') as p:
        for activation in activations:
            a.write(str(activation) + ' ')
        a.write('\n')

        for parameter in parameters:
            p.write(str(parameter) + ' ')
        p.write('\n')


# generate activations for the hidden and motor neurons based on sensor values
# and genome, then generate motor parameters based on the newly generated
# activations and genome
def generate_activations_parameters(genome_filename,
                                    activations_filename='activations',
                                    sensors_filename='sensors'):

    # read the network connection matrix from genome_filename
    genome = genome2csv.load_genome(genome_filename)
    connections = genome['weights'] * genome['expression']

    # read the activations from the previous time step and concatenate sensor
    # values to the end
    # this new 1d array will be used to generate new activations
    activations = read_activations(activations_filename)
    sensors = read_sensors(sensors_filename)
    activations_and_sensors = np.concatenate((activations, sensors))

    # for each column in the connection matrix, multiply the column by the
    # activation vector and apply tanh to the sum to obtain the new activation
    # for the node corresponding to this column
    num_hm_nodes = connections.shape[1]
    new_activations = np.zeros(num_hm_nodes)
    for col in range(num_hm_nodes):
        col_activation = np.tanh(np.sum(connections[:, col]
                                        * activations_and_sensors))
        new_activations[col] = col_activation

    # based on genome joint values and motor activations. generate the motor
    # parameters for arduino
    joints = genome['joints']
    num_motors = joints.shape[0]
    motor_activations = new_activations[-num_motors:]
    motor_parameters = np.zeros(joints.shape)
    for motor_num in range(num_motors):
        # for now I'm assuming the max range of all position servos are 180
        parameter = motor_activations[motor_num] * joints[motor_num] / 2 * 90 \
            + 90
        motor_parameters[motor_num] = parameter

    save(new_activations, motor_parameters)


if __name__ == '__main__':
    generate_activations_parameters('2019-01-25-test.genome')
