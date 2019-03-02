import numpy as np
import matplotlib.pyplot as plt

def get_fk_branches( angles, leaf_offsets, root_length, leaf_length ):

    root_base = np.array( [ 0, 0  ] )
    root_tip = np.array( [ root_length * np.sin( angles[0] ), 
                           root_length * np.cos( angles[0] ) ] )

    left_base = np.copy( root_tip )
    right_base = np.copy( root_tip )

    left_direction = np.array( [ np.sin( leaf_offsets[0] + angles[0] + angles[1] ), np.cos( angles[0] + angles[1] + leaf_offsets[0] ) ] )
    right_direction = np.array( [ np.sin( leaf_offsets[1] + angles[0] + angles[2] ), np.cos( angles[0] + angles[2] + leaf_offsets[1] ) ] )
    left_tip = left_base + left_direction * leaf_length
    right_tip = right_base + right_direction * leaf_length

    return [ np.array( [ root_base, root_tip ] ),
             np.array( [ left_base, left_tip ] ),
             np.array( [ right_base, right_tip ] )
            ]

def check_is_seen( angles, leaf_offsets, cylinder_positions, cylinder_radius, root_length, leaf_length ):

    root, left_branch, right_branch = get_fk_branches( angles, leaf_offsets, root_length, leaf_length )

    is_seen = [ False, False ]



    for branch in [ left_branch, right_branch ]:
        # get minimum distance from ray projection to center point of cylinder

        # find line projection from branch
        # find line frome each branch in form
        # ax + by + c = 0
        # 𝑎=𝑦1−𝑦2,𝑏=𝑥2−𝑥1,𝑐=(𝑥1−𝑥2)𝑦1+𝑥1(𝑦2−𝑦1)
        
        x1 = branch[ 0, 0 ]
        x2 = branch[ 1, 0 ] 
        y1 = branch[ 0, 1 ]
        y2 = branch[ 1, 1 ]

        a = y1 - y2
        b = x2 - x1
        c = ( x1 - x2 ) * y1 + x1 * ( y2 - y1 )

        for i, cylinder_position in enumerate( cylinder_positions ):
            # project and see if looking at cylinder
            x, y = cylinder_position

            distance = np.abs( a * x + b * y + c ) / np.sqrt( a * a + b * b )

            if ( distance < cylinder_radius ):
                is_seen[i] = True 

    return is_seen, [ root, left_branch, right_branch ]

if __name__ == "__main__":

    # change these to measurements in reality

    # [ root, left, right ]
    # uncomment to try different angle setups
    angles = [ 0, 0, 0 ]
    # angles = [ 0, np.pi / 8.0, 0 ]
    # angles = [ 0, -np.pi / 4.0, np.pi / 4.0 ]
    # angles = [ np.pi / 4, np.pi / 10, 0 ]

    leaf_offsets = [ -np.pi / 4.0, np.pi / 4.0 ]

    root_length = 18 # cm
    leaf_length = 15 # cm

    # cylinder positions
    left_cyl_position = ( -30, 40 ) # cm
    right_cyl_position = ( +30, 40 ) # cm

    cylinder_radius = 10 # cm

    is_seen, branches = check_is_seen( angles, leaf_offsets, [ left_cyl_position, right_cyl_position ], cylinder_radius, root_length, leaf_length )

    print( is_seen )

    # draw branches 
    for branch in branches:
        plt.plot( [ branch[0][0], branch[1][0] ], [ branch[0][1], branch[1][1] ], 'k' )

    # draw cicles representing cylinders
    in_vals = np.linspace( 0, 1 )

    left_x_points = np.cos( in_vals * 2 * np.pi ) * cylinder_radius + left_cyl_position[0]
    left_y_points = np.sin( in_vals * 2 * np.pi ) * cylinder_radius + left_cyl_position[1]

    right_x_points = np.cos( in_vals * 2 * np.pi ) * cylinder_radius + right_cyl_position[0]
    right_y_points = np.sin( in_vals * 2 * np.pi ) * cylinder_radius + right_cyl_position[1]

    if is_seen[0] == True:
        color = 'g'
    else:
        color = 'k'
    plt.plot( left_x_points, left_y_points, color = color, lw = 5 )

    if is_seen[1] == True:
        color = 'g'
    else:
        color = 'k'   
    plt.plot( right_x_points, right_y_points, color = color, lw = 5 )

    plt.gca().set_aspect( 'equal' )
    plt.ylim( 0, 50 )
    plt.show()
