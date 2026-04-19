# Configuration file.

import arenas

# general -- first three parameters can be overwritten with command-line arguments (cf. "python tetracomposibot.py --help")

display_mode = 1
arena = 1
position = False 

# affichage

display_welcome_message = False
verbose_minimal_progress = False # display iterations
display_robot_stats = False
display_team_stats = False
display_tournament_results = False
display_time_stats = True

# optimisation 

evaluations = 500          # nombre de stratégies différentes
nb_init_conditions = 3     # 3 orientations initiales
it_per_evaluation = 400    # durée d'une évaluation
replay_iterations = 2000   # pour rejouer le meilleur comportement

max_iterations = evaluations * nb_init_conditions * it_per_evaluation + replay_iterations

# initialization : create and place robots at initial positions (returns a list containing the robots)

import robot_optimize
import robot_randomsearch2

def initialize_robots(arena_size=-1, particle_box=-1): # particle_box: size of the robot enclosed in a square
    x_center = arena_size // 2 - particle_box / 2
    y_center = arena_size // 2 - particle_box / 2
    robots = []
    robots.append(robot_randomsearch2.Robot_player(x_center, y_center, 0, name="My Robot", team="RandomSearch2",evaluations=evaluations,it_per_evaluation=it_per_evaluation)) # start from left: 4, y_center
    return robots
