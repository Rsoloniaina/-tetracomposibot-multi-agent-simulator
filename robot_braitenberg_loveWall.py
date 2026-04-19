from robot import * 

nb_robots = 0
debug = True

class Robot_player(Robot):

    team_name = "Braitenberg_LoveWall"
    robot_id = -1
    iteration = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots+=1
        super().__init__(x_0, y_0, theta_0, name=name, team=team)

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        sensor_to_wall = []
        sensor_to_robot = []
        for i in range (0,8):
            if  sensor_view[i] == 1:
                sensor_to_wall.append( sensors[i] )
                sensor_to_robot.append(1.0)
            elif  sensor_view[i] == 2:
                sensor_to_wall.append( 1.0 )
                sensor_to_robot.append( sensors[i] )
            else:
                sensor_to_wall.append(1.0)
                sensor_to_robot.append(1.0)

        if debug == True:
            if self.iteration % 100 == 0:
                print ("Robot",self.robot_id," (team "+str(self.team_name)+")","at step",self.iteration,":")
                print ("\tsensors (distance, max is 1.0)  =",sensors)
                print ("\t\tsensors to wall  =",sensor_to_wall)
                print ("\t\tsensors to robot =",sensor_to_robot)
                print ("\ttype (0:empty, 1:wall, 2:robot) =",sensor_view)
                print ("\trobot's name (if relevant)      =",sensor_robot)
                print ("\trobot's team (if relevant)      =",sensor_team)
                
        #loveWall: va vers les murs et ignore les robots
        
        front_wall = 1.0 - sensor_to_wall[sensor_front]
        left_wall  = 1.0 - sensor_to_wall[sensor_front_left]
        right_wall = 1.0 - sensor_to_wall[sensor_front_right]

        # plus un mur est proche devant/gauche/droite, plus on avance
        translation = 0.1 + 0.9 * (front_wall + left_wall + right_wall)

        # on tourne vers le côté où le mur est le plus proche
        rotation = 2.0 * (left_wall - right_wall) + 0.10 * ( (self.iteration % 20) - 10 )
        
        self.iteration = self.iteration + 1        
        return translation, rotation, False
        
