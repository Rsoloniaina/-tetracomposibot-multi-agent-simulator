from robot import * 

nb_robots = 0
debug = True

class Robot_player(Robot):

    team_name = "Subsomption"
    robot_id = -1
    iteration = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots+=1
        super().__init__(x_0, y_0, theta_0, name=name, team=team)


    # Comportement 1 : aller tout droit
    def comportement_tout_droit(self):
        translation = 0.7
        rotation = 0.0
        return translation, rotation

    # Comportement 2 : éviter les murs (hateWall)
    def comportement_hateWall(self, sensor_to_wall):

        front_wall = 1.0 - sensor_to_wall[sensor_front]
        left_wall  = 1.0 - sensor_to_wall[sensor_front_left]
        right_wall = 1.0 - sensor_to_wall[sensor_front_right]

        translation = max(0.0, 0.9 - 1.3 * (front_wall + left_wall + right_wall))
        rotation    = 3.2 * (right_wall - left_wall) + 1.8 * front_wall

        return translation, rotation

    # Comportement 3 : aller vers les robots (loveBot)
    def comportement_loveBot(self, sensor_to_robot):

        front_bot = 1.0 - sensor_to_robot[sensor_front]
        left_bot  = 1.0 - sensor_to_robot[sensor_front_left]
        right_bot = 1.0 - sensor_to_robot[sensor_front_right]

        translation = 0.15 + 0.85 * (front_bot + left_bot + right_bot)
        rotation    = 2.0 * (left_bot - right_bot) + 0.15 * math.sin(0.2 * self.iteration)

        return translation, rotation

    # STEP : architecture de subsomption
    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # Séparer ce que voient les capteurs
        sensor_to_wall = []
        sensor_to_robot = []

        for i in range(0, 8):
            if sensor_view[i] == 1:      # mur
                sensor_to_wall.append(sensors[i])
                sensor_to_robot.append(1.0)
            elif sensor_view[i] == 2:    # robot
                sensor_to_wall.append(1.0)
                sensor_to_robot.append(sensors[i])
            else:                        # vide
                sensor_to_wall.append(1.0)
                sensor_to_robot.append(1.0)

        # Activations
        wall_front = 1.0 - sensor_to_wall[sensor_front]
        wall_left  = 1.0 - sensor_to_wall[sensor_front_left]
        wall_right = 1.0 - sensor_to_wall[sensor_front_right]
        danger_wall = wall_front + wall_left + wall_right

        bot_front = 1.0 - sensor_to_robot[sensor_front]
        bot_left  = 1.0 - sensor_to_robot[sensor_front_left]
        bot_right = 1.0 - sensor_to_robot[sensor_front_right]
        target_bot = bot_front + bot_left + bot_right

   
        # SUBSOMPTION (priorités)
        seuil_mur = 0.35
        seuil_robot = 0.10

        if danger_wall > seuil_mur:
            translation, rotation = self.comportement_hateWall(sensor_to_wall)
        elif target_bot > seuil_robot:
            translation, rotation = self.comportement_loveBot(sensor_to_robot)
        else:
            translation, rotation = self.comportement_tout_droit()

        # Debug
        if debug and self.iteration % 100 == 0:
            print("Robot", self.robot_id,
                  "danger_wall =", round(danger_wall, 3),
                  "target_bot =", round(target_bot, 3))

        self.iteration += 1
        return translation, rotation, False
