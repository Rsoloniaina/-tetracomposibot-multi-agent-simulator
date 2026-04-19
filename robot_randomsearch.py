
# Exercice 1 - Recherche aléatoire
from robot import *
import math
import random


NB_STRATEGIES = 500        # nombre de comportements testés
NB_ITER_EVAL = 400         # durée d'évaluation d'un comportement
NB_ITER_REPLAY = 1000      # durée d'affichage du meilleur comportement ern boucle

nb_robots = 0
debug = False


class Robot_player(Robot):

    team_name = "RandomSearch"
    robot_id = -1
    iteration = 0

    # paramètres du perceptron (8 poids : biais+3 capteurs pour translation puis biais+3 capteurs pour rotation)
    param = []

    # suivi recherche
    trial = 0
    it_per_evaluation = NB_ITER_EVAL

    # meilleur individu
    best_score = -1e18
    best_param = None
    best_trial = -1

    # score courant (somme sur itérations)
    score_current = 0.0

    # pour calculer les incréments effectifs
    prev_log_trans = 0.0
    prev_log_rot = 0.0

    # mode : "search" puis "replay"
    mode = "search"
    replay_iter = 0

    # init pose
    x_0 = 0
    y_0 = 0
    theta_0 = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a", evaluations=0, it_per_evaluation=NB_ITER_EVAL):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1

        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0

        self.it_per_evaluation = it_per_evaluation

        # première stratégie aléatoire
        self.param = [random.choice([-1, 0, 1]) for _ in range(8)]

        super().__init__(x_0, y_0, theta_0, name=name, team=team)

    def reset(self):
        # reset moteur + logs internes (log_sum_of_translation/rotation sont remis à zéro côté Robot)
        super().reset()

        # reset des accumulateurs de score pour une nouvelle évaluation ou un replay
        self.score_current = 0.0
        self.prev_log_trans = 0.0
        self.prev_log_rot = 0.0

    def _compute_control(self, sensors):
        # Perceptron tanh : 3 senseurs utilisés (front_left, front, front_right) + biais
        translation = math.tanh(
            self.param[0]
            + self.param[1] * sensors[sensor_front_left]
            + self.param[2] * sensors[sensor_front]
            + self.param[3] * sensors[sensor_front_right]
        )

        rotation = math.tanh(
            self.param[4]
            + self.param[5] * sensors[sensor_front_left]
            + self.param[6] * sensors[sensor_front]
            + self.param[7] * sensors[sensor_front_right]
        )
        return translation, rotation

    def _update_score_effective(self):
        """
        Score demandé :
            score += trans_eff * (1 - abs(rot_eff))

        Ici on reconstruit trans_eff et rot_eff par incréments des cumuls effectifs.
        """
        delta_trans = self.log_sum_of_translation - self.prev_log_trans
        delta_rot = self.log_sum_of_rotation - self.prev_log_rot

        self.score_current += delta_trans * (1.0 - abs(delta_rot))

        self.prev_log_trans = self.log_sum_of_translation
        self.prev_log_rot = self.log_sum_of_rotation

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # met à jour le score à partir du mouvement *effectif* réalisé depuis le pas précédent
        if self.iteration > 0:
            self._update_score_effective()

        # toutes les NB_ITER_EVAL itérations : fin d'une évaluation (ou fin d'un segment de replay)
        if self.iteration % self.it_per_evaluation == 0:

            # fin d'une évaluation (pas au tout début)
            if self.iteration > 0:
                if self.mode == "search":
                    print("\n[END EVAL] trial =", self.trial)
                    print("  params =", self.param)
                    print("  score  =", self.score_current)

                    if self.score_current > self.best_score:
                        self.best_score = self.score_current
                        self.best_param = self.param.copy()
                        self.best_trial = self.trial
                        print("  >>> NEW BEST! best_score =", self.best_score, "found at trial", self.best_trial)

                elif self.mode == "replay":
                    # on peut afficher de temps en temps le score du meilleur (fixe)
                    if self.replay_iter == 0:
                        print("\n[REPLAY] best_score =", self.best_score, "best_trial =", self.best_trial)
                        print("  best_param =", self.best_param)

            # ----------- MODE SEARCH -----------
            if self.mode == "search":
                # incrément du compteur de stratégies après chaque bloc de 400 itérations
                self.trial += 1

                # budget épuisé => on passe en mode replay
                if self.trial >= NB_STRATEGIES:
                    print("\n==== RANDOM SEARCH FINISHED ====")
                    print("BEST SCORE =", self.best_score)
                    print("BEST TRIAL =", self.best_trial)
                    print("BEST PARAM =", self.best_param)

                    self.mode = "replay"
                    self.replay_iter = 0
                    self.param = self.best_param.copy()

                else:
                    # nouvelle stratégie aléatoire
                    print("\nTrying strategy no.", self.trial)
                    self.param = [random.choice([-1, 0, 1]) for _ in range(8)]

                # demande un reset (position initiale)
                self.iteration += 1
                return 0.0, 0.0, True

            # ----------- MODE REPLAY -----------
            if self.mode == "replay":
                # on rejoue le meilleur par blocs de 400, mais on veut une boucle totale de 1000 iters
                # on compte en "vrai" pas de simulation via self.replay_iter
                self.replay_iter += self.it_per_evaluation

                # garde le meilleur param
                self.param = self.best_param.copy()

                # toutes les 1000 itérations, reset pour rejouer à l'infini
                if self.replay_iter >= NB_ITER_REPLAY:
                    self.replay_iter = 0
                    print("\n[REPLAY LOOP] restarting 1000-iter replay of best strategy")

                self.iteration += 1
                return 0.0, 0.0, True

        # contrôle normal (pendant l'évaluation / replay)
        translation, rotation = self._compute_control(sensors)

        if debug and self.iteration % 100 == 0:
            print("Robot", self.robot_id, "(team", self.team_name + ")", "step", self.iteration)
            print("  sensors =", sensors)

        self.iteration += 1
        return translation, rotation, False

