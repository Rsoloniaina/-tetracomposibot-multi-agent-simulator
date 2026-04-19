
# Exercice 2-effet des conditions initiales 
from robot import *
import math
import random
# --- paramètres de l'expérience ---
NB_STRATEGIES   = 500        # nombre de stratégies différentes
NB_ITER_EVAL    = 400        # durée d'UNE évaluation (un run)
NB_EVALS_PER_STRATEGY = 3    # ici: 3 orientations différentes par stratégie
NB_ITER_REPLAY  = 1000       # durée d'un replay du meilleur (boucle)

nb_robots = 0
debug = False

class Robot_player(Robot):

    team_name = "RandomSearch2"
    robot_id = -1
    iteration = 0

    # 8 paramètres (biais + 3 poids) pour translation, puis (biais + 3 poids) pour rotation
    param = []

    # best
    best_score = -1e18
    best_param = None
    best_trial = -1

    # --- score courant ---
    score_run = 0.0          # score du run en cours (une orientation)
    score_strategy = 0.0     # score total de la stratégie (somme des 3 runs)

    prev_log_trans = 0.0
    prev_log_rot = 0.0

    # --- gestion des runs ---
    trial = 0                         # numéro de stratégie
    run_id = 0                        # 0,1,2 pour les 3 orientations
    eval_thetas = [0, 0, 0]           # les 3 orientations FIXES de la stratégie

    mode = "search"                  # "search" ou "replay"
    replay_iter = 0

    # position initiale (on garde la même position pour comparer)
    x_0 = 0
    y_0 = 0
    theta_0 = 0

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a", evaluations=0, it_per_evaluation=0):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1

        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0

        # 1ère stratégie aléatoire
        self.param = [random.choice([-1, 0, 1]) for _ in range(8)]

        # orientations pour cette stratégie (fixées une fois)
        self.eval_thetas = [random.randint(0, 359) for _ in range(NB_EVALS_PER_STRATEGY)]
        self.run_id = 0
        self.theta_0 = self.eval_thetas[self.run_id]

        super().__init__(x_0, y_0, self.theta_0, name=name, team=team)

    def reset(self):
        # reset du simulateur (remet le robot à x_0,y_0,theta_0)
        super().reset()

        # reset des compteurs "effectifs"
        self.prev_log_trans = 0.0
        self.prev_log_rot = 0.0

        # reset score du run (pas la stratégie entière !)
        self.score_run = 0.0

    def _compute_control(self, sensors):
        # perceptron (3 capteurs utilisés)
        s_fl = sensors[sensor_front_left]
        s_f  = sensors[sensor_front]
        s_fr = sensors[sensor_front_right]

        translation = math.tanh(
            self.param[0] + self.param[1]*s_fl + self.param[2]*s_f + self.param[3]*s_fr
        )
        rotation = math.tanh(
            self.param[4] + self.param[5]*s_fl + self.param[6]*s_f + self.param[7]*s_fr
        )
        return translation, rotation

    def _update_score_effective(self):
        # valeurs effectives via les logs internes (déplacement réellement mesuré)
        delta_trans = self.log_sum_of_translation - self.prev_log_trans
        delta_rot   = self.log_sum_of_rotation - self.prev_log_rot

        self.prev_log_trans = self.log_sum_of_translation
        self.prev_log_rot   = self.log_sum_of_rotation

        # score demandé
        self.score_run += delta_trans * (1.0 - abs(delta_rot))

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # 1) Mise à jour du score (à partir du pas 1)
        if self.iteration > 0:
            self._update_score_effective()

        # 2) Fin d'un run (400 itérations)
        if self.mode == "search" and self.iteration > 0 and (self.iteration % NB_ITER_EVAL == 0):

            # on ajoute le score de ce run au score total de la stratégie
            self.score_strategy += self.score_run

            if debug:
                print(f"[END RUN] trial={self.trial} run_id={self.run_id} theta={self.eval_thetas[self.run_id]}")
                print("  run_score =", self.score_run, "  strategy_score_so_far =", self.score_strategy)

            # run suivant ?
            self.run_id += 1

            if self.run_id < NB_EVALS_PER_STRATEGY:
                # on garde les MÊMES paramètres, mais on change l'orientation initiale
                self.theta_0 = self.eval_thetas[self.run_id]

                # demander reset pour repartir de x_0,y_0 avec nouvelle orientation
                self.iteration += 1
                return 0.0, 0.0, True

            # sinon: on a fini les 3 runs => fin d'évaluation de la stratégie
            print(f"\n[END EVAL] trial = {self.trial}")
            print("  thetas  =", self.eval_thetas)
            print("  params  =", self.param)
            print("  score   =", self.score_strategy)

            # mise à jour du best
            if self.score_strategy > self.best_score:
                self.best_score = self.score_strategy
                self.best_param = list(self.param)
                self.best_trial = self.trial
                print(f"  >>> NEW BEST! best_score={self.best_score} found at trial {self.best_trial}")

            # prochaine stratégie
            self.trial += 1

            # budget atteint ?
            if self.trial >= NB_STRATEGIES:
                print("\n==== RANDOM SEARCH 2 FINISHED ====")
                print("BEST SCORE =", self.best_score)
                print("BEST TRIAL =", self.best_trial)
                print("BEST PARAM =", self.best_param)

                # passer en replay
                self.mode = "replay"
                self.replay_iter = 0

                # remettre un theta simple pour le replay (par ex le premier theta du dernier set)
                self.theta_0 = 0

                # reset pour commencer le replay
                self.iteration += 1
                return 0.0, 0.0, True

            # sinon on tire une nouvelle stratégie + ses 3 orientations fixes
            self.param = [random.choice([-1, 0, 1]) for _ in range(8)]
            self.eval_thetas = [random.randint(0, 359) for _ in range(NB_EVALS_PER_STRATEGY)]
            self.run_id = 0
            self.theta_0 = self.eval_thetas[self.run_id]

            # reset des scores stratégie (nouvelle candidate)
            self.score_strategy = 0.0

            print(f"\nTrying strategy no. {self.trial}")

            # reset pour démarrer le run 0
            self.iteration += 1
            return 0.0, 0.0, True

        # 3) Replay du meilleur (par blocs de 1000 itérations)
        if self.mode == "replay":
            self.param = list(self.best_param)

            self.replay_iter += 1
            if self.replay_iter >= NB_ITER_REPLAY:
                self.replay_iter = 0
                print("\n[REPLAY LOOP] restarting 1000-iter replay of best strategy")
                print("[REPLAY] best_score =", self.best_score, "best_trial =", self.best_trial)
                print("best_param =", self.best_param)

                self.iteration += 1
                return 0.0, 0.0, True

        # 4) Contrôle normal
        translation, rotation = self._compute_control(sensors)

        self.iteration += 1
        return translation, rotation, False

