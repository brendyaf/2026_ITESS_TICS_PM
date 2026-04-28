from sim2D import sim_run

options = {}

options['FIG_SIZE'] = [8,8]

# Cambia aquí:
# 'a' = recto
# 'b' = derecha
# 'c' = izquierda
# 'd' = combinado
options['MOVEMENT'] = 'd'

class KalmanFilter:
    def __init__(self):
        pass

sim_run(options, KalmanFilter)
