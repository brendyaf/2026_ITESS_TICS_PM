import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

def sim_run(options, KalmanFilter):

    start = time.process_time()

    FIG_SIZE = options['FIG_SIZE']
    MOVEMENT = options['MOVEMENT']

    # Parámetros reales
    b = 11.5   # cm
    r = 2.8    # cm
    dt = 0.1

    # -------- MODELO --------
    def physics(t0, state):

        if len(state) == 0:
            x = 1
            y = 40
            theta = 0
        else:
            x = state[-1][0]
            y = state[-1][1]
            theta = state[-1][2]

        # -------- MOVIMIENTOS --------
        if MOVEMENT == 'a':  # recto
            if t0 < 4:
                wR = 2
                wL = 2
            else:
                return [x, y, theta] 

        elif MOVEMENT == 'b':  # derecha
            # Estado actual
            theta = state[-1][2] if len(state) > 0 else 0

            # 1) Recto inicial
            if t0 < 4:
                wR = 2
                wL = 2

            # 2) Giro hasta ~90° (derecha)
            elif theta > -np.pi/2:
                wR = 1
                wL = 2

            # 3) Recto después del giro
            elif t0 < 10:
                wR = 2
                wL = 2

            # 4) Detenerse
            else:
                return [x, y, theta]  # ← se detiene totalmente

        elif MOVEMENT == 'c':

            theta = state[-1][2] if len(state) > 0 else 0

            # 1) Recto inicial
            if t0 < 4:
                wR = 2
                wL = 2

            # 2) Giro hasta ~90° (izquierda)
            elif theta < np.pi/2:
                wR = 2
                wL = 1

            # 3) Recto después del giro
            elif t0 < 10:
                wR = 2
                wL = 2

            # 4) Detenerse
            else:
                return [x, y, theta]  # ← se detiene totalmente
            
        elif MOVEMENT == 'd':  # combinado
        
            if t0 < 10:
                wR = 2 + 1.5 * np.sin(2 * t0)
                wL = 2 - 1.5 * np.sin(2 * t0)

            elif t0 < 5:
                wR = 2
                wL = 2
                

            else:
                return [x, y, theta]  

        # -------- CINEMÁTICA --------
        dR = r * wR * dt
        dL = r * wL * dt

        d = (dR + dL) / 2
        dtheta = (dR - dL) / b

        x_new = x + d * np.cos(theta + dtheta/2)
        y_new = y + d * np.sin(theta + dtheta/2)
        theta_new = theta + dtheta

        return [x_new, y_new, theta_new]

    # -------- SIMULACIÓN --------
    state = []
    t = np.linspace(0.0, 60, 600)

    for t0 in t:
        state.append(physics(t0, state))

    print("Tiempo de cálculo:", round(time.process_time() - start, 3), "s")

    # -------- GRÁFICA --------
    fig, ax = plt.subplots(figsize=(FIG_SIZE[0], FIG_SIZE[1]))

    ax.set_xlim(0, 80)
    ax.set_ylim(0, 80)
    ax.set_title("Simulación Carrito(LEGO)")

    path_x, path_y = [], []

    # Trayectoria
    trayectoria, = ax.plot([], [], 'b-', label="Trayectoria")

    # Carrito (cuerpo + dirección)
    robot_body, = ax.plot([], [], 'r-', linewidth=3, label="Carrito")
    robot_dir, = ax.plot([], [], 'g-', linewidth=2)

    ax.legend()

    def update(frame):
        x = state[frame][0]
        y = state[frame][1]
        theta = state[frame][2]

        path_x.append(x)
        path_y.append(y)

        trayectoria.set_data(path_x, path_y)

        # Tamaño del carrito
        L = 6

        x_front = x + L * np.cos(theta)
        y_front = y + L * np.sin(theta)

        # cuerpo
        robot_body.set_data([x, x_front], [y, y_front])

        # dirección
        robot_dir.set_data([x, x_front], [y, y_front])

        return trayectoria, robot_body, robot_dir

    ani = animation.FuncAnimation(fig, update, frames=len(state), interval=50, repeat=False)

    plt.show()