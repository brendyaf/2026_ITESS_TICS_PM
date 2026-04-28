import numpy as np
import matplotlib.pyplot as plt
import random

r = 0.05
L = 0.15
t = 1.5
factor = 4

goal = (1.8, 1.8)

def simulate(x, y, theta, wl, wr):
    v = (r/2)*(wr + wl)
    w = (r/L)*(wr - wl)

    x_new = x + v*np.cos(theta)*t
    y_new = y + v*np.sin(theta)*t
    theta_new = theta + w*t

    return x_new, y_new, theta_new


def collision(x, y):
    if x < 0 or x > 2 or y < 0 or y > 2:
        return True

    if 0 <= x <= 1.5 and 1.5 <= y <= 2:
        return True

    if 0.5 <= x <= 1.0 and 0.5 <= y <= 1.0:
        return True

    if 1.5 <= x <= 1.65 and 0.5 <= y <= 1.5:
        return True

    if 1.0 <= x <= 2.0 and 0 <= y <= 0.1:
        return True

    return False

def collision_line(x1, y1, x2, y2, steps=25):
    for i in range(steps):
        a = i / steps
        x = x1 + a*(x2 - x1)
        y = y1 + a*(y2 - y1)
        if collision(x, y):
            return True
    return False

acciones = [
    (1,1),(1,0.5),(0.5,1),(1,-1),(-1,1),
    (0.5,0.5),(-0.5,-0.5),(1,0),(0,1),(-1,0)
]


def generar_arbol(max_nodos=3000):
    nodos = [(0,0,0)]
    padres = {0: None}
    aristas = []
    resultados = [] 

    goal_idx = None

    for _ in range(max_nodos):

        if random.random() < 0.5:
            x_rand, y_rand = goal
        else:
            x_rand = random.uniform(0,2)
            y_rand = random.uniform(0,2)

        idx = min(range(len(nodos)),
                  key=lambda i: (nodos[i][0]-x_rand)**2 + (nodos[i][1]-y_rand)**2)

        x, y, theta = nodos[idx]

        for wl, wr in acciones:
            wl_s = wl * factor
            wr_s = wr * factor

            x_new, y_new, theta_new = simulate(x, y, theta, wl_s, wr_s)

            col = collision_line(x, y, x_new, y_new)

            resultados.append((x_new, y_new, not col))

            if not col:
                nodos.append((x_new, y_new, theta_new))
                new_idx = len(nodos)-1

                padres[new_idx] = idx
                aristas.append((x, y, x_new, y_new))

                # conexión final a meta
                if np.hypot(x_new - goal[0], y_new - goal[1]) < 0.3:
                    if not collision_line(x_new, y_new, goal[0], goal[1]):
                        nodos.append((goal[0], goal[1], 0))
                        goal_idx = len(nodos)-1

                        padres[goal_idx] = new_idx
                        aristas.append((x_new, y_new, goal[0], goal[1]))

                        return nodos, aristas, padres, goal_idx, resultados

    return nodos, aristas, padres, goal_idx, resultados

def reconstruir_camino(nodos, padres, idx):
    camino = []
    while idx is not None:
        camino.append(nodos[idx])
        idx = padres[idx]
    camino.reverse()
    return camino

# -----------------------------
# Gráfica árbol
# -----------------------------
def graficar(aristas, camino):
    plt.figure(figsize=(6,6))

    plt.xlim(0,2)
    plt.ylim(0,2)
    plt.title("RRT con trayectoria a meta")

    # obstáculos
    plt.gca().add_patch(plt.Rectangle((0,1.5),1.5,0.5, color='black'))
    plt.gca().add_patch(plt.Rectangle((0.5,0.5),0.5,0.5, color='black'))
    plt.gca().add_patch(plt.Rectangle((1.5,0.5),0.15,1.0, color='black'))
    plt.gca().add_patch(plt.Rectangle((1.0,0.0),1.0,0.1, color='black'))

    for x1, y1, x2, y2 in aristas:
        plt.plot([x1, x2], [y1, y2], linewidth=0.5)

    if camino:
        xs = [p[0] for p in camino]
        ys = [p[1] for p in camino]
        plt.plot(xs, ys, linewidth=3)

    plt.scatter(goal[0], goal[1], s=100)

    plt.grid()
    plt.show()

# -----------------------------
# Gráfica puntitos
# -----------------------------
def graficar_cspace(resultados):
    libres_x, libres_y = [], []
    col_x, col_y = [], []

    for x, y, libre in resultados:
        if libre:
            libres_x.append(x)
            libres_y.append(y)
        else:
            col_x.append(x)
            col_y.append(y)

    plt.figure(figsize=(6,6))

    plt.scatter(col_x, col_y, s=8, c='red', alpha=0.5, label="Colisión")
    plt.scatter(libres_x, libres_y, s=10, c='blue', alpha=0.8, label="Libre")

    plt.xlim(0,2)
    plt.ylim(0,2)

    plt.title("Mapa de espacio libre vs colisión")
    plt.legend()
    plt.grid()
    plt.show()


# Ejecutar

nodos, aristas, padres, goal_idx, resultados = generar_arbol(3000)

if goal_idx is not None:
    camino = reconstruir_camino(nodos, padres, goal_idx)
else:
    print("No llegó a la meta, vuelve a correr")
    camino = []

graficar(aristas, camino)
graficar_cspace(resultados)