import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import numpy as np

# Definir el robot UR5 utilizando los parámetros DH ajustados

ur5 = rtb.models.DH.UR5()

# Definir la pose deseada (posición y orientación)

T_desired = SE3(0.49,0.66,0.48) * SE3.Rz(np.pi/2)

# Probar diferentes configuraciones iniciales para la cinemática inversa
initial_guesses = [
    np.zeros(6),                      # Todos los ángulos en 0
    np.array([np.pi, 0, 0, 0, 0, 0]),
    np.array([-np.pi, 0, 0, 0, 0, 0]),
    np.array([0, np.pi, 0, 0, 0, 0]),
    np.array([0, -np.pi, 0, 0, 0, 0]),
    np.array([0, 0, np.pi, 0, 0, 0]),
    np.array([0, 0, -np.pi, 0, 0, 0]),
    np.array([np.pi, -np.pi, np.pi, -np.pi, np.pi, -np.pi]),
]

# Almacenar soluciones únicas
unique_solutions = set()

for q0 in initial_guesses:
    solution = ur5.ikine_LM(T_desired, q0=q0)
    if solution.success:
        q_normalized = tuple((solution.q + np.pi) % (2 * np.pi) - np.pi)
        unique_solutions.add(q_normalized)

# Imprimir las soluciones ajustadas y únicas en radianes
print("Todas las posibles posiciones únicas de las articulaciones en radianes ajustadas al rango [-π, π] para agarrar la botella:")
for idx, q in enumerate(unique_solutions):
    print(f"Solución {idx + 1}: {q}")
