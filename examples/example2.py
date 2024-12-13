import pyf16
import numpy as np
import matplotlib.pyplot as plt

solver = pyf16.SimpleSolver(pyf16.SolverType.RK4, 0.01)


def dynamics(t: float, state: list, input: list) -> list:
    A = [[1, 0, 0], [0, 0, 0], [0, 0, 0]]
    B = [0, 0, 0]

    dxdt = [0, 0, 0]
    for i in range(len(A)):
        dxdt[i] = sum(A[i][j] * state[j] for j in range(len(state))) + B[i] * input[i]

    return dxdt


state_init = [1, 0, 0]
input_init = [0, 0, 0]
states = [state_init]
# Solve the system
for i in range(1000):
    state = solver.solve(dynamics, solver.delta_t * i, states[-1], input_init)
    states.append(state)

# Analytical solution for state 1
time = np.arange(0, 1001)
state1_analytical = np.exp(time / 100)

# Plot the results
plt.figure()
states = list(zip(*states))  # Transpose the list of states
plt.plot(states[0], label=f"State {0}")
plt.plot(time, state1_analytical, "r--", label="State 1 Analytical")
plt.xlabel("Time")
plt.ylabel("State")
plt.legend()
plt.title("State over Time")
plt.show()
