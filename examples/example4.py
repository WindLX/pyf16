import pyf16
import logging
import numpy as np
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.ERROR)

aero_model = pyf16.AerodynamicModel("./models/f16_model")
aero_model.install("./models/f16_model/data")
control_limits = aero_model.load_ctrl_limits()

trim_target = pyf16.TrimTarget(15000, 500, None, None)
trim_init = None
trim_result = pyf16.trim(aero_model, trim_target, control_limits, trim_init)

print(trim_result.state.to_list())
print(trim_result.control.to_list())
print(trim_result.state_extend.to_list())

state_names = [
    "npos",
    "epos",
    "altitude",
    "phi",
    "theta",
    "psi",
    "velocity",
    "alpha",
    "beta",
    "p",
    "q",
    "r",
]

solvers = [
    pyf16.SolverType.RK1,
    pyf16.SolverType.RK2,
    pyf16.SolverType.RK3,
    pyf16.SolverType.RK4,
]
solver_names = ["Euler", "RK2", "RK3", "RK4"]
colors = ["b", "g", "r", "c"]

fig, axs = plt.subplots(4, 3, figsize=(15, 10))
axs = axs.flatten()

reference_solver = pyf16.SolverType.RK4
reference_name = "RK4"
reference_color = "c"

# Run the reference solver
f16_ref = pyf16.PlaneBlock(
    reference_solver,
    0.01,
    aero_model,
    trim_result.to_core_init(),
    [0, 0, 0],
    control_limits,
)

reference_states = []
for i in range(1000):
    core_output = f16_ref.update(trim_result.control, 0.01 * i)
    reference_states.append(core_output.state.to_list())

reference_states = list(zip(*reference_states))

# Run other solvers and calculate differences
for solver, solver_name, color in zip(solvers, solver_names, colors):
    if solver == reference_solver:
        continue

    f16 = pyf16.PlaneBlock(
        solver,
        0.01,
        aero_model,
        trim_result.to_core_init(),
        [0, 0, 0],
        control_limits,
    )

    states = []
    for i in range(1000):
        core_output = f16.update(trim_result.control, 0.01 * i)
        states.append(core_output.state.to_list())

    states = list(zip(*states))

    for i, (state, ref_state, name) in enumerate(
        zip(states, reference_states, state_names)
    ):
        if name in ["phi", "theta", "psi", "alpha", "beta"]:
            state = np.degrees(state)
            ref_state = np.degrees(ref_state)
        diff = np.abs(np.array(state) - np.array(ref_state))
        axs[i].plot(
            diff, label=f"{name} ({solver_name} - {reference_name})", color=color
        )
        axs[i].set_xlabel("Time Step")
        axs[i].set_ylabel("Difference Value")
        axs[i].set_title(f"{name.capitalize()} Difference Over Time")
        axs[i].set_yscale("log")
        axs[i].legend()

f16_ref.delete_model()
f16.delete_model()
aero_model.uninstall()

plt.tight_layout()
plt.show()
