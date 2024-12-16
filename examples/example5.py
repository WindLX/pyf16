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

time_steps = [0.1, 0.05, 0.01]
colors = ["b", "g", "r"]
solver = pyf16.SolverType.RK4
solver_name = "RK4"

fig, axs = plt.subplots(4, 3, figsize=(15, 10))
axs = axs.flatten()

for time_step, color in zip(time_steps, colors):
    f16 = pyf16.PlaneBlock(
        solver,
        time_step,
        aero_model,
        trim_result.to_core_init(),
        [0, 0, 0],
        control_limits,
    )

    states = []
    for i in range(int(10 / time_step)):
        core_output = f16.update(trim_result.control, time_step * i)
        states.append(core_output.state.to_list())

    states = list(zip(*states))
    time_points = np.arange(0, 10, time_step)

    for i, (state, name) in enumerate(zip(states, state_names)):
        if name in ["phi", "theta", "psi", "alpha", "beta"]:
            state = np.degrees(state)
        axs[i].plot(time_points, state, label=f"{name} (dt={time_step})", color=color)
        axs[i].set_xlabel("Time (s)")
        axs[i].set_ylabel("State Value")
        axs[i].set_title(f"{name.capitalize()} Evolution Over Time")
        axs[i].legend()

f16.delete_model()
aero_model.uninstall()

plt.tight_layout()
plt.show()
