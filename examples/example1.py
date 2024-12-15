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

f16 = pyf16.PlaneBlock(
    pyf16.SolverType.RK4,
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

states = list(zip(*states))  # Transpose the list of states

f16.delete_model()
aero_model.uninstall()

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

fig, axs = plt.subplots(
    len(state_names) // 3, 3, figsize=(12, 2 * (len(state_names) // 2))
)
axs = axs.flatten()

for i, (state, name) in enumerate(zip(states, state_names)):
    if name in ["phi", "theta", "psi", "alpha", "beta"]:
        state = np.degrees(state)  # Convert from radians to degrees
    axs[i].plot(state, label=name)
    axs[i].set_xlabel("Time Step")
    axs[i].set_ylabel("State Value")
    axs[i].set_title(f"{name.capitalize()} Evolution Over Time")
    axs[i].legend()

plt.tight_layout()
plt.show()
