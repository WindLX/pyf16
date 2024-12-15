import pyf16
import logging
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
    core_output = f16.update(
        pyf16.Control(thrust=2109, elevator=0, aileron=0, rudder=5), 0.01 * i
    )
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

for i, (state, name) in enumerate(zip(states, state_names)):
    plt.figure(figsize=(12, 8))
    plt.plot(state, label=name)
    plt.xlabel("Time Step")
    plt.ylabel("State Value")
    plt.title(f"{name.capitalize()} Evolution Over Time")
    plt.legend()
    plt.show()
