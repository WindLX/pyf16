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
    print(core_output.state.to_list())

f16.delete_model()
aero_model.uninstall()
