# pyf16

[English](README.md) | [中文](README_zh.md)

`pyf16` 是一个用于模拟 F-16 飞机动力学的 Python 库。

下面这张图展示了使用本程序仿真的 F-16 在配平点的各个状态随时间的变化曲线：

![state_plot_1](img/state_plot_1.png)

## 安装

本项目提供了气动模型的预编译程序，支持 Windows 和 Linux 系统的 x86 平台。如果您的操作系统符合要求，您可以跳过编译步骤，直接使用预编译程序。若需要其他平台的编译程序，或者希望自行编译，请按照以下步骤操作。

### 安装气动模型

#### 获取预编译程序

附件中提供了 Windows 和 Linux 系统的 x86 平台下的气动模型预编译程序，您也可以前往本项目的 [GitHub 仓库](https://github.com/WindLX/pyf16/releases) 下载相关版本。

运行程序所需的目录结构已经配置好，气动模型、气动数据和相关信息文件已经放置在 `models/f16_model` 目录下。如果您的系统符合要求，则可以跳过编译步骤。

#### 自行编译气动模型

如果您需要编译气动模型，请确保您的系统已经安装了 CMake 和任意 C 语言的编译器，并正确配置了环境变量。

1. 打开Shell（Windows 下建议使用 PowerShell），并定位到 `f16_model` 目录。

2. 执行以下命令：
   ```bash
   cmake -S . -B build
   cmake --build build [--config Release] --target install
   ```

3. 编译完成后，会在 `f16_model` 目录下生成一个 `install` 文件夹。接下来：
   - 在程序的顶层目录创建 `models` 文件夹。
   - 将 `install` 文件夹中的 `f16_model` 目录复制到 `models` 目录下。

### 安装 pyf16

#### 使用 pip 安装

您可以直接使用 PyPI 上的已发布版本，可以直接运行以下命令进行安装：
```bash
pip install pyf16
```

#### 从源码编译安装

如果您没有合适的 Wheel 文件版本，可以从源码进行编译安装。首先，请确保您的系统中已安装 Rust 编译器。

##### 使用 uv

如果您使用 `uv` 进行 Python 项目管理，可以直接运行以下命令：

```bash
uv run examples/example1.py
```

`uv` 会自动创建虚拟环境并安装所需模块，然后执行示例程序。

##### 不使用 uv

如果您不使用 `uv`，请按照以下步骤操作：

1. 创建虚拟环境：
   ```bash
   python -m venv .venv
   ```

2. 激活虚拟环境：
   - Linux/macOS:
     ```bash
     source ./.venv/bin/activate
     ```
   - Windows:
     ```powershell
     .\.venv\Scripts\Activate.ps1
     ```

3. 安装依赖：
   ```bash
   pip install maturin
   ```

4. 编译并安装模块：
   ```bash
   maturin develop
   ```

5. 安装 Python 库依赖：
   ```bash
   pip install matplotlib numpy
   ```

6. 运行示例程序：
   ```bash
   python examples/example3.py
   ```

### 运行示例程序

安装完成后，您可以运行以下命令来验证安装是否成功：

```bash
python examples/example1.py
```

或者，如果您使用 `uv` 管理虚拟环境，可以运行：

```bash
uv run examples/example1.py
```

## 使用示例

### F-16 仿真

以下是一个简单的使用示例，展示了如何加载气动模型、安装模型、加载控制限制、进行配平计算并创建飞机对象：

```python
import pyf16

aero_model = pyf16.AerodynamicModel("/path/to/f16_model")
aero_model.install("/path/to/f16_model/data")
control_limits = aero_model.load_ctrl_limits()

trim_target = pyf16.TrimTarget(15000, 500, None, None)
trim_init = None
trim_result = pyf16.trim(aero_model, trim_target, control_limits, trim_init).to_core_init()

f16 = pyf16.PlaneBlock(
    pyf16.SolverType.RK4,
    0.01,
    aero_model,
    trim_result.to_core_init(),
    [0, 0, 0],
    control_limits,
)
core_output = f16.update(
    pyf16.Control(thrust=100, elevator=0, aileron=0, rudder=0), 0.1
)
print(core_output.state.to_list())

f16.delete_model()
aero_model.uninstall()

```

### SimpleSolver

同时 pyf16 还提供了一个简单的求解器的接口，用以在 Python 中求解常微分方程：

```python
from pyf16 import SolverType, SimpleSolver

# dx/dt = -k * x
def simple_dynamics(time: float, state: List[float], input_: List[float]) -> List[float]:
    k = input_[0]
    dx_dt = [-k * state[0]]
    return dx_dt

initial_state = [10.0]
input_value = [0.1]
time_step = 0.1
simulation_time = 5.0

solver_type = SolverType.RK4
solver = SimpleSolver(solver_type, delta_t=time_step)

state = initial_state
time = 0.0
while time < simulation_time:
    state = solver.solve(simple_dynamics, time, state, input_value)
    time += time_step
    print(f"Time: {time:.1f}, State: {state[0]:.4f}")

```

## API

见 [API](docs/API.md) 文档


## TODO

- [ ] 更详细的文档
- [x] 可变的 ODE 求解器


## LICENSE

[许可证](LICENSE)


## 鸣谢

本项目的部分实现参考了以下网站提供的代码：
- [DARPA SEC Software](https://dept.aem.umn.edu/~balas/darpa_sec/SEC.Software.html)

感谢该项目的贡献。
