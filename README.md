# pyf16

`pyf16` 是一个用于模拟 F-16 飞机动力学的 Python 库。

The aerodynamics included in this model come from the NASA Technical Report 1538, Simulator Study of Stall/Post-Stall Characteristics of a Fighter Airplane with Relaxed Longitudinal Static Stability, by Nguyen, Ogburn, Gilbert, Kibler, Brown, and Deal, Dec 1979. The model is based on Aircraft Control and Simulations, by Brian Stevens and Frank Lewis, Wiley Inter-Science, New York, 1992.

[原始的 MATLAB/Simulink 模型](https://dept.aem.umn.edu/~balas/darpa_sec/SEC.Software.html)


## 安装

确保你已经安装了 Rust, CMake 和 Python 3.10 及以上版本。

### 下载发布的预编译程序

### 构建

#### 使用 [uv](https://docs.astral.sh/uv/) 进行构建

#### 不使用 [uv](https://docs.astral.sh/uv/) 进行构建

1. 克隆仓库：
    ```sh
    git clone https://github.com/WindLX/pyf16.git
    cd pyf16
    ```

2. 使用 `maturin` 构建并安装：
    ```sh
    python -m venv .venv
    source ./.venv/bin/activate
    pip install maturin
    maturin develop
    ```

3. (optional)手动构建 *f16_model*
    ```sh
    rm build
    cd f16_model
    mkdir build
    cd build
    cmake ..
    make install
    ```
    在 *f16_model* 目录下会生成一个 *install* 文件夹， 其中的 *f16_model* 文件夹为后续加载模型所需的目录


## Feature

- 模型配平
- 单步仿真
- 动态加载气动数据


## 使用示例

以下是一个简单的使用示例，展示了如何加载气动模型、安装模型、加载控制限制、进行配平计算并创建飞机对象：

```python
import pyf16

aero_model = pyf16.AerodynamicModel("/path/to/f16_model")
aero_model.install("/path/to/f16_model/data")
control_limits = aero_model.load_ctrl_limits()

trim_target = pyf16.TrimTarget(15000, 500, None, None)
trim_init = None
trim_result = pyf16.trim(aero_model, trim_target, control_limits, trim_init).to_core_init()

f16 = pyf16.PlaneBlock("1", aero_model, trim_result, [0, 0, 0], control_limits)
core_output = f16.update(
    pyf16.Control(thrust=100, elevator=0, aileron=0, rudder=0), 0.1
)
print(core_output.state.to_list())

f16.delete_model()
aero_model.uninstall()
```


## API

见 [stub 文件](pyf16.pyi)


## TODO

- [ ] 更详细的文档
- [ ] 可变的 ODE 求解器


## LICENSE

[许可证](LICENSE)