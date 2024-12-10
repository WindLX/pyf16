from enum import Enum
from typing import List

from pyf16._core import *


SolverType = Enum("SolverType", "RK1 RK2 RK3 RK4")


class PlaneBlock:
    def __init__(
        self,
        solver: SolverType,
        step: float,
        model: AerodynamicModel,
        init: CoreInit,
        deflection: List[float],
        ctrl_limit: ControlLimit,
    ) -> None:
        core = self._get_core_class(solver)
        self._core = core(step, model, init, deflection, ctrl_limit)

    @staticmethod
    def _get_core_class(solver: SolverType) -> type:
        if solver == SolverType.RK1:
            return PlaneBlockRK1
        elif solver == SolverType.RK2:
            return PlaneBlockRK2
        elif solver == SolverType.RK3:
            return PlaneBlockRK3
        elif solver == SolverType.RK4:
            return PlaneBlockRK4

    def update(self, control: Control, t: float) -> CoreOutput:
        return self._core.update(control, t)

    def reset(self, init: CoreInit) -> None:
        self._core.reset(init)

    @property
    def state(self) -> CoreOutput:
        return self._core.state

    def delete_model(self) -> None:
        self._core.delete_model()
