from enum import Enum
from typing import List

from pyf16._core import *


SolverType = Enum("SolverType", "RK1 RK2 RK3 RK4")


class SimpleSolver:
    def __init__(self, solver: SolverType, delta_t: float) -> None:
        self._solver = self._get_solver_class(solver)(delta_t)

    def solve(self, dynamics, time: float, state: list, input_: list) -> list:
        return self._solver.solve(dynamics, time, state, input_)

    @property
    def delta_t(self) -> float:
        return self._solver.delta_t

    @staticmethod
    def _get_solver_class(solver: SolverType) -> type:
        if solver == SolverType.RK1:
            return SimpleSolverRK1
        elif solver == SolverType.RK2:
            return SimpleSolverRK2
        elif solver == SolverType.RK3:
            return SimpleSolverRK3
        elif solver == SolverType.RK4:
            return SimpleSolverRK4


class PlaneBlock:
    def __init__(
        self,
        solver: SolverType,
        delta_t: float,
        model: AerodynamicModel,
        init: CoreInit,
        deflection: List[float],
        ctrl_limit: ControlLimit,
    ) -> None:
        core = self._get_core_class(solver)
        self._delta_t = delta_t
        self._core = core(delta_t, model, init, deflection, ctrl_limit)

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

    @property
    def state_dot(self) -> State:
        return self._core.state_dot

    def delete_model(self) -> None:
        self._core.delete_model()

    @property
    def delta_t(self) -> float:
        return self._delta_t
