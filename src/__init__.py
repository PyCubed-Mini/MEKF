from .mekf import step, State

try:
    from ulab.numpy import eye as I, array
except ImportError:
    from numpy import eye as I, array
import time

MEKFState = State

default_state = MEKFState(
    array([0., 0., 0., 0.]),  # q = Quaternion attitude vector
    array([0., 0., 0.]),  # β = Gyro bias vector
    I(6)  # P = Covariance matrix
)


class MEKF:

    def __init__(self, initial_state=default_state):
        self._prev = 0
        self._state = initial_state

    def step(self, w, nr_mag, nr_sun, br_mag, br_sun):
        if self._prev == 0:
            self._prev = time.time()
            return
        delta_t = time.time() - self._prev
        self._prev = time.time()
        step(self._state, w, delta_t, nr_mag, nr_sun, br_mag, br_sun)
