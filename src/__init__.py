from .mekf import step, State

try:
    from ulab.numpy import eye as I, array
except ImportError:
    from numpy import eye as I, array

MEKFState = State

default_state = MEKFState(
    array([1., 0., 0., 0.]),  # q = Quaternion attitude vector
    array([0., 0., 0.]),  # β = Gyro bias vector
    I(6)  # P = Covariance matrix
)


class MEKF:

    def __init__(self, initial_state=default_state):
        self._state = initial_state

    def step(self, w, nr_mag, nr_sun, br_mag, br_sun, dt):
        self._state = step(self._state, w, dt, nr_mag, nr_sun, br_mag, br_sun)
    
    @property
    def attitude(self):
        return self._state.q
    
    @property
    def gyro_bias(self):
        return self._state.β
