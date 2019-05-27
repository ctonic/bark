# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os
from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.ml.runtime_rl import RuntimeRL
from modules.runtime.ml.nn_state_observer import StateConcatenation
from modules.runtime.ml.action_wrapper import MotionPrimitives
from modules.runtime.viewer.pygame_viewer import PygameViewer
import numpy as np


class RuntimeRLTests(unittest.TestCase):
    def test_motion_primitives_concat_state(self):
        scenario_generation = UniformVehicleDistribution(num_scenarios=1, random_seed=0)
        state_observer = StateConcatenation()
        action_wrapper = MotionPrimitives()

        runtimerl = RuntimeRL(action_wrapper=action_wrapper, nn_observer=state_observer,
                        reward_observer=None, step_time=0.2, viewer=PygameViewer,
                        scenario_generator=scenario_generation)

        runtimerl.reset()

        next_state, reward, done, success = runtimerl.step(np.array([1,2,3,4,4]))
        
        
if __name__ == '__main__':
    unittest.main()