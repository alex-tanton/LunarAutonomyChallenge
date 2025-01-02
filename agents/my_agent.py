#!/usr/bin/env python


# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


"""
TODO: Add a description of the agent.
"""

### Import used libraries. ###
import carla
from carla import SensorPosition, VehicleVelocityControl
import cv2 as cv
import numpy as np

# TODO Add any other used libraries.

### Import the AutonomousAgent from the Leaderboard. ###
from Leaderboard.leaderboard.autoagents.autonomous_agent import AutonomousAgent


def get_entry_point() -> str:
    """
    This function returns the name of the agent class. It will be used by the
    Leaderboard to instantiate an agent at the beginning of mission execution.

    Returns:
        str: The name of the agent class.
    """

    return "my_agent"


class my_agent(AutonomousAgent):

    def setup(self, path_to_conf_file: str) -> None:
        """
        Setup function to initialize the agent.

        All procedures necessary to initialize the control agent are executed
        here. This includes the initialization of the agent's sensors and other
        variables, as well as the loading of any pre-trained models.

        It is recommended to call the get_initial_position() method at some early
        point in the mission to begin the mapping process with the correct
        initial position of the robot.

        Args:
            path_to_conf_file (str): Path to the configuration file. This is an
            optional argument and can be ignored if not needed.
        """

        # TODO: Add any setup code needed for the agent.

        return

    def use_fiducials(self) -> bool:
        """
        Defines whether the agent uses fiducial markers for navigation. Extra
        points are scored for not using fiducials.

        Returns:
            bool: True if fiducials are used, False otherwise.
        """

        return True

    def sensors(self) -> dict[SensorPosition, dict[str, any]]:
        """
        Sets the initial sensor configuration of the agent. By default, all
        sensors are disabled.

        Each sensor position is initialized with a dictionary containing keys
        for the state of the camera, the light, and also the width and height in
        pixels of the camera.

        You can activate and deactivate the cameras and lights later in the
        simulation using the relevant setter methods of the API.

        The use_semantic argument is optional, to activate the  semantic
        counterpart of each camera, it is False by default if not set. The
        maximum camera resolution permitted is 2448 x 2048 pixels, if a higher
        resolution is requested the resolution will be clipped to the maximum and
        a warning  will be given on the command line.

        Returns:
            dict: A dictionary of sensors to be used, with keys corresponding to
            sensor positions and values containing sensor configuration.
        """

        sensors = {
            SensorPosition.Front: {
                "camera_active": True,
                "light_intensity": 1.0,
                "width": "2448",
                "height": "2048",
                "use_semantic": False,
            },
            SensorPosition.FrontLeft: {
                "camera_active": True,
                "light_intensity": 1.0,
                "width": "2448",
                "height": "2048",
                "use_semantic": False,
            },
            SensorPosition.FrontRight: {
                "camera_active": True,
                "light_intensity": 1.0,
                "width": "2448",
                "height": "2048",
                "use_semantic": False,
            },
            SensorPosition.Left: {
                "camera_active": False,
                "light_intensity": 0,
                "width": "2448",
                "height": "2048",
                "use_semantic": False,
            },
            SensorPosition.Right: {
                "camera_active": False,
                "light_intensity": 0,
                "width": "2448",
                "height": "2048",
                "use_semantic": False,
            },
            SensorPosition.BackLeft: {
                "camera_active": False,
                "light_intensity": 0,
                "width": "2448",
                "height": "2048",
                "use_semantic": False,
            },
            SensorPosition.BackRight: {
                "camera_active": False,
                "light_intensity": 0,
                "width": "2448",
                "height": "2048",
                "use_semantic": False,
            },
            SensorPosition.Back: {
                "camera_active": False,
                "light_intensity": 0,
                "width": "2448",
                "height": "2048",
                "use_semantic": False,
            },
        }

        return sensors

    def run_step(self, input_data) -> VehicleVelocityControl:
        """
        Implements the logic for a simulation time-step.

        The simulator runs in discrete time steps at 20Hz in simulation time.
        This method is executed with each simulation time-step by the Leaderboard.
        It processes sensor data and returns a control instruction to the
        simulator to move or modify the robot's state. Cameras run at 10Hz, so
        camera data is delivered every other time-step.

        Parameters:
        input_data (dict): A dictionary containing sensor data from the previous
        simulation time-step. The dictionary has two keys:
            - 'Grayscale': Access the standard, grayscale camera data.
            - 'Semantic': Access the semantic segmentation data.
        Each of these keys contains another dictionary with keys corresponding
        to the sensor positions:
            - carla.SensorPosition.Front
            - carla.SensorPosition.FrontLeft
            - carla.SensorPosition.FrontRight
            - carla.SensorPosition.Left
            - carla.SensorPosition.Right
            - carla.SensorPosition.BackLeft
            - carla.SensorPosition.BackRight
            - carla.SensorPosition.Back

        Returns:
        carla.VehicleVelocityControl: The control input to the simulator, even
        if the control decision hasn't been updated during the time-step.
        Returning None or any other type will cause an error.
        """

        # Return the control input to the simulator
        return VehicleVelocityControl(0.0, 0.0)
