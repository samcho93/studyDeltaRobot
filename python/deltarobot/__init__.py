"""deltarobot — rotary delta robot kinematics, design analysis and control.

The same package runs in the browser (Pyodide Playground) and on a PC / ROS 2.
"""

from .design import DeltaDesign, catalog
from .kinematics import Unreachable, fk, ik, jacobian, reachable
from .robot import DeltaRobot, EmergencyStop, WorkspaceError
from .scene import Scene, default_scene

__version__ = "0.1.0"
__all__ = ["DeltaDesign", "DeltaRobot", "Scene", "default_scene", "catalog", "fk", "ik", "jacobian",
           "reachable", "Unreachable", "WorkspaceError", "EmergencyStop"]
