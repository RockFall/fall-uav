# Classe Mission: Uma missao é conjunto de etapas que o drone deve seguir,
# por exemplo, takeoff() -> seguir um conjunto de waypoints -> land()
# e depois takeoff() de novo -> seguir outro conjunto de waypoints -> land()

from fall_uav.controllers.drone_controller import DroneController
from enum import Enum
import json

class MissionStep(Enum):
    TAKEOFF = "takeoff"
    LAND = "land"
    MOVE_TO = "move_to"
    CENTRALIZE = "centralize"
    SCAN_QR = "scan_qr"
    SLOW_DESCENT = "slow_descent"
    ARM = "arm"
    SET_FLIGHT_MODE = "set_flight_mode"
    DISARM = "disarm"


class Mission:
    def __init__(self, steps_json, controller=None, file=True):
        # Get the steps from a json structure
        self.steps = steps_json
        if file:
            with open(steps_json, 'r') as file:
                self.steps = json.load(file)

        # Starting the DroneController
        if controller is None:
            self.drone_controller = DroneController(system='ros', log=[])
        else:
            self.drone_controller = controller
    
    def execute(self):

        self.drone_controller.setup()

        for step in self.steps:
            action = step.get('action')
            params = step.get('params', {})

            # ========= ARM ==========
            if action == MissionStep.ARM.value:
                self.drone_controller.arm()
                if 'delay' not in params:
                    self.drone_controller.sleep(2.0)

            # ========= DISARM ==========
            elif action == MissionStep.DISARM.value:
                self.drone_controller.disarm()
                if 'delay' not in params:
                    self.drone_controller.sleep(2.0)

            # ========= SET_FLIGHT_MODE ==========
            elif action == MissionStep.SET_FLIGHT_MODE.value:
                mode = params.get('mode', 'GUIDED')
                self.drone_controller.set_flight_mode(mode)

            # ======== TAKEOFF =========
            if action == MissionStep.TAKEOFF.value:
                self.drone_controller.takeoff()
                if 'delay' not in params:
                    self.drone_controller.sleep(5.0)

            # ========== LAND ===========
            elif action == MissionStep.LAND.value:
                self.drone_controller.land()
                if 'delay' not in params:
                    self.drone_controller.sleep(10.0)

            # ========= MOVE_TO ==========
            elif action == MissionStep.MOVE_TO.value:
                print("...Iniciado movimento")

                waypoints = params['waypoints']
                self.drone_controller.move_to_waypoints(waypoints)

            # ======== CENTRALIZE =========
            elif action == MissionStep.CENTRALIZE.value:
                self.drone_controller.centralize()

            # ========== SCAN_QR ===========
            elif action == MissionStep.SCAN_QR.value:
                self.drone_controller.scan_QR()

            # ========= SLOW_DESCEND ==========
            elif action == MissionStep.SLOW_DESCENT.value:
                min_height = params.get('min_height', 1.0)  # defaulting to 1.0 if not provided
                self.drone_controller.slow_descent(min_height)

            if 'delay' in params:
                    self.drone_controller.sleep(params['delay'])


    def add_step(self, action, params={}):
        self.steps.append({"action": action, "params": params})

    def print_steps(self):
        print("Mission steps:")
        # Pretty print of steps
        idx=1
        for step in self.steps:
            print(f"{idx} - {step}")
            idx += 1

