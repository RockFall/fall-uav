# Classe Mission: Uma missao é conjunto de etapas que o drone deve seguir,
# por exemplo, takeoff() -> seguir um conjunto de waypoints -> land()
# e depois takeoff() de novo -> seguir outro conjunto de waypoints -> land()

from DroneController import DroneController
#from ScanArea import ScanArea
from enum import Enum
import json

class MissionStep(Enum):
    TAKEOFF = "takeoff"
    LAND = "land"
    MOVE_TO = "move_to"
    CENTRALIZE = "centralize"
    SCAN_QR = "scan_qr"
    SLOW_DESCENT = "slow_descent"


class Mission:
    def __init__(self, steps_json, system='mavros', file=True, log=[]):
        # Get the steps from a json structure
        self.steps = steps_json
        if file:
            with open(steps_json, 'r') as file:
                self.steps = json.load(file)
        
        # Set the system to be used (uav_manager or mavros)
        self.system = system

        # Starting the DroneController
        self.drone_controller = DroneController(system=system, log=log)

        #self.scan = ScanArea()

    
    def start_mission(self):

        self.drone_controller.setup()

        for step in self.steps:
            action = step.get('action')
            params = step.get('params', {})

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