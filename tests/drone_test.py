from ..fall_uav import Mission, DroneController

controller = DroneController(system='ros')
controller.setup()

mission = Mission(controller)

mission.add_step('arm')
mission.add_step('set_flight_mode', {'mode': 'GUIDED'})
mission.add_step('takeoff')
mission.add_step('move_to', [0.0, 0.0, 2.0, 1.57])
mission.add_step('land ')
mission.add_step('disarm')

mission.execute()
