from FallWaypoints import FallWaypoints

import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_srvs.srv import Trigger

# Classe Missao: Uma missao é conjunto de etapas que o drone deve seguir,
# por exemplo, takeoff() -> seguir um conjunto de waypoints -> land()
# e depois takeoff() de novo -> seguir outro conjunto de waypoints -> land()

class Missao:
    def __init__(self, steps):
        self.steps = steps
    def start_mission(self, fw):
        for step in self.steps:
            if move:
                waypoints = step
                move = False
                fw.set_waypoints(waypoints)
                fw.start_movement()
                continue
            if step == 'takeoff':
                self.takeoff()
                rospy.sleep(5)
            elif step == 'land':
                self.land()
                rospy.sleep(10)
            elif step == 'move_to':
                move = True
                continue
            


    def takeoff():
        try:
            print("\n----------Tentando decolar----------")
            rospy.wait_for_service('/uav1/uav_manager/takeoff')
            takeoff_service = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
            response = takeoff_service.call()  # Altitude de decolagem em metros
            print('Takeoff response:', response)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Falha na decolagem: %s", str(e))
            return False 
        
    def land():
        try:
            print("\n----------Pousando----------")
            rospy.wait_for_service('/uav1/uav_manager/land')
            landing_service = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
            response = landing_service.call()
            print('Landing response:', response)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Falha no pouso: %s", str(e))
            return False


def help():
    print("Lista de comandos:")
    print("start /- Inicia a missão (Takeoff -> Waypoints -> Landing)")
    print("takeoff /- Decola o drone")
    print("setwaypoint x y z w /- Adiciona um novo waypoint")
    print("listwaypoints /- Lista os waypoints")
    print("clearwaypoints /- Limpa a lista de waypoints")
    print("stop /- Para o drone")
    print("continue /- Continua a missão atual")
    print("land /- Pousa o drone")
    print("exit /- Sai do programa")
    print("help /- Mostra essa mensagem novamente")
    print("----------------------------")

if __name__ == '__main__':
    fw = FallWaypoints(type='goto')

    # Help me making a better cli interface
    print("--- Iniciando drone... ----")
    help()
    while True:
        cmd = input("c> ")
        if cmd == 'start':
            mission = Missao(
                [
                    'takeoff',
                    'move_to', [4.0, 0.0, 2.0, 1.57],
                    'land',
                    'takeoff',
                    'move_to', [1.5, 2.5, 2.0, 1.57],
                    'land',
                    'takeoff',
                    'move_to', [1.5, 5.5, 2.0, 1.57],
                    'land',
                    'takeoff',
                    'move_to', [0.0, 4.0, 2.0, 1.57],
                    'land',
                    'takeoff',
                    'move_to', [-1.5, 5.5, 2.0, 1.57],
                    'land',
                    'takeoff',
                    'move_to', [-1.5, 2.5, 2.0, 1.57],
                    'land',
                    'takeoff',
                    'move_to', [-4.0, 0.0, 2.0, 1.57],
                    'land',
                    'takeoff',
                    'move_to', [0.0, 0.0, 2.0, 1.57],
                    'land'
                    'takeoff'
                ]
            )

            mission.start_mission(fw)

            print("Mission completed")
        elif cmd == 'takeoff':
            Missao.takeoff()
        elif cmd == 'setwaypoint':
            x, y, z, w = cmd.split(' ')[1:]
            fw.append_waypoint((float(x), float(y), float(z), float(w)))
            print("Waypoint adicionado!")
        elif cmd == 'listwaypoints':
            print("Waypoints:")
            print(fw.waypoints)
        elif cmd == 'clearwaypoints':
            fw.waypoints = []
            print("Lista de waypoints limpa!")
        elif cmd == 'land':
            Missao.land()
        elif cmd == 'stop':
            fw.stop_movement()
        elif cmd == 'continue':
            fw.resume_movement()
        elif cmd == 'exit':
            break
        elif cmd == 'help':
            help()
        else:
            print("!! Comando inválido !!")
