from FallWaypoints import FallWaypoints

import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_srvs.srv import Trigger

from mavros_msgs.srv import CommandTOL


# Classe Missao: Uma missao é conjunto de etapas que o drone deve seguir,
# por exemplo, takeoff() -> seguir um conjunto de waypoints -> land()
# e depois takeoff() de novo -> seguir outro conjunto de waypoints -> land()

TARGET_HEIGHT_PICTURES = 3.0

class Missao:
    def __init__(self, steps, system='uav_manager'):
        self.steps = steps
        self.system = system
    def start_mission(self, fw):
        if self.system == 'mavros':
            self.arm()
            self.set_flight_mode('GUIDED')
        move = False
        for step in self.steps:
            if move:
                waypoints = step
                move = False
                fw.clear_waypoints()
                fw.set_waypoints(waypoints)
                print("...Iniciado movimento")
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
    
    def arm(self):
        rospy.wait_for_service('/uav1/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Drone armado com sucesso!")
            else:
                rospy.logwarn("Falha ao armar o drone")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de armamento: %s" % e)

    def set_flight_mode(mode):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
            response = set_mode_service(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Modo de voo alterado para '{mode}' com sucesso!")
            else:
                rospy.logwarn("Falha ao alterar o modo de voo")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de alteração de modo de voo: %s" % e)


    def takeoff(self):
        try:
            print("\n----------Tentando decolar----------")

            if self.system == "uav_manager":
                rospy.wait_for_service('/uav1/uav_manager/takeoff')
                takeoff_service = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
                response = takeoff_service.call()  # Altitude de decolagem em metros
            elif self.system == "mavros":
                rospy.wait_for_service('/uav1/mavros/cmd/takeoff')
                takeoff_service = rospy.ServiceProxy('/uav1/mavros/cmd/takeoff', CommandTOL)
                response = takeoff_service(altitude= TARGET_HEIGHT_PICTURES)
            else:
                raise Exception("Sistema de controle de voo não reconhecido (escolha mavros ou uav_control)")
            print('Takeoff response:', response)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Falha na decolagem: %s", str(e))
            return False 
        
    def land(self):
        try:
            print("\n----------Pousando----------")

            if self.system == "uav_manager":
                rospy.wait_for_service('/uav1/uav_manager/land')
                landing_service = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
                response = landing_service.call()
            elif self.system == "mavros":
                rospy.wait_for_service('/uav1/mavros/cmd/land')
                landing_service = rospy.ServiceProxy('/uav1/mavros/cmd/land', CommandTOL)
                response = landing_service(altitude=0.0)
            else:
                raise Exception("Sistema de controle de voo não reconhecido (escolha mavros ou uav_control)")
            print('Landing response:', response)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Falha no pouso: %s", str(e))
            return False


def help():
    print("Lista de comandos:")
    print("go_back_TEMP - Volta ao inicio desta missao")
    print("start /- Inicia a missão (Takeoff -> Waypoints -> Landing)")
    print("takeoff /- Decola o drone")
    print("land /- Pousa o drone")
    print("stop /- Para o drone")
    print("continue /- Continua a missão atual")
    print("setwaypoint x y z w /- Adiciona um novo waypoint")
    print("listwaypoints /- Lista os waypoints")
    print("clearwaypoints /- Limpa a lista de waypoints")
    print("exit /- Sai do programa")
    print("help /- Mostra essa mensagem novamente")
    print("----------------------------")

if __name__ == '__main__':
    fw = FallWaypoints(type='goto')
    mission = Missao(
                [
                    'takeoff',
                    'move_to', [[4.0, 0.0, 2.0, 1.57]],
                    'land',
                    'takeoff',
                    'move_to', [[1.5, 2.5, 2.0, 1.57]],
                    'land',
                    'takeoff',
                    'move_to', [[1.5, 5.5, 2.0, 1.57]],
                    'land',
                    'takeoff',
                    'move_to', [[0.0, 4.0, 2.0, 1.57]],
                    'land',
                    'takeoff',
                    'move_to', [[-1.5, 5.5, 2.0, 1.57]],
                    'land',
                    'takeoff',
                    'move_to', [[-1.5, 2.5, 2.0, 1.57]],
                    'land',
                    'takeoff',
                    'move_to', [[-4.0, 0.0, 2.0, 1.57]],
                    'land',
                    'takeoff',
                    'move_to', [[0.0, 0.0, 2.0, 1.57]],
                    'land'
                    'takeoff'
                ]
            )

    # Help me making a better cli interface
    print("--- Iniciando drone... ----")
    help()
    while True:
        cmd = input("c> ")
        if cmd == 'start':
            mission.start_mission(fw)
            print("Mission completed")
        elif cmd == 'go_back_TEMP':
            m2 = Missao(['move_to', [[0.0, 0.0, 2.0, 1.57]]], 'land')
            m2.start_mission(FallWaypoints(type='goto'))
            print("De volta ao inicio!")
        elif cmd == 'takeoff':
            mission.takeoff()
        elif cmd == 'setwaypoint':
            x, y, z, w = cmd.split(' ')[1:]
            fw.append_waypoint((float(x), float(y), float(z), float(w)))
            print("Waypoint adicionado!")
        elif cmd == 'listwaypoints':
            print("Waypoints:")
            print(fw.waypoints)
        elif cmd == 'clearwaypoints':
            fw.clear_waypoints()
            print("Lista de waypoints limpa!")
        elif cmd == 'land':
            mission.land()
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