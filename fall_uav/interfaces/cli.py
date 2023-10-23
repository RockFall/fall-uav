from ..mission_control import Mission

def help():
    print("Lista de comandos:")
    print("reset - Volta ao inicio desta missao")
    print("start /- Inicia a missão (Takeoff -> Waypoints -> Landing)")
    print("takeoff /- Decola o drone")
    print("land /- Pousa o drone")
    #print("stop /- Para o drone")
    #print("continue /- Continua a missão atual")
    #print("setwaypoint x y z w /- Adiciona um novo waypoint")
    #print("listwaypoints /- Lista os waypoints")
    #print("clearwaypoints /- Limpa a lista de waypoints")
    print("exit /- Sai do programa")
    print("help /- Mostra essa mensagem novamente")
    print("----------------------------")

if __name__ == '__main__':
    SYSTEM_TYPE = 'uav_manager'
    mission = Mission("fase01.json", system=SYSTEM_TYPE)

    # Help me making a better cli interface
    print("--- Iniciando drone... ----")
    help()
    while True:
        cmd = input("c> ")

        # -------- START COMMAND ------------
        if cmd == 'start':
            mission.start_mission()
            print("Mission completed")

        # -------- RESET COMMAND ------------
        elif cmd == 'reset':
            go_back_steps = [
                {"action": "takeoff", "params": {"delay": 5.0}},
                {"action": "move_to", "params": {"waypoints": [[0.0, 0.0, 2.0, 1.57]]}},
                {"action": "land"}
            ]
            m = Mission(go_back_steps, file=False, system=SYSTEM_TYPE)
            m.start_mission()
            print("De volta ao inicio!")

        # -------- TAKEOFF COMMAND ------------
        elif cmd == 'takeoff':
            takeoff_step = [
                {"action": "takeoff"}
            ]
            m = Mission(takeoff_step, file=False, system=SYSTEM_TYPE)
            m.start_mission()

        # -------- LAND COMMAND ------------
        elif cmd == 'land':
            land_step = [
                {"action": "land"}
            ]
            m = Mission(land_step, file=False, system=SYSTEM_TYPE)
            m.start_mission()

        elif cmd == 'exit':
            break
        elif cmd == 'help':
            help()
        elif cmd == 'escanea':
            mission.scan()
        else:
            print("!! Comando inválido !!")