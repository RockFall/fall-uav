from ..mission_control.mission import Mission

def help():
    print("Lista de comandos:")
    print("reset - Volta ao inicio desta missao")
    print("start /- Inicia a missão (Takeoff -> Waypoints -> Landing)")
    print("takeoff /- Decola o drone")
    print("land /- Pousa o drone")
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
            mission.execute()
            print("Mission completed")

        # -------- RESET COMMAND ------------
        elif cmd == 'reset':
            go_back_steps = [
                {"action": "takeoff", "params": {"delay": 5.0}},
                {"action": "move_to", "params": {"waypoints": [[0.0, 0.0, 2.0, 1.57]]}},
                {"action": "land"}
            ]
            m = Mission(go_back_steps, file=False, system=SYSTEM_TYPE)
            m.execute()
            print("De volta ao inicio!")

        # -------- TAKEOFF COMMAND ------------
        elif cmd == 'takeoff':
            takeoff_step = [
                {"action": "takeoff"}
            ]
            m = Mission(takeoff_step, file=False, system=SYSTEM_TYPE)
            m.execute()

        # -------- LAND COMMAND ------------
        elif cmd == 'land':
            land_step = [
                {"action": "land"}
            ]
            m = Mission(land_step, file=False, system=SYSTEM_TYPE)
            m.execute()

        elif cmd == 'exit':
            break
        elif cmd == 'help':
            help()
        elif cmd == 'escanea':
            mission.scan()
        else:
            print("!! Comando inválido !!")