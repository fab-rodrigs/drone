from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

def arm_and_takeoff(vehicle, target_altitude):
    while not vehicle.is_armable:
        print("Esperando veículo ficar armável...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == 'GUIDED':
        print("Aguardando mudança para o modo GUIDED...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("Aguardando armar os motores...")
        time.sleep(1)

    vehicle.simple_takeoff(target_altitude)
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Alcançou a altitude alvo")
            break
        time.sleep(1)

def main():
    # Conectar ao drone (utilizando internamente MAVLink)
    vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

    # Chamar a função de decolagem
    arm_and_takeoff(vehicle, 10)

    # Navegar para um waypoint
    point1 = LocationGlobalRelative(-35.363261, 149.165230, 20)
    vehicle.simple_goto(point1)

    # Esperar alguns segundos para permitir que o drone alcance o waypoint
    time.sleep(30)

    # Pousar
    vehicle.mode = VehicleMode("LAND")

    # Fechar a conexão
    vehicle.close()

if __name__ == "__main__":
    main()
