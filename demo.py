import os
import sys
import optparse
import traci
from traci import simulation
from traci import vehicle
from traci import chargingstation
from traci import lane
from traci import edge
from sumolib import checkBinary
import random

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))

def get_options(args=None):
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

# contains TraCI control loop
def run():
    step = 0
    car_index = 0
    battery_full_amount = 250
    charging_station_list = []
    edge_list = []

    for charging_station in chargingstation.getIDList():
        temp_charging_station_dict = {}
        temp_charging_station_dict = {
            "charging_station_id": charging_station,
            "waiting_vehicle_num": 0
        }
        charging_station_list.append(temp_charging_station_dict)
    for temp_edge in edge.getIDList():
        for charging_station_dict in charging_station_list:
            # 엣지중 사용 가능한 엣지를 뽑기 위하여 다음과 같이 설정
            # 충전소가 있는 엣지는 목적지 엣지에서 제외
            if temp_edge[0] != ":" and temp_edge != lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])):
                edge_list.append(temp_edge)
                break
            else:
                break

    # 전기자동차 5대 생성
    for i in range(0, 100):
        vehicle.add(vehID="vehicle_" + str(car_index), routeID="route_0", typeID='ElectricCar')
        car_index += 1

    print(edge_list)
    while simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        print(step)

        for vehicle_id in vehicle.getIDList():
            # 초기화 코드
            if vehicle.getParameter(vehicle_id, "state") == "":
                vehicle.setParameter(vehicle_id, "actualBatteryCapacity", battery_full_amount)
                vehicle.setParameter(vehicle_id, "state", "running")
                vehicle.setParameter(vehicle_id, "destination", "-gneE186")
                vehicle.setParameter(vehicle_id, "next_charging_station", "")
            else:
                # vehicle 상태가 Running이면
                # 시간이 지남에 따라 배터리 양을 1씩 감소
                if vehicle.getParameter(vehicle_id, "state") == "running":
                    battery_amount = int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity"))
                    battery_amount -= 1
                    vehicle.setParameter(vehicle_id, "actualBatteryCapacity", battery_amount)

                    # 배터리의 양이 Threshold일때
                    if int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity")) == 200:
                        while True:
                            charging_station_dict = random.choice(charging_station_list)
                            # 충전을 위하여 자신이 있는 도로에 있는 충전소에 충전시 급 브레이크로 인한 문제 생겨 다음과 같이 프로그래밍 함
                            if vehicle.getRoadID(vehicle_id) != lane.getEdgeID(
                                    chargingstation.getLaneID(charging_station_dict["charging_station_id"])):
                                vehicle.changeTarget(vehicle_id,
                                                    lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])))
                                route = simulation.findRoute(vehicle.getRoadID(vehicle_id), lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])))

                                print(route)
                                vehicle.setChargingStationStop(vehicle_id, charging_station_dict["charging_station_id"])
                                print("charging_station: ", charging_station_dict["charging_station_id"])
                                print(vehicle.getNextStops(vehicle_id))
                                vehicle.setParameter(vehicle_id, "next_charging_station", charging_station_dict["charging_station_id"])
                                charging_station_dict["waiting_vehicle_num"] += 1
                                chargingstation.setParameter(charging_station_dict["charging_station_id"], "waiting_vehicle_num", charging_station_dict["waiting_vehicle_num"])
                                print("waiting_vehicle_count: ", chargingstation.getParameter(charging_station_dict["charging_station_id"], "waiting_vehicle_num"))
                                break

                    # 충전소에 도착했을 때 상태를 Charging으로 수정
                    if vehicle.getStopState(vehicle_id) == 65:  # 65가 충전소에서 Stop을 의미
                        vehicle.setParameter(vehicle_id, "state", "charging")

                    # 자동차가 목적지에 도착했을 때 새로운 목적지 생성
                    if vehicle.getRoadID(vehicle_id) == vehicle.getParameter(vehicle_id, "destination"):
                        if vehicle.getParameter(vehicle_id, "next_charging_station") == "":
                            destination = random.choice(edge_list)
                            vehicle.changeTarget(vehicle_id, destination)
                            vehicle.setParameter(vehicle_id, "destination", destination)

                # vehicle 상태가 Charging이면
                # 시간이 지남에 따라 배터리 양을 1씩 증가
                elif vehicle.getParameter(vehicle_id, "state") == "charging":
                    battery_amount = int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity"))
                    battery_amount += 1
                    vehicle.setParameter(vehicle_id, "actualBatteryCapacity", battery_amount)

                    # 배터리 양이 최대치로 충전 됐을때
                    # 충전중인 자동차들 다시 목적지로 출발
                    if battery_amount == battery_full_amount:
                        vehicle.resume(vehicle_id)
                        vehicle.changeTarget(vehicle_id, vehicle.getParameter(vehicle_id, "destination"))
                        vehicle.setParameter(vehicle_id, "state", "running")
                        vehicle.setParameter(vehicle_id, "next_charging_station", "")
                        charging_station_dict["waiting_vehicle_num"] -= 1
                        chargingstation.setParameter(charging_station_dict["charging_station_id"], "waiting_vehicle_num",
                                                     charging_station_dict["waiting_vehicle_num"])
        step += 1

    traci.close()
    sys.stdout.flush()


if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    traci.start([sumoBinary, "-c", "demo.sumocfg", "--tripinfo-output", "tripinfo.xml"])

    run()
