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
import numpy as np

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))

def get_options(args=None):
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

def needtime(vehicle_id, charging_station_dict, destination, battery_full_amount):
    a = 0
    if vehicle.getRoadID(vehicle_id) == lane.getEdgeID(
            chargingstation.getLaneID(charging_station_dict["charging_station_id"])):
        a = 10000
    else:
        a += simulation.findRoute(vehicle.getRoadID(vehicle_id), lane.getEdgeID(
            chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime
        a += battery_full_amount - int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity")) + simulation.findRoute(vehicle.getRoadID(vehicle_id), lane.getEdgeID(
            chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime
        a += simulation.findRoute(
            lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])),
            destination).travelTime
    return a #중간에 250은 나중에 설정할 배터리 풀충전량

def scheduling(a): #array를 overlap 없이 배열하는 함수
  a=a[:,a[0].argsort()]
  for i in range(1,len(a[0])):
    if a[0,i]<a[1,i-1]:
      a[1,i]+=2*(a[1,i-1]-a[0,i])
      a[0,i]+=a[1,i-1]-a[0,i]

def require_time(a,b,way_to_destination):
  if len(a[0])!=0:
    time=sum(a[0])
    arrive_type=-1
    for i in range(0,len(a[0])):
      if a[2,i]<=b[0]:
        arrive_type=i
    if arrive_type<0:
      if a[0,0]<b[1]:
        a[1,0]+=2*(b[1]-a[0,0])
        a[0,0]+=b[1]-a[0,0]
        scheduling(a)
      return sum(a[0])-time+b[1]+way_to_destination
    elif arrive_type==len(a[0])-1:
      if b[0]<a[1,-1]:
        b[1]+=2*(a[1,-1]-b[0])
      return b[1]+way_to_destination
    else:
        if a[0,arrive_type+1]<b[1]:
            a[1,arrive_type+1]+=2*(b[1]-a[0,arrive_type+1])
            a[0,arrive_type+1]+=b[1]-a[0,arrive_type+1]
        scheduling(a)
        return sum(a[0])-time+b[1]+way_to_destination
  else:
    return b[1]+way_to_destination

def set_destination(edge_list, vehicle_id):
    a=1
    while a==1:
        edge=random.choice(edge_list)
        if edge!=vehicle.getRoadID(vehicle_id):
            a=0
    vehicle.setParameter(vehicle_id, "destination", edge)

def search_qcm(charging_staion_list, vehicle_id):
    a=1
    while a==1:
        random_qcm=random.choice(charging_staion_list)
        if lane.getEdgeID(chargingstation.getLaneID(random_qcm["charging_station_id"])) != vehicle.getRoadID(vehicle_id):
            a=0
    return random_qcm


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
            "waiting_vehicle_info": np.empty((0,3)),
            "waiting_vehicle": np.array([])
        }
        charging_station_list.append(temp_charging_station_dict)
    for temp_edge in edge.getIDList():
        for charging_station_dict in charging_station_list:
            # 엣지중 사용 가능한 엣지를 뽑기 위하여 다음과 같이 설정
            # 충전소가 있는 엣지는 목적지 엣지에서 제외함
            if temp_edge[0] != ":" and temp_edge != lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])):
                edge_list.append(temp_edge)
                break
            else:
                break

    # 전기자동차 15대 생성
    for i in range(0, 100):
        vehicle.add(vehID="vehicle_" + str(car_index), routeID="route_"+str(random.randint(0,100)), typeID='ElectricCar')
        car_index += 1

    #print(edge_list)
    while simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        for vehicle_id in vehicle.getIDList():
            # 초기화 코드
            if vehicle.getParameter(vehicle_id, "state") == "":
                vehicle.setParameter(vehicle_id, "actualBatteryCapacity", battery_full_amount)
                vehicle.setParameter(vehicle_id, "state", "running")
                set_destination(edge_list, vehicle_id)
                print(vehicle.getParameter(vehicle_id, "destination"),
                      type(vehicle.getParameter(vehicle_id, "destination")))
                vehicle.changeTarget(vehicle_id, vehicle.getParameter(vehicle_id, "destination"))
                vehicle.setParameter(vehicle_id, "next_charging_station", "")
            else:
                # vehicle 상태가 Running이면
                # 시간이 지남에 따라 배터리 양을 1씩 감소
                if vehicle.getParameter(vehicle_id, "state") == "running":
                    battery_amount = int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity"))
                    battery_amount -= 1
                    vehicle.setParameter(vehicle_id, "actualBatteryCapacity", battery_amount)

                    # 배터리의 양이 Threshold일때
                    if vehicle.getParameter(vehicle_id, "next_charging_station") == "" and int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity"))<simulation.findRoute(vehicle.getRoadID(vehicle_id), vehicle.getParameter(vehicle_id, "destination")).travelTime+25:
                        print(1)
                        min_expect_time=10000 #이거 무한으로 바꿔놔도 상관없긴 함.
                        min_expect_qcm=search_qcm(charging_station_list, vehicle_id)
                        qcm_distance_list=[]
                        for charging_station_dict in charging_station_list:
                            qcm_distance_list.append([charging_station_dict, vehicle.getDrivingDistance(vehicle_id, lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])), 32, laneIndex=1)]) #여기만요 **********************************8
                        qcm_distance_list.sort(key=lambda x:x[1])
                        reservating_vehicle_info_need=[]

                        for lists in qcm_distance_list[0:5]:
                            dict=lists[0]
                            if (vehicle.getRoadID(vehicle_id) in lane.getEdgeID(chargingstation.getLaneID(dict["charging_station_id"])))==False: #and (int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity")) > simulation.findRoute(vehicle.getRoadID(vehicle_id),
                                                                            #lane.getEdgeID(chargingstation.getLaneID(dict["charging_station_id"]))).travelTime): #이건 테스트용
                                #print(simulation.findRoute(vehicle.getRoadID(vehicle_id),
                                                                            #lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime)
                                '''waiting_car_array=np.empty((0,3)) #여기부터 필요 없음
                                for waiting in charging_station_dict["waiting_vehicle"]:
                                    information=np.array([[simulation.findRoute(vehicle.getRoadID(waiting),
                                                                            lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime,
                                                        battery_full_amount - int(vehicle.getParameter(waiting,
                                                                                                        "actualBatteryCapacity")) + 2*simulation.findRoute(
                                                            vehicle.getRoadID(waiting), lane.getEdgeID(
                                                                chargingstation.getLaneID(charging_station_dict[
                                                                                                "charging_station_id"]))).travelTime, simulation.findRoute(vehicle.getRoadID(waiting),
                                                                            lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime]])
                                #복잡하긴 한데 저 위의 information은 예약된 차들의 오는시간, 오는시간+충전시간, 오는시간의 array임.
                                    waiting_car_array=np.append(waiting_car_array, information, axis=0)
                                waiting_car_array=waiting_car_array.T
                                scheduling(waiting_car_array)
                                #print(len(waiting_car_array[0])) #얘 길이가 0이면?'''
                                waiting_car_array=dict['waiting_vehicle_info']
                                waiting_car_array=waiting_car_array.T
                                scheduling(waiting_car_array)
                                reservating_vehicle_info=np.array([simulation.findRoute(vehicle.getRoadID(vehicle_id),
                                                                            lane.getEdgeID(chargingstation.getLaneID(dict["charging_station_id"]))).travelTime,battery_full_amount - int(vehicle.getParameter(vehicle_id,
                                                                                                        "actualBatteryCapacity")) + 2*simulation.findRoute(
                                                            vehicle.getRoadID(vehicle_id), lane.getEdgeID(
                                                                chargingstation.getLaneID(dict[
                                                                                                "charging_station_id"]))).travelTime,simulation.findRoute(vehicle.getRoadID(vehicle_id),
                                                                            lane.getEdgeID(chargingstation.getLaneID(dict["charging_station_id"]))).travelTime])
                                way_to_destination=simulation.findRoute(
                                    lane.getEdgeID(chargingstation.getLaneID(dict["charging_station_id"])), vehicle.getParameter(vehicle_id, "destination")).travelTime
                                if min_expect_time >= require_time(waiting_car_array, reservating_vehicle_info,
                                                                   way_to_destination):
                                    min_expect_time = require_time(waiting_car_array, reservating_vehicle_info,
                                                                   way_to_destination)
                                    min_expect_qcm = dict
                                    reservating_vehicle_info_need=reservating_vehicle_info

                                     #print(lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])))
                                        #print(min_expect_qcm, min_expect_time)
                        vehicle.changeTarget(vehicle_id,
                                             lane.getEdgeID(chargingstation.getLaneID(
                                                 min_expect_qcm["charging_station_id"])))

                        vehicle.setChargingStationStop(vehicle_id, min_expect_qcm["charging_station_id"])

                        vehicle.setParameter(vehicle_id, "next_charging_station",
                                             min_expect_qcm["charging_station_id"])

                        min_expect_qcm["waiting_vehicle_info"]=np.append(min_expect_qcm["waiting_vehicle_info"], [reservating_vehicle_info_need],axis=0)
                        min_expect_qcm["waiting_vehicle"]=np.append(min_expect_qcm["waiting_vehicle"],vehicle_id)
                        chargingstation.setParameter(min_expect_qcm["charging_station_id"],
                                                     "waiting_vehicle",
                                                     min_expect_qcm["waiting_vehicle"])
                        chargingstation.setParameter(min_expect_qcm["charging_station_id"], "waiting_vehicle_info", min_expect_qcm["waiting_vehicle_info"])

                    # 충전소에 도착했을 때 상태를 Cahrging으로 수정
                    if vehicle.getStopState(vehicle_id) == 65:  # 65가 충전소에서 Stop을 의미
                        vehicle.setParameter(vehicle_id, "state", "charging")
                        print(2)
                    # 자동차가 목적지에 도착했을 때 새로운 목적지 생성
                    if vehicle.getRoadID(vehicle_id) == vehicle.getParameter(vehicle_id, "destination"):
                        if vehicle.getParameter(vehicle_id, "next_charging_station") == "":
                            set_destination(edge_list, vehicle_id)
                            vehicle.changeTarget(vehicle_id, vehicle.getParameter(vehicle_id, "destination"))

                # vehicle 상태가 Charging이면
                # 시간이 지남에 따라 배터리 양을 1씩 증가
                elif vehicle.getParameter(vehicle_id, "state") == "charging":
                    battery_amount = int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity"))
                    battery_amount += 1
                    vehicle.setParameter(vehicle_id, "actualBatteryCapacity", battery_amount)

                    # 배터리 양이 최대치로 충전 됐을때
                    # 충전중인 자동차들 다시 목적지로 출발
                    if battery_amount >= battery_full_amount:
                        vehicle.resume(vehicle_id)
                        vehicle.changeTarget(vehicle_id, vehicle.getParameter(vehicle_id, "destination"))
                        vehicle.setParameter(vehicle_id, "state", "running")
                        for charging_station_dict in charging_station_list:
                            if charging_station_dict["charging_station_id"]==vehicle.getParameter(vehicle_id, "next_charging_station"):
                                charging_station_dict_wanted=charging_station_dict
                                break
                        id=np.where(charging_station_dict_wanted["waiting_vehicle"]==vehicle_id)
                        charging_station_dict_wanted["waiting_vehicle_info"]=np.delete(charging_station_dict_wanted["waiting_vehicle_info"],id[0][0],axis=0)
                        charging_station_dict_wanted["waiting_vehicle"] = charging_station_dict_wanted["waiting_vehicle"][charging_station_dict_wanted["waiting_vehicle"]!=vehicle_id]
                        chargingstation.setParameter(charging_station_dict_wanted["charging_station_id"], "waiting_vehicle",
                                                     charging_station_dict_wanted["waiting_vehicle"])
                        chargingstation.setParameter(charging_station_dict_wanted["charging_station_id"],"waiting_vehicle_info", charging_station_dict_wanted["waiting_vehicle"])
                        vehicle.setParameter(vehicle_id, "next_charging_station", "")

        for charging_station_dict in charging_station_list:
            if len(charging_station_dict["waiting_vehicle_info"])!=0:
                charging_station_dict["waiting_vehicle_info"]=charging_station_dict["waiting_vehicle_info"]-1
                chargingstation.setParameter(charging_station_dict["charging_station_id"], "waiting_vehicle_info", charging_station_dict["waiting_vehicle_info"])
            if len(charging_station_dict["waiting_vehicle"])>=3:
                print(len(charging_station_dict["waiting_vehicle"]))
        step += 1

    traci.close()
    sys.stdout.flush()


if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    traci.start([sumoBinary, "-c", "demo.sumocfg", "--tripinfo-output", "tripinfo_1.xml"])

    run()
