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
import math
import time


def get_options(args=None):
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


def needtime(vehicle_id, charging_station_dict, destination, battery_full_amount):
    a = 0
    if vehicle.getRoadID(vehicle_id) == lane.getEdgeID(
            chargingstation.getLaneID(charging_station_dict["charging_station_id"])):
        a = math.inf
    else:
        a += simulation.findRoute(vehicle.getRoadID(vehicle_id), lane.getEdgeID(
            chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime
        a += battery_full_amount - int(vehicle.getParameter(vehicle_id, "actualBatteryCapacity")) + simulation.findRoute(vehicle.getRoadID(vehicle_id), lane.getEdgeID(
            chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime
        a += simulation.findRoute(
            lane.getEdgeID(chargingstation.getLaneID(
                charging_station_dict["charging_station_id"])),
            destination).travelTime
    return a  # 중간에 250은 나중에 설정할 배터리 풀충전량

#scheduling(waiting_car_array)
#array[0] : 오는 시간

def scheduling(a):  # array를 overlap 없이 배열하는 함수 -> waiting queue에 있는 자동차들의 대기시간을 앞 순서 자동차들의 이동시간과 충전시간에 맞춰서 조절하는 함수.
  for i in range(1, len(a[0])): #결국 len(a[0])는 기다리고 있는 자동차들의 현재 위치와 충전소까지의 거리 사이의 최소 시간들을 모아놓은 array
    if a[0, i] < a[1, i-1]: # i는 1부터 시작, a[0,1] = 0번째 행의 1번째 열, 만약 a[0,1]가 a[1,0]보다 작으면 : 1번째 자동차의 오는데에 걸리는 시간이 0번째 자동차의 오는시간+충전시간보다 작으면, : 0번째 순서의 자동차가 도착하고 충전을 끝마치기 전에 1번째가 도착하면!
      a[1, i] += 2*(a[1, i-1]-a[0, i]) # 1번째 자동차의 오는시간+충전시간에 (0번째 자동차의 오는 시간+충전시간에서 1번째 자동차의 오는 시간을 뺀 값의 2배)를 더한다. 
      a[0, i] += a[1, i-1]-a[0, i] # 1번째 자동차의 오는시간에 (0번째 자동차의 오는시간+충전시간에서 1번째 자동차의 오는 시간을 뺀 값)을 더한다. 


def require_time(a, b, way_to_destination):
  if len(a[0]) != 0:
    time = sum(a[1])
    arrive_type = -1
    for i in range(0, len(a[0])):
      if a[2, i] <= b[0]:
        arrive_type = i
    if arrive_type < 0:
      if a[0, 0] < b[1]:
        a[1, 0] += 2*(b[1]-a[0, 0])
        a[0, 0] += b[1]-a[0, 0]
        scheduling(a)
      return sum(a[1])-time+b[1]+way_to_destination
    elif arrive_type == len(a[0])-1:
      if b[0] < a[1, -1]:
        b[1] += 2*(a[1, -1]-b[0])
      return b[1]+way_to_destination
    else:
        if b[0] < a[1, arrive_type]:
            b[1] += 2*(a[1, arrive_type]-b[0])
            b[0] += a[1, arrive_type]-b[0]
        if a[0, arrive_type+1] < b[1]:
            a[1, arrive_type+1] += 2*(b[1]-a[0, arrive_type+1])
            a[0, arrive_type+1] += b[1]-a[0, arrive_type+1]
        scheduling(a)
        return sum(a[1])-time+b[1]+way_to_destination
  else:
    return b[1]+way_to_destination


def set_destination_info(edge_list, destination_number):
    destination_info = random.sample(edge_list, destination_number)
    return destination_info  # return list


def change_destination_info(destination_info):
    a = destination_info[0]
    destination_info = np.delete(destination_info, (0))
    destination_info = np.append(destination_info, a)
    return destination_info
# 걍 0번째 값, 맨 뒤로 순서 바꾸는 듯


def search_qcm(charging_staion_list, vehicle_id):
    a = 1
    while a == 1:
        random_qcm = random.choice(charging_staion_list)
        # 충전소 리스트에서 랜덤으로 하나 골라서 , 충전소 아이디를 통해 레인 아이디를 얻고, 레인 아이디를 통해서 엣지 아이디를 얻음. 그리고 현재 차량이 있는 엣지에
        if lane.getEdgeID(chargingstation.getLaneID(random_qcm["charging_station_id"])) != vehicle.getRoadID(vehicle_id):
            a = 0
    return random_qcm


if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
np.random.seed(0)

# contains TraCI control loop


def run(vehiclenum, destination_number, banbok, start_time):
    #veh num : 200, destination num : 5, start_time : time.time() : real-time
    step = 0
    car_index = 0
    battery_full_amount = 1500
    charging_station_list = []
    edge_list = []

    for charging_station in chargingstation.getIDList():
        # TODO : who set the number of charging station?
        # Initialize the charging_station list
        # Each element of list is the charging station dictionary containing charging station id and waiting vehicle list (which is numpy array)
        temp_charging_station_dict = {}
        temp_charging_station_dict = {
            "charging_station_id": charging_station,
            # different with normal array, np.array's elements should be same datatype
            "waiting_vehicle": np.array([])
        }
        charging_station_list.append(temp_charging_station_dict)

    for temp_edge in edge.getIDList():
        for charging_station_dict in charging_station_list:
            # 엣지중 사용 가능한 엣지를 뽑기 위하여 다음과 같이 설정
            # 충전소가 있는 엣지는 목적지 엣지에서 제외함
            # print(temp_edge)
            if temp_edge[0] != ":" and temp_edge != lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])):
                edge_list.append(temp_edge)
                break
            else:
                break

    # 전기자동차 15대 생성
    destination_info = np.empty((0, destination_number))
    # dnum개의 0값을 가진 요소를 갖는 array vnum개를 만들어냄
    time_info = np.zeros((vehiclenum, destination_number))
    charge_info = np.zeros((vehiclenum, 2))  # [0,0]을 vehiclenum개 만들어냄
    finish_info = np.empty((0, vehiclenum))
    for i in range(0, vehiclenum):
        vehicle.add(vehID="vehicle_" + str(car_index), routeID="route_" +str(random.randint(0, 287)), typeID='ElectricCar')
        destination_info = np.append(destination_info, np.array([set_destination_info(edge_list, destination_number)]), axis=0)
        finish_info = np.append(finish_info, 0)
        car_index += 1
    #print(edge_list)
    while simulation.getMinExpectedNumber() > 0:
        # getMinExpectedNumber() -> integer
        # Returns the number of all active vehicles and persons which are in the net plus the ones still waiting to start.
        traci.simulationStep()

        for vehicle_id in vehicle.getIDList():
            # 초기화 코드
            # vehicle state가 running이 아니면,
            if vehicle.getParameter(vehicle_id, "state") == "":
                vehicle.setParameter(
                    vehicle_id, "actualBatteryCapacity", random.randint(200, 1200))
                vehicle.setParameter(vehicle_id, "state", "running")

                vehicle.setParameter(vehicle_id, "destination", destination_info[int(vehicle_id.split(
                    '_')[1])][0])  # vehicle_1 <- 1 extract, -> vehicle 1은 1 destination info를 선택
                # vehicle의 destination edge가 2번째 parameter로 설정됨
                vehicle.changeTarget(
                    vehicle_id, vehicle.getParameter(vehicle_id, "destination"))
                vehicle.setParameter(vehicle_id, "next_charging_station", "")

            else:
                # vehicle 상태가 Running이면
                # 시간이 지남에 따라 배터리 양을 1씩 감소
                if vehicle.getParameter(vehicle_id, "state") == "running":
                    battery_amount = float(vehicle.getParameter(
                        vehicle_id, "actualBatteryCapacity"))  # 현재 값을 가져와서
                    battery_amount -= 1  # 1 줄이고
                    vehicle.setParameter(
                        vehicle_id, "actualBatteryCapacity", battery_amount)  # 줄인 값을 다시 할당

                    # 배터리의 양이 Threshold일때
                    if vehicle.getParameter(vehicle_id, "next_charging_station") == "" and float(vehicle.getParameter(vehicle_id, "actualBatteryCapacity")) < simulation.findRoute(vehicle.getRoadID(vehicle_id), vehicle.getParameter(vehicle_id, "destination")).travelTime+25:
                        #findRoute : 현재 edge와 최종 도착지 edge 사이에 fastest road를 찾아냄
                        #findRoute : stage라는 class 변수를 반환 & 해당 class 내에 traveltime이라는 변수 있음
                        min_expect_time = 100000000  # 이거 무한으로 바꿔놔도 상관없긴 함.
                        # 현재 같은 edge에 있지 않은 충전소 반환?
                        min_expect_qcm = search_qcm(
                            charging_station_list, vehicle_id)
                        for charging_station_dict in charging_station_list:
                            if vehicle.getRoadID(vehicle_id) != lane.getEdgeID(
                                chargingstation.getLaneID(charging_station_dict["charging_station_id"])) and (float(vehicle.getParameter(vehicle_id, "actualBatteryCapacity")) > simulation.findRoute(vehicle.getRoadID(vehicle_id),
                                                                                                                                                                                                      lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime):
                                #충전소가 위치한 엣지와 차량이 위치한 엣지가 다르고 && 충전소의 엣지와 차량의 엣지 사이의 최소 거리의 운행 시간보다 현재 차량의 배터리로 갈 수 있는 거리가 크면(현재 배터리로 갈 수 있는 충전소이면), 
                                #print(simulation.findRoute(vehicle.getRoadID(vehicle_id),
                                #lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime)
                                # anyway, 크기 3짜리 빈 배열 하나 만듦
                                waiting_car_array = np.empty((0, 3))
                                for waiting in charging_station_dict["waiting_vehicle"]:
                                    information = np.array([[simulation.findRoute(vehicle.getRoadID(waiting), lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime, (battery_full_amount - float(vehicle.getParameter(waiting, "actualBatteryCapacity")))/12 + (13/12)*simulation.findRoute(
                                        vehicle.getRoadID(waiting), lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime, simulation.findRoute(vehicle.getRoadID(waiting), lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime]])
                                #복잡하긴 한데 저 위의 information은 예약된 차들의 오는시간, 오는시간+충전시간, 오는시간의 array임.
                                # 1. 기다리는 차들의 현재 위치와 충전소 위치의 최소 거리의 travel time
                                # 2-1. 기다리는 차의 풀충 배터리 양 - 남은 배터리 양 = 비어있는 배터리 양 을 12로 왜 나누지? -> 어쨌든 이게 충전시간이겠지
                                # 2-2. 기다리는 차의 위치와 충전소의 위치 사이에 걸리는 시간에 13/12를 곱해. 왜 곱하지? -> 어쨌든 이게 오는 시간이겠지
                                # 3. 기다리는 차들의 현재 위치와 충전소 위치의 최소 거리의 travel time
                                    # 기다리는 차 배열에 위의 정보를 append.
                                    waiting_car_array = np.append(waiting_car_array, information, axis=0)
                                # transpose : information : [0,0,0] ~~ [[a,a,a],[b,b,b],[c,c,c],[d,d,d], ..., [n,n,n]] <- waiting_car_array, n is the number of waiting car in specific charging station. , 3,n,1 <- 1,n,3 -> 모든 차의 오는 시간 [a,b,c,d,...,n]이 0번째 요소인 배열
                                waiting_car_array = waiting_car_array.T
                                # 오는 시간, 오름차순으로 정렬
                                waiting_car_array = waiting_car_array[:, waiting_car_array[0].argsort()]

                                scheduling(waiting_car_array)

                                #print(len(waiting_car_array[0])) #얘 길이가 0이면?
                                reservating_vehicle_info = np.array([simulation.findRoute(vehicle.getRoadID(vehicle_id),
                                                                                          lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime, (battery_full_amount - float(vehicle.getParameter(vehicle_id, "actualBatteryCapacity")))/12 + (13/12)*simulation.findRoute(
                                    vehicle.getRoadID(vehicle_id), lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"]))).travelTime])
                                way_to_destination = simulation.findRoute(lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])), vehicle.getParameter(vehicle_id, "destination")).travelTime
                                # 충전소에서 종착지까지 걸리는 시간

                                if min_expect_time >= require_time(waiting_car_array, reservating_vehicle_info, way_to_destination):
                                    min_expect_time = require_time(waiting_car_array, reservating_vehicle_info, way_to_destination)

                                    min_expect_qcm = charging_station_dict
                                    #print(lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])))
                                    #print(min_expect_qcm, min_expect_time)
                        vehicle.changeTarget(vehicle_id,lane.getEdgeID(chargingstation.getLaneID(min_expect_qcm["charging_station_id"])))

                        vehicle.setChargingStationStop(vehicle_id, min_expect_qcm["charging_station_id"])

                        vehicle.setParameter(vehicle_id, "next_charging_station",min_expect_qcm["charging_station_id"])
                        min_expect_qcm["waiting_vehicle"] = np.append(min_expect_qcm["waiting_vehicle"], vehicle_id)
                        chargingstation.setParameter(min_expect_qcm["charging_station_id"],"waiting_vehicle",min_expect_qcm["waiting_vehicle"])

                    if vehicle.getParameter(vehicle_id, "next_charging_station") != "" and vehicle.getRoadID(vehicle_id) != lane.getEdgeID(chargingstation.getLaneID(vehicle.getParameter(vehicle_id, "next_charging_station"))):
                        charge_info[int(vehicle_id.split('_')[1])][1] = step
                        #충전하러 가고 있고, 아직 도착은 안했음, vehicle_1의 1

                    # 충전소에 도착했을 때 상태를 Charging으로 수정
                    if vehicle.getStopState(vehicle_id) == 65:  # 65가 충전소에서 Stop을 의미

                        vehicle.setParameter(vehicle_id, "state", "charging")
                    # 자동차가 목적지에 도착했을 때 새로운 목적지 생성
                    if vehicle.getRoadID(vehicle_id) == vehicle.getParameter(vehicle_id, "destination"):
                        if vehicle.getParameter(vehicle_id, "next_charging_station") == "":
                            if finish_info[int(vehicle_id.split('_')[1])] < destination_number:
                                time_info[int(vehicle_id.split('_')[1])][int(finish_info[int(vehicle_id.split('_')[1])])] = step
                                # vehicle 1이 특정 번째 destination에 도착했을 때의 step을 기록하는 듯. (step은 clock같은 느낌)
                                finish_info[int(vehicle_id.split('_')[1])] += 1
                                # vehicle 1이 destination에 도착한 횟수를 기록
                                
                            destination_info_i = destination_info[int(vehicle_id.split('_')[1])]
                            destination_info[int(vehicle_id.split('_')[1])] = change_destination_info(destination_info_i)
                            vehicle.setParameter(vehicle_id, "destination", destination_info[int(vehicle_id.split('_')[1])][0]) # 방금 도착한 것 그 다음 인덱스 destination으로 지정
                            vehicle.changeTarget(vehicle_id, vehicle.getParameter(vehicle_id, "destination"))

                # vehicle 상태가 Charging이면
                # 시간이 지남에 따라 배터리 양을 12씩 증가
                elif vehicle.getParameter(vehicle_id, "state") == "charging":
                    battery_amount = float(vehicle.getParameter(vehicle_id, "actualBatteryCapacity"))
                    battery_amount += 4
                    vehicle.setParameter(vehicle_id, "actualBatteryCapacity", battery_amount)

                    # 배터리 양이 최대치로 충전 됐을때
                    # 충전중인 자동차들 다시 목적지로 출발
                    if battery_amount >= battery_full_amount:
                        vehicle.resume(vehicle_id)
                        vehicle.changeTarget(vehicle_id, vehicle.getParameter(vehicle_id, "destination"))
                        vehicle.setParameter(vehicle_id, "state", "running")
                        
                        for charging_station_dict in charging_station_list:
                            if charging_station_dict["charging_station_id"] == vehicle.getParameter(vehicle_id, "next_charging_station"):
                                charging_station_dict_wanted = charging_station_dict
                                break
                            
                        charging_station_dict_wanted["waiting_vehicle"] = charging_station_dict_wanted["waiting_vehicle"][charging_station_dict_wanted["waiting_vehicle"] != vehicle_id] #조건에 맞는 요소만 return하나 봄
                        chargingstation.setParameter(charging_station_dict_wanted["charging_station_id"], "waiting_vehicle", charging_station_dict_wanted["waiting_vehicle"]) #충전하고 떠난 vehicle을 제외한 waiting list로 update
                        vehicle.setParameter(vehicle_id, "next_charging_station", "")
                        charge_info[int(vehicle_id.split('_')[1])][0] = charge_info[int(vehicle_id.split('_')[1])][0]+step-charge_info[int(vehicle_id.split('_')[1])][1] #뭐 .. 뭔 info인지 모르겟네 ..
                        
        if sum(finish_info) >= 0.9 * destination_number*vehiclenum:
            #finish_info는 모든 vehicle들이 destination 까지 주행끝낸 경우마다 값을 추가한 정보를 갖고 있음
            np.save('vn_{}_{}_{}_traveltime_info'.format(vehiclenum, destination_number, banbok), time_info)
            np.save('vn_{}_{}_{}_charge_info_{}'.format(vehiclenum, destination_number, banbok, step), charge_info)
            traci.close()
            sys.stdout.flush()

        #모든 vehicle에 대해서 step하나 진행한 것임. 
        step += 1

        if step % 10 == 0: 
            # 10step에 한번씩, 현재 상태 print
            perc = sum(finish_info)/(destination_number*vehiclenum)*100
            namt = (time.time() - start_time)*(100/(perc+0.0001)-1)
            print('현재 차량', vehiclenum, '대', banbok+1, '번째', round(perc, 1), '% 진행중이며 시작한지', int(round(time.time() - start_time, 0)), '초 경과되었습니다.')
            
    
    
    # 주행 중인 vehicle 더 이상 없으면 끝
    traci.close()
    sys.stdout.flush()


if __name__ == "__main__":

    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    for num in [200]:
        #걍 200이라는 값을 ..  ? ? ? ? ? ? 왜 for 문에 ?!?!?!?!
        for num2 in [5]:
            for n in range(30, 65, 5):
                #range(start,stop,step)
                #demo 30 to 60 by 5
                try:
                    traci.start([sumoBinary, "-c", "demo_"+str(n) +
                                ".sumocfg", "--tripinfo-output", "tripinfo_1.xml"])
                    """
                    https://sumo.dlr.de/docs/Simulation/Output/TripInfo.html
                    the contents of the list will be executed as the command
                    --tripinfo-output <FILE_NAME> 
                    The simulation is forced to generate this output using 
                    the option --tripinfo-output <FILE> on the command line/within a configuration.
                    This output contains the information about each vehicle's departure time, 
                    the time the vehicle wanted to start at (which may be lower than the real departure time) 
                    and the time the vehicle has arrived. 
                    The information is generated for each vehicle as soon as the vehicle arrived at its destination 
                    and is removed from the network.
                    """
                    start = time.time()
                    run(num, num2, n, start)
                except Exception:
                    pass
