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

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))

np.random.seed(0)

def get_options(args=None):
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
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
            lane.getEdgeID(chargingstation.getLaneID(charging_station_dict["charging_station_id"])),
            destination).travelTime
    return a #중간에 250은 나중에 설정할 배터리 풀충전량

def scheduling(a): #array를 overlap 없이 배열하는 함수
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

def set_destination_info(edge_list,destination_number):
    destination_info=random.sample(edge_list, destination_number)
    return destination_info

def change_destination_info(destination_info):
    a=destination_info[0]
    destination_info=np.delete(destination_info, (0))
    destination_info=np.append(destination_info, a)
    return destination_info

def search_qcm(charging_staion_list, vehicle_id):
    a=1
    while a==1:
        random_qcm=random.choice(charging_staion_list)
        if lane.getEdgeID(chargingstation.getLaneID(random_qcm["charging_station_id"])) != vehicle.getRoadID(vehicle_id):
            a=0
    return random_qcm


# contains TraCI control loop
def run(vehiclenum,destination_number,banbok,start_time, velocity):
    step = 0
    car_index = 0
    battery_full_amount = 1500
    charging_station_list = []
    edge_list = []
    for charging_station in chargingstation.getIDList():
        temp_charging_station_dict = {}
        temp_charging_station_dict = {
            "charging_station_id": charging_station,
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
    destination_info=np.empty((0,destination_number))
    time_info=np.zeros((vehiclenum,destination_number))
    charge_info=np.zeros((vehiclenum,2))
    finish_info=np.empty((0,vehiclenum))
    for i in range(0, vehiclenum):
        vehicle.add(vehID="vehicle_" + str(car_index), routeID="route_"+str(random.randint(0,287)), typeID='ElectricCar')
        destination_info=np.append(destination_info, np.array([set_destination_info(edge_list,destination_number)]), axis=0)
        finish_info=np.append(finish_info, 0)
        car_index += 1
    #print(edge_list)
    while simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        for vehicle_id in vehicle.getIDList():
            # 초기화 코드
            if vehicle.getParameter(vehicle_id, "state") == "":
                vehicle.setParameter(vehicle_id, "actualBatteryCapacity", random.randint(200,1200))
                vehicle.setParameter(vehicle_id, "state", "running")

                vehicle.setParameter(vehicle_id, "destination", destination_info[int(vehicle_id.split('_')[1])][0])
                vehicle.changeTarget(vehicle_id, vehicle.getParameter(vehicle_id, "destination"))
                vehicle.setParameter(vehicle_id, "next_charging_station", "")

            else:
                # vehicle 상태가 Running이면
                # 시간이 지남에 따라 배터리 양을 1씩 감소
                if vehicle.getParameter(vehicle_id, "state") == "running":
                    battery_amount = float(vehicle.getParameter(vehicle_id, "actualBatteryCapacity"))
                    battery_amount -= 1
                    vehicle.setParameter(vehicle_id, "actualBatteryCapacity", battery_amount)
                    # 배터리의 양이 Threshold일때
                    if vehicle.getParameter(vehicle_id, "next_charging_station") == "" and float(vehicle.getParameter(vehicle_id, "actualBatteryCapacity")) < simulation.findRoute(vehicle.getRoadID(vehicle_id),vehicle.getParameter(vehicle_id, "destination")).travelTime+25:
                        min_expect_qcm = search_qcm(charging_station_list, vehicle_id)
                        for charging_station_dict in random.sample(charging_station_list, 5):
                            if vehicle.getRoadID(vehicle_id) != lane.getEdgeID(
                                    chargingstation.getLaneID(charging_station_dict["charging_station_id"])) and (float(
                                vehicle.getParameter(vehicle_id, "actualBatteryCapacity")) > simulation.findRoute(
                                vehicle.getRoadID(vehicle_id),
                                lane.getEdgeID(chargingstation.getLaneID(
                                    charging_station_dict["charging_station_id"]))).travelTime):
                                min_expect_qcm=charging_station_dict
                                break



                        vehicle.changeTarget(vehicle_id,
                                             lane.getEdgeID(chargingstation.getLaneID(
                                                 min_expect_qcm["charging_station_id"])))
                        vehicle.setChargingStationStop(vehicle_id, min_expect_qcm["charging_station_id"])

                        vehicle.setParameter(vehicle_id, "next_charging_station",
                                             min_expect_qcm["charging_station_id"])
                    if vehicle.getParameter(vehicle_id, "next_charging_station") != "" and vehicle.getRoadID(vehicle_id) != lane.getEdgeID(
                                                    chargingstation.getLaneID(vehicle.getParameter(vehicle_id, "next_charging_station"))):
                        charge_info[int(vehicle_id.split('_')[1])][1] = step

                    # 충전소에 도착했을 때 상태를 Cahrging으로 수정
                    if vehicle.getStopState(vehicle_id) == 65:  # 65가 충전소에서 Stop을 의미
                        vehicle.setParameter(vehicle_id, "state", "charging")
                    # 자동차가 목적지에 도착했을 때 새로운 목적지 생성
                    if vehicle.getRoadID(vehicle_id) == vehicle.getParameter(vehicle_id, "destination"):

                        if vehicle.getParameter(vehicle_id, "next_charging_station") == "":

                            if finish_info[int(vehicle_id.split('_')[1])]<destination_number:

                                time_info[int(vehicle_id.split('_')[1])][int(finish_info[int(vehicle_id.split('_')[1])])]=step
                                finish_info[int(vehicle_id.split('_')[1])]+=1
                            destination_info_i=destination_info[int(vehicle_id.split('_')[1])]
                            destination_info[int(vehicle_id.split('_')[1])]=change_destination_info(destination_info_i)
                            vehicle.setParameter(vehicle_id, "destination",destination_info[int(vehicle_id.split('_')[1])][0])
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
                            if charging_station_dict["charging_station_id"]==vehicle.getParameter(vehicle_id, "next_charging_station"):
                                charging_station_dict_wanted=charging_station_dict
                                break
                        #charging_station_dict_wanted["waiting_vehicle"] = np.delete(charging_station_dict_wanted["waiting_vehicle"], vehicle_id)
                        chargingstation.setParameter(charging_station_dict_wanted["charging_station_id"], "waiting_vehicle",
                                                     charging_station_dict_wanted["waiting_vehicle"])
                        vehicle.setParameter(vehicle_id, "next_charging_station", "")
                        charge_info[int(vehicle_id.split('_')[1])][0]=charge_info[int(vehicle_id.split('_')[1])][0]+step-charge_info[int(vehicle_id.split('_')[1])][1]

        if sum(finish_info) >= 0.9 * destination_number*vehiclenum:
            np.save('r_vc_{}_{}_{}_traveltime_info'.format(velocity,destination_number,banbok),time_info)
            np.save('r_vc_{}_{}_{}_charge_info_{}'.format(velocity,destination_number,banbok,step), charge_info)
            traci.close()
            sys.stdout.flush()
        step += 1
        if step % 10 == 0:
            perc = sum(finish_info)/(destination_number*vehiclenum)*100
            namt = int(round((time.time() - start_time)*(90-perc)/(perc+0.0001),4))
            if namt > 1000:
                nam = '계산중'
            else:
                nam = namt
            print('현재 속도',velocity, banbok+1,'번째',round(perc, 1), '% 진행중이며 시작한지',int(round(time.time() - start_time,0)),'초 경과되었으며 예상 종료 시간은',nam,'초 입니다.')

    traci.close()
    sys.stdout.flush()


if __name__ == "__main__":
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    for velocity in range(40,65,5):
        for i in range(3):
            try:
                traci.start([sumoBinary, "-c", "demo_"+str(velocity)+".sumocfg", "--tripinfo-output", "tripinfo_1.xml"])
                start = time.time()
                run(200,5,i,start, velocity)
            except Exception:
                pass