#   -*- coding: utf-8 -*-
# @Author  : lcx
# @File    : test_3_1.py


import carla

import multiprocessing
import time
import math
import numpy as np
from planner import planning_utils, path_planning
from controller.controller import Vehicle_control
from planner.global_planning import global_path_planner


def get_traffic_light_state(current_traffic_light):  # get_state()方法只能返回红灯和黄灯状态，没有绿灯状态（默认把绿灯认为正常）
    # 此方法把红绿蓝三种状态都标定出来
    if current_traffic_light is None:
        current_traffic_light_state = "Green"
    else:
        current_traffic_light_state = current_traffic_light.get_state()
    return current_traffic_light_state


def emergence_brake():
    brake_control = carla.VehicleControl()
    brake_control.steer = 0  # 转向控制
    brake_control.throttle = 0  # 油门控制
    brake_control.brake = 1  # 刹车控制
    return brake_control


def get_actor_from_world(ego_vehicle: carla.Vehicle, dis_limitation=50):
    """已验证
    获取当前车辆前方潜在的车辆障碍物
    首先获取在主车辆一定范围内的其他车辆，再通过速度矢量和位置矢量将在主车辆运动方向后方的车辆过滤掉
    param:  ego_vehicle: 主车辆
            dis_limitation: 探测范围
    return:
    """
    carla_world = ego_vehicle.get_world()  # type:carla.World
    ego_vehicle_loc = ego_vehicle.get_location()
    v_list = []  # 储存范围内的车辆
    vehicle_list = carla_world.get_actors().filter("vehicle.*")  # 所有vehicle
    for vehicle in vehicle_list:
        dis = math.sqrt((ego_vehicle_loc.x - vehicle.get_location().x) ** 2 +
                        (ego_vehicle_loc.y - vehicle.get_location().y) ** 2 +
                        (ego_vehicle_loc.z - vehicle.get_location().z) ** 2)
        if dis < dis_limitation and ego_vehicle.id != vehicle.id:
            v1 = np.array([vehicle.get_location().x - ego_vehicle_loc.x,
                           vehicle.get_location().y - ego_vehicle_loc.y,
                           vehicle.get_location().z - ego_vehicle_loc.z])  # 其他车辆到ego_vehicle的矢量
            ego_vehicle_velocity = np.array([ego_vehicle.get_velocity().x, ego_vehicle.get_velocity().y,
                                             ego_vehicle.get_velocity().z])  # ego_vehicle的速度矢量
            """# 如果车辆出现在ego_vehicle的运动前方，则有可能是障碍物
            # 还需要控制可能的障碍物距离参考线的横向距离, 我的想法是将障碍物在参考线上投影，计算投影点和车辆的距离，
            # 如果距离大于阈值则认为不影响ego-vehicle的运动，反之认为是障碍物会影响ego-vehicle的运动
            # 现在简化一下，将横向距离暂时设定为ego-vehicle当前航向方向的垂直距离，即ego完全按照参考线行驶"""
            ego_vehicle_theta = ego_vehicle.get_transform().rotation.yaw * (math.pi / 180)
            n_r = np.array([-math.sin(ego_vehicle_theta), math.cos(ego_vehicle_theta), 0])  # ego法向量
            if -10 < np.dot(v1, n_r) < 12:  # v1在n_r方向上的投影
                if np.dot(v1, ego_vehicle_velocity) > 0:  # 同向
                    v_list.append((vehicle, dis))
                elif np.dot(v1, ego_vehicle_velocity) < 0 and dis < 10:  # 自车后面十米以内的障碍物仍然考虑，超过十米就不再考虑
                    v_list.append((vehicle, -dis))
    v_list.sort(key=lambda tup: tup[1])  # 按dis从小到大排序
    return v_list


def motion_planning(conn):
    while 1:
        # 接收主进程发送的用于局部路径规划的数据，如果没有收到数据子进程会阻塞
        # (obs_xy, (vehicle_loc.x, vehicle_loc.y), (pred_x, pred_y),
        # (vehicle_v.x, vehicle_v.y), (vehicle_a.x, vehicle_a.y),
        # global_frenet_path, match_point_list)
        possible_obs_, vehicle_loc_, pred_loc_, vehicle_v_, vehicle_a_, global_frenet_path_, match_point_list_ = conn.recv()
        start_time = time.time()
        # 1.确定预测点在全局路径上的投影点索引
        match_point_list_, _ = planning_utils.find_match_points(xy_list=[pred_loc_],
                                                                frenet_path_node_list=global_frenet_path_,
                                                                is_first_run=False,
                                                                pre_match_index=match_point_list_[0])
        # 2.根据匹配点的索引在全局路径上采样一定数量的点
        local_frenet_path_ = planning_utils.sampling(match_point_list_[0], global_frenet_path_,
                                                     back_length=10, forward_length=30)
        # 3.对采样点进行平滑，作为后续规划的参考线
        local_frenet_path_opt_ = planning_utils.smooth_reference_line(local_frenet_path_)

        # 计算以车辆当前位置为原点的s_map
        s_map = planning_utils.cal_s_map_fun(local_frenet_path_opt_, origin_xy=vehicle_loc_)
        path_s, path_l = planning_utils.cal_s_l_fun(local_frenet_path_opt_, local_frenet_path_opt_, s_map)
        # 提取障碍物的位置信息
        if len(possible_obs_) != 0 and possible_obs_[0][-1] <= 30:
            obs_xy = []
            for x, y, dis in possible_obs_:
                obs_xy.append((x, y))

            # 计算障碍物的s,l
            obs_s_list, obs_l_list = planning_utils.cal_s_l_fun(obs_xy, local_frenet_path_opt_, s_map)
        else:
            obs_s_list, obs_l_list = [], []
        # 计算规划起点的s, l
        begin_s_list, begin_l_list = planning_utils.cal_s_l_fun([pred_loc_], local_frenet_path_opt_, s_map)
        """从规划起点进行动态规划"""
        # 计算规划起点的l对s的导数和偏导数
        l_list, _, _, _, l_ds_list, _, l_dds_list = \
            planning_utils.cal_s_l_deri_fun(xy_list=[pred_loc_],
                                            V_xy_list=[vehicle_v_],
                                            a_xy_list=[vehicle_a_],
                                            local_path_xy_opt=local_frenet_path_opt_,
                                            origin_xy=pred_loc_)
        # 从起点开始沿着s进行横向和纵向采样，然后动态规划,相邻点之间依据五次多项式进一步采样，间隔一米
        # print("*motion planning time cost:", time.time() - start_time)
        dp_path_s, dp_path_l = path_planning.DP_algorithm(obs_s_list, obs_l_list,
                                                          plan_start_s=begin_s_list[0],
                                                          plan_start_l=l_list[0],
                                                          plan_start_dl=l_ds_list[0],
                                                          plan_start_ddl=l_dds_list[0])
        # print("**dp planning time cost:", time.time() - start_time)
        # 对动态规划得到的路径进行降采样，减少二次规划的计算量，然后二次规划完成后再插值填充恢复
        dp_path_l = dp_path_l[::2]
        dp_path_s = dp_path_s[::2]
        l_min, l_max = path_planning.cal_lmin_lmax(dp_path_s=dp_path_s, dp_path_l=dp_path_l,
                                                   obs_s_list=obs_s_list, obs_l_list=obs_l_list,
                                                   obs_length=5, obs_width=4)  # 这一步的延迟很低，忽略不计

        # 二次规划变量过多会导致计算延迟比较高，需要平衡二者之间的关系
        # print("l_min_max_length", len(l_min))
        """二次规划"""
        qp_path_l, qp_path_dl, qp_path_ddl = path_planning.Quadratic_planning(l_min, l_max,
                                                                              plan_start_l=l_list[0],
                                                                              plan_start_dl=l_ds_list[0],
                                                                              plan_start_ddl=l_dds_list[0])
        # print("**qp planning time cost:", time.time() - start_time)
        path_s = [dp_path_s[0]]
        path_l = [qp_path_l[0]]
        for i in range(1, len(qp_path_l)):
            # 将相邻的路径点取平均值，这样可以减小路径上的抖动，使路径更加平滑
            path_s.append((dp_path_s[i] + dp_path_s[i - 1]) / 2)
            path_l.append((qp_path_l[i] + qp_path_l[i - 1]) / 2)
            # path_s.append(dp_path_s[i])
            # path_l.append(qp_path_l[i])
        path_s.append(dp_path_s[-1])
        path_l.append(qp_path_l[-1])
        current_local_frenet_path_opt = path_planning.frenet_2_x_y_theta_kappa(plan_start_s=begin_s_list[0],
                                                                               plan_start_l=begin_l_list[0],
                                                                               enriched_s_list=path_s,
                                                                               enriched_l_list=path_l,
                                                                               frenet_path_opt=local_frenet_path_opt_,
                                                                               s_map=s_map)
        # 将重新规划得到的路径信息发送给主进程，让控制器进行轨迹跟踪
        # print("******  in sub process  ******")
        conn.send((current_local_frenet_path_opt, match_point_list_, path_s, path_l))
        # print("***motion planning time cost:", time.time() - start_time)


if __name__ == '__main__':
    conn1, conn2 = multiprocessing.Pipe()
    # conn2传递给一个新的进程p1，conn1留在主进程
    # conn2的作用是作为参数传递给了子进程p1的motion_planning函数，用于向p1进程发送数据
    # 在p1进程中，可以使用conn2.send()方法向conn2所代表的连接中发送数据，这些数据可以通过主进程中的conn1.recv()方法进行接收
    # 换句话说，conn1和conn2组成的一对连接，允许不同进程之间进行双向通信。
    # conn1.send()<-->conn2.recv()      conn2.send()<-->conn1.recv()
    p1 = multiprocessing.Process(target=motion_planning, args=(conn2,))
    p1.start()

    client = carla.Client("localhost", 2000)
    client.set_timeout(10)
    # 对象创建好了之后，在对象中添加需要的环境中的地图
    world = client.load_world('Town05')  # type: carla.World
    amap = world.get_map()  # type: carla.Map
    topo = amap.get_topology()
    global_route_plan = global_path_planner(world_map=amap, sampling_resolution=2)  # 实例化全局规划器
    # topology, graph, id_map, road_id_to_edge = global_route_plan.get_topology_and_graph_info()

    All_spawn_points = amap.get_spawn_points()  # 获取所有carla提供的actor产生位置
    """# 定义一个ego-vehicle"""
    model3_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    model3_bp.set_attribute('color', '255,88,0')
    model3_spawn_point = All_spawn_points[259]
    # print(model3_spawn_point)
    # model3_spawn_point.location = model3_spawn_point.location + carla.Location(x=-100, y=0, z=0)
    model3_actor = world.spawn_actor(model3_bp, model3_spawn_point)  # type: carla.Vehicle
    # 定义轮胎特性
    # wheel_f = carla.WheelPhysicsControl()  # type: carla.WheelPhysicsControl
    # 定义车辆特性
    physics_control = carla.VehiclePhysicsControl()  # type: carla.VehiclePhysicsControl
    physics_control.mass = 1412  # 质量kg
    model3_actor.apply_physics_control(physics_control)

    """路径规划"""
    # 1. 规划路径，输出的每个路径点是一个元组形式【(wp, road_option), ...】第一个是元素是carla中的路点，第二个是当前路点规定的一些车辆行为
    pathway = global_route_plan.search_path_way(origin=model3_spawn_point.location,
                                                destination=All_spawn_points[12].location)
    debug = world.debug  # type: carla.DebugHelper

    # 2. 将路径点构成的路径转换为[(x, y, theta, kappa), ...]的形式
    global_frenet_path = planning_utils.waypoint_list_2_target_path(pathway)

    # 3.提取局部路径
    transform = model3_actor.get_transform()
    vehicle_loc = transform.location  # 获取车辆的当前位置
    match_point_list, _ = planning_utils.find_match_points(xy_list=[(vehicle_loc.x, vehicle_loc.y)],
                                                           frenet_path_node_list=global_frenet_path,
                                                           is_first_run=True,  # 寻找车辆起点的匹配点就属于第一次运行，
                                                           pre_match_index=0)  # 没有上一次运行得到的索引，索引自然是全局路径的起点
    local_frenet_path = planning_utils.sampling(match_point_list[0], global_frenet_path)
    local_frenet_path_opt = planning_utils.smooth_reference_line(local_frenet_path)
    # 计算参考线的s, l
    cur_s_map = planning_utils.cal_s_map_fun(local_frenet_path_opt, origin_xy=(vehicle_loc.x, vehicle_loc.y))
    cur_path_s, cur_path_l = planning_utils.cal_s_l_fun(local_frenet_path_opt, local_frenet_path_opt, cur_s_map)

    """整车参数设定"""
    vehicle_para = (1.015, 2.910 - 1.015, 1412, -148970, -82204, 1537)
    controller = "LQR_controller"
    # controller = "MPC_controller"
    Controller = Vehicle_control(ego_vehicle=model3_actor, vehicle_para=vehicle_para,
                                 pathway=local_frenet_path_opt,
                                 controller_type=controller)  # 实例化控制器

    DIS = math.sqrt((pathway[0][0].transform.location.x - pathway[1][0].transform.location.x) ** 2
                    + (pathway[0][0].transform.location.y - pathway[1][0].transform.location.y) ** 2)  # 计算轨迹相邻点之间的距离
    # print("The distance between two adjacent points in route:", DIS)
    direction = []
    speed = []
    target_speed = []
    max_speed = 50  # 初始速度设为50km/h，
    # 设定一个观察者视角
    spectator = world.get_spectator()
    count = 1  # 控制规划器和控制器相对频率
    main_process_start_time = time.time()
    plan_count = 100
    # control_count = 10
    pred_ts = 0.2
    while True:
        """获取交通速度标志,考虑道路速度限制"""
        # 获取车辆位置信息（包括坐标和姿态信息）， get the transformation, a combination of location and rotation
        transform = model3_actor.get_transform()
        # 不断更新观测视角的位置， update the position of spectator
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20), carla.Rotation(pitch=-90)))
        vehicle_loc = transform.location  # 获取车辆的当前位置

        """获取局部路径，局部路径规划的频率是控制的1/100"""
        if count % plan_count == 0:  # 这里表示控制器执行100次规划器执行1次
            cur_time = time.time()
            # print("main_process_cost_time", time.time() - main_process_start_time)
            main_process_start_time = cur_time

            vehicle_loc = model3_actor.get_transform().location
            vehicle_v = model3_actor.get_velocity()
            vehicle_a = model3_actor.get_acceleration()
            # 基于笛卡尔坐标系预测ts秒过后车辆的位置，以预测点作为规划起点
            pred_x, pred_y, pred_fi = planning_utils.predict_block(model3_actor, ts=pred_ts)
            # 基于frenet坐标系预测ts秒过后车辆的位置，以预测点作为规划起点
            # pred_x, pred_y = planning_utils.predict_block_based_on_frenet(vehicle_loc, vehicle_v,
            #                                                               local_frenet_path_opt,
            #                                                               cur_path_s, cur_path_l, ts=0.2)
            """
            没有找到合适的传感器，暂时用车联网的方法,设定合适的感知范围，获取周围环境中的actor，这里我们人为制造actor作为障碍物
            再到后面可以考虑用多传感器数据融合来做动态和静态障碍物的融合感知
            """
            # 提取障碍物的位置信息
            obs_xy = []

            conn1.send((obs_xy, (vehicle_loc.x, vehicle_loc.y), (pred_x, pred_y),
                        (vehicle_v.x, vehicle_v.y), (vehicle_a.x, vehicle_a.y),
                        global_frenet_path, match_point_list))
            if count != plan_count:  # 第一个循环周期，因为有初始阶段规划好的局部路径，第二个周期的规划还未计算完成，一旦执行接收数据，会阻塞主进程
                cur_local_frenet_path_opt, match_point_list, cur_path_s, cur_path_l = conn1.recv()  # 新规划出的轨迹
                local_frenet_path_opt = cur_local_frenet_path_opt
                for point in local_frenet_path_opt:
                    debug.draw_point(carla.Location(point[0], point[1], 2),
                                     size=0.05, color=carla.Color(255, 0, 0), life_time=0)
            # 注意重新实例化控制器的位置，不能放错了
            Controller = Vehicle_control(ego_vehicle=model3_actor, vehicle_para=vehicle_para,
                                         pathway=local_frenet_path_opt,
                                         controller_type=controller)  # 实例化控制器

        """控制部分"""
        control = Controller.run_step(target_speed=max_speed)  # 实例化的时候已经将必要的信息传递给规划器，这里告知目标速度即可
        direction.append(model3_actor.get_transform().rotation.yaw * (math.pi / 180))
        V = model3_actor.get_velocity()  # 利用 carla API to 获取速度矢量， use the API of carla to get the velocity vector
        V_len = 3.6 * math.sqrt(V.x * V.x + V.y * V.y + V.z * V.z)  # transfer m/s to km/h
        speed.append(V_len)
        target_speed.append(max_speed)
        model3_actor.apply_control(control)  # 执行最终控制指令, execute the final control signal

        """debug 部分"""
        # # 将预测点和投影点的位置标出来, mark the predicted point and project point in the simulation world for debug
        debug.draw_point(carla.Location(Controller.Lat_control.x_pre, Controller.Lat_control.y_pre, 2),
                         size=0.05, color=carla.Color(0, 255, 255), life_time=0)
        # debug.draw_point(carla.Location(Controller.Lat_control.x_pro, Controller.Lat_control.y_pro, 2),
        #                  size=0.05, color=carla.Color(100, 0, 0), life_time=0)

        """距离判断，程序终止条件"""
        count += 1
        # print("in main process")
        # 计算当前车辆和终点的距离, calculate the distance between vehicle and destination
        dist = vehicle_loc.distance(pathway[-1][0].transform.location)
        # print("The distance to the destination: ", dist)

        if dist < 2:  # 到达终点后产生制动信号让车辆停止运动
            control = emergence_brake()
            model3_actor.apply_control(control)
            # print("last waypoint reached")
            p1.terminate()
            break

    """可视化速度变化和航向变化"""
    import matplotlib.pyplot as plt

    plt.figure(1)
    plt.plot(direction)
    plt.ylim(bottom=-5, top=5)

    plt.figure(2)
    plt.plot(speed)
    plt.plot(target_speed, color="r")
    plt.ylim(bottom=0, top=max(target_speed) + 10)
    plt.show()
