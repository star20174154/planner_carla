#   -*- coding: utf-8 -*-
# @Author  : lcx
# @File    : global_planning.py

import math
import carla
import networkx as nx
import numpy as np
from enum import Enum
from planner import planning_utils


class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to others.
    这段代码定义了一个名为RoadOption的枚举类，RoadOption表示从一段车道移动到其他车道时可能的拓扑配置
    """
    # RoadOption枚举类的一个成员，其值为-1。它表示没有有效的拓扑配置，因为没有可用的车道或道路段
    # 这些枚举成员提供了不同的选项，用于描述车辆在路网拓扑中的不同行驶情况和转向动作。通过枚举类，可以更方便地表示和处理车辆的行驶决策和路径规划。
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANE_FOLLOW = 4
    CHANGE_LANE_LEFT = 5
    CHANGE_LANE_RIGHT = 6


class global_path_planner(object):
    # 在Python中，object是所有类的基类。当定义一个新的类时，可以指定其继承自object类，这意味着该类将继承object类的一些通用行为和属性。
    # 继承自object类可以为类提供一些默认的方法和属性，例如__init__构造函数和一些特殊方法（如__str__、__repr__等）。此外，通过继承object类，可以让类获得一些常用的基本行为和功能。
    # 在Python 3中，如果没有显式指定基类，所有的类默认都会继承自object类; python2需要显式指定
    # 这是global_path_planner类的构造函数。它接受两个参数：world_map表示Carla地图对象，sampling_resolution表示路径采样的分辨率
    def __init__(self, world_map, sampling_resolution):
        self._map = world_map       # type: carla.Map    # 赋值给类的成员变量 self._map
        self._sampling_resolution = sampling_resolution
        self._topology = None       # 拓扑信息
        self._graph = nx.DiGraph()  # type: nx.DiGraph  # 创建有向图对象 self._graph，使用了nx库 DiGraph()类  Directed Graph
        self._id_map = None         # 存储id映射信息
        self._road_to_edge = None   # 用于存储道路到边缘的映射信息。

        # initiate the planner
        self._build_topology()      # 用于构建地图的拓扑信息
        self._build_graph()         # 用于构建地图的图形表示

    # 公共方法，用于获取拓扑和图形信息
    def get_topology_and_graph_info(self):
        # 返回成员变量的值
        return self._topology, self._graph, self._id_map, self._road_to_edge

    # 私有方法
    def _build_topology(self):
        """
        main: 构建道路拓扑，获取地图中各个拓扑分段之间的路径点，并将其存储在 self._topology 列表中
        the output of carla.Map.get_topology() could look like this: [(w0, w1), (w0, w2), (w1, w3), (w2, w3), (w0, w4)].
        由于carla.Map.get_topology()只能函数获取起点和终点信息构成的边信息，这些信息不能够为全局路径规划提供细节信息，因此需要重新构建拓扑
        新拓扑用字典类型存储每个路段，具有以下结构：
        {
        entry (carla.Waypoint): waypoint of entry point of road segment
        exit (carla.Waypoint): waypoint of exit point of road segment
        path (list of carla.Waypoint):  list of waypoints between entry to exit, separated by the resolution
        }
        return: None
        注：carla自带的_build_topology中还有entryxyz,exitxyz属性，在这里不做使用
        """
        self._topology = []  # 用于存储拓扑结构
        for seg in self._map.get_topology():
            w1 = seg[0]  # type: carla.Waypoint  # 起点
            w2 = seg[1]  # type: carla.Waypoint  # 终点
            new_seg = dict()    # 创建一个新的字典，用于存储当前拓扑分段的信息
            new_seg["entry"] = w1
            new_seg["exit"] = w2
            new_seg["path"] = []    # 字典内 创建一个空列表，用于存储路径点
            # 按照采样分辨率将w1和w2之间的路径点采样出来
            w1_loc = w1.transform.location  # type: carla.Location  #
            # 检查起始点（w1）和结束点（w2）之间的距离是否大于采样分辨率
            if w1_loc.distance(w2.transform.location) > self._sampling_resolution:
                # 如果起始路点和结束路点之间存在其他路点，则根据采样分辨率将中间点全部存储在new_seg["path"]中
                new_waypoint = w1.next(self._sampling_resolution)[0]  # 从起点开始使用给定的采样分辨率获取下一个路点
                # 进入循环，直到下一个路径点和结束点（w2）之间的距离小于等于采样分辨率。
                while new_waypoint.transform.location.distance(w2.transform.location) > self._sampling_resolution:
                    # 结束路点不会记录到new_seg["path"]中
                    new_seg["path"].append(new_waypoint)
                    new_waypoint = new_waypoint.next(self._sampling_resolution)[0]
            else:
                # 如果起始路点和结束路点之间的距离小于或等于采样分辨率，则仍然让new_seg["path"]保持空列表
                # new_seg["path"].append(w1.next(self._sampling_resolution)[0])
                pass
            self._topology.append(new_seg)  # 将当前拓扑分段的信息（new_seg）添加到拓扑列表中（self._topology）

    def _build_graph(self):
        """"
        根据车辆仿真器中的道路拓扑信息构建一个图形结构，用于表示道路之间的连接关系和车辆行驶方向
        构建图，方便可视化和运用图论的知识进行全局路径规划
        self._graph是一个二向图，属性如下：
            Node properties:
                vertex: (x,y,z) position in world map， 在DiGraph类型下数据结构为{id: {'vertex': (x, y, z)}}
            Edge properties:
                entry_vector: 入口点沿切线方向的单位向量（unit vector along tangent at entry point）
                exit_vector: 出口点沿切线方向的单位向量（unit vector along tangent at exit point）
                net_vector:  入口指向出口的方向的单位向量（unit vector of the chord from entry to exit）
                intersection: 布尔类型，是否属于交叉路口boolean indicating if the edge belongs to an  intersection
        self._id_map  # 字典类型，建立节点id和位置的对应{(x, y, z): id}
        self._road_to_edge  # 字典类型，建立road_id,section_id,lane_id 和边的对应关系
        """
        # self._graph = nx.DiGraph()  # it is initializes in the
        self._id_map = dict()  # 字典类型，建立节点id和位置的对应{(x, y, z): id}
        self._road_to_edge = dict()  # 字典类型，建立road_id,section_id,lane_id 和 边的对应关系

        for seg in self._topology:
            entry_waypoint = seg["entry"]  # type: carla.Waypoint   # 起始和结束的路点对象，用于表示拓扑段的入口和出口
            exit_waypoint = seg["exit"]  # type: carla.Waypoint     #
            path = seg["path"]  # 不包含端点， 拓扑段中的路径
            intersection = entry_waypoint.is_intersection   # bool 表示该拓扑段是否为交叉口
            # 入口路点的 道路id, 路段id 和 车道 id
            road_id, section_id, lane_id = entry_waypoint.road_id, entry_waypoint.section_id, entry_waypoint.lane_id
            entry_xyz = entry_waypoint.transform.location
            # 对小数长度进行限制 ?元组类型
            entry_xyz = (np.round(entry_xyz.x, 2), np.round(entry_xyz.y, 2), np.round(entry_xyz.z, 2))
            exit_xyz = exit_waypoint.transform.location
            exit_xyz = (np.round(exit_xyz.x, 2), np.round(exit_xyz.y, 2), np.round(exit_xyz.z, 2))
            for xyz in entry_xyz, exit_xyz:  # ？？？？？？？？？？？
                if xyz not in self._id_map:
                    New_ID = len(self._id_map)
                    self._id_map[xyz] = New_ID
                    # 将新的节点加入graph
                    self._graph.add_node(New_ID, vertex=xyz)

            n1 = self._id_map[entry_xyz]
            n2 = self._id_map[exit_xyz]

            if road_id not in self._road_to_edge:
                self._road_to_edge[road_id] = dict()
            if section_id not in self._road_to_edge[road_id]:
                self._road_to_edge[road_id][section_id] = dict()
            # 会有左右车道和多车道的情况 举例 13: {0: {-1: (34, 46), 1: (47, 31)}}，
            # 即id为13的道路，包含一个section,这个section是双向单车道
            self._road_to_edge[road_id][section_id][lane_id] = (n1, n2)

            entry_forward_vector = entry_waypoint.transform.rotation.get_forward_vector()  # 这里是入口节点的方向信息
            exit_forward_vector = exit_waypoint.transform.rotation.get_forward_vector()  # 这里是出口节点的方向信息，用于车辆规划路径时的转向

            # 将新的边加入graph
            self._graph.add_edge(u_of_edge=n1, v_of_edge=n2,
                                 length=len(path) + 1, path=path,
                                 entry_waypoint=entry_waypoint, exit_waypoint=exit_waypoint,
                                 entry_vector=entry_forward_vector, exit_vector=exit_forward_vector,
                                 net_vector=planning_utils.Vector_fun(entry_waypoint.transform.location,
                                                                      exit_waypoint.transform.location),
                                 intersection=intersection, type=RoadOption.LANE_FOLLOW)

    # ？？？这TMD也不是私有方法啊 __
    def _find_location_edge(self, loc: carla.Location):
        """
        用于查找给定位置所属的路段，并返回其所属的edge
        param:  loc: 给定的一个位置
        return: 返回graph的一条边(n1, n2)
        """
        nearest_wp = self._map.get_waypoint(loc)  # type: carla.Waypoint    # 获取最近的路径点
        # 现在面临一个问题，对于两个路段相接处的节点，定位的是前一个路段还是后一个路段,在路径规划中二者本质上没有区别，但是自己没有搞明白这个方法的原理
        # 测试的结果是在交叉路口或者弯道情况下，返回的是后一个路段； 在直线道路中返回的是前一个路段
        edge = None
        try:
            # 用最近的路点所在的road_id,section_id和lane_id来定位其所在的边
            edge = self._road_to_edge[nearest_wp.road_id][nearest_wp.section_id][nearest_wp.lane_id]
        except KeyError:
            pass
        return edge

    def _route_search(self, origin, destination):
        """
        使用A*确定从起点到终点的最优距离
        param:  origin: carla.Location 类型
                destination:
        return: list类型，成员是图中节点id
        """
        start_edge = self._find_location_edge(origin)  # 获取起点所在边
        end_edge = self._find_location_edge(destination)  # 获取终点所在边
        route = self._A_star(start_edge[0], end_edge[0])
        if route is None:  # 如果不可达就报错
            raise nx.NetworkXNoPath(f"Node {start_edge[0]} not reachable from {end_edge[0]}")
        route.append(end_edge[1])  # 可达的话就将终点所在边的右端点加入路径
        return route

    def _A_star(self, n_begin, n_end):
        """
        采用A*算法计算两点之间的最短路径
        param:  n_begin: 起点所在边的左端点id
                n_end:  终点所在边的左端点id
        return: 路径list， 每个元素是图中节点id
        """
        route = []
        open_set = dict()  # 字典， 记录每个节点的父节点和最短路径
        closed_set = dict()
        open_set[n_begin] = (0, -1)  # 每个节点对应一个元组，第一个元素是节点到起点的最短路径，第二个元素是父节点的id

        def cal_heuristic(n):
            # hypot返回原点到一点的多维欧几里得距离
            return math.hypot(self._graph.nodes[n]['vertex'][0] - self._graph.nodes[n_end]['vertex'][0],
                              self._graph.nodes[n]['vertex'][1] - self._graph.nodes[n_end]['vertex'][1],
                              self._graph.nodes[n]['vertex'][2] - self._graph.nodes[n_end]['vertex'][2])

        while 1:
            if len(open_set) == 0:  # 终点不可达
                return None
            # find the node with minimum distance between n_begin in open_set
            c_node = min(open_set, key=lambda n: open_set[n][0] + cal_heuristic(n))
            # print(c_node)
            if c_node == n_end:
                closed_set[c_node] = open_set[c_node]
                del open_set[c_node]  # 如果当前节点是终点，则把该节点从open_set中移除，加入到close_set.
                break
            for suc in self._graph.successors(c_node):  # 处理当前所有节点的后继
                new_cost = self._graph.get_edge_data(c_node, suc)["length"]     # 当前节点到后继节点的cost
                if suc in closed_set:  # 如果访问过就不再访问
                    continue
                elif suc in open_set:  # 如果在即将访问的集合中，判断是否需要更新路径
                    if open_set[c_node][0] + new_cost < open_set[suc][0]:
                        open_set[suc] = (open_set[c_node][0] + new_cost, c_node)
                else:  # 如果是新节点，直接加入open_set中
                    open_set[suc] = (open_set[c_node][0] + new_cost, c_node)
            closed_set[c_node] = open_set[c_node]
            del open_set[c_node]  # 遍历过该节点，则把该节点从open_set中移除，加入到close_set.

        route.append(n_end)
        while 1:
            if closed_set[route[-1]][1] != -1:
                route.append(closed_set[route[-1]][1])  # 通过不断回溯找到最短路径
            else:
                break
        return list(reversed(route))

    @staticmethod
    def _closest_index(current_waypoint, waypoint_list):
        """
        确定waypoint_list中距离当前路点最近的路点的索引值
        param:  current_waypoint:
                waypoint_list:
        return: 整数， 索引值
        """
        min_distance = float('inf')  # 初始情况下设置为最大值
        closest_index = -1
        for i, waypoint in enumerate(waypoint_list):
            distance = waypoint.transform.location.distance(current_waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def search_path_way(self, origin, destination):
        """
        得到完整的由waypoint构成的完整路径
        param:  origin: 起点，carla.Location类型
                destination: 终点
        return: list类型，元素是(carla.Waypoint类型, edge["type"]),这里多加了一个边的类型进行输出，
                 是为了后面的更全面考虑某些道路规定的跟车或者超车行为
        """
        route = self._route_search(origin, destination)  # 获取A*的初步规划结果->list
        origin_wp = self._map.get_waypoint(origin)  # type: carla.Waypoint
        destination_wp = self._map.get_waypoint(destination)  # type: carla.Waypoint
        path_way = []

        # 第一段路径
        edge = self._graph.get_edge_data(route[0], route[1])
        path = [edge["entry_waypoint"]] + edge["path"] + [edge["exit_waypoint"]]
        clos_index = self._closest_index(origin_wp, path)
        for wp in path[clos_index:]:
            path_way.append((wp, edge["type"]))

        # 中间路径
        if len(route) > 3:  # 先判断是否有中间路径
            for index in range(1, len(route) - 2):
                edge = self._graph.get_edge_data(route[index], route[index + 1])
                path = edge["path"] + [edge["exit_waypoint"]]  # 每一段路段的终点是下一个路段的起点，所以这里不加起点
                for wp in path:
                    path_way.append((wp, edge["type"]))

        # 最后一段路径
        edge = self._graph.get_edge_data(route[-2], route[-1])
        # print(edge)
        path = edge["path"] + [edge["exit_waypoint"]]
        clos_index = self._closest_index(destination_wp, path)
        if clos_index != 0:  # 判断终点是否是当前路段的起点，如果不是，将后续的路点加入path_way;
            for wp in path[:clos_index + 1]:
                path_way.append((wp, edge["type"]))
        else:  # 如果是，后面的路段终点则在上个路段已经添加进path_way中，这里不进行重复操作
            pass
        return path_way
