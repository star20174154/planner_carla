# -*- coding: utf-8 -*-
# @Time    : 2023/5/24 11:37
# @Author  : Star
# @File    : 3.py
# @Software: PyCharm


#下面是道路边界获取函数
        """
        1、撒点采样
        2、获取每个点的id
        3、遍历每一个点，对她们的id进行比较：看是不是有返回值null；看有几种
        4、计算左右各有多少种
        """
        left_road_boundary=[]
        right_road_boundary = []
        for i in range(len(self.local_frenet_path )):
            posx,posy=self.local_frenet_path [i][0],self.local_frenet_path [i][1]
            wp=self._map.get_waypoint(carla.Location(x=posx,y=posy))
            print('wp', wp)

            # wp_right_loc = None
            # wp_left_loc = None

            self_lane_width = wp.lane_width
            print('lane_width', self_lane_width)
            self_lane_id = wp.lane_id
            print('当前点lane id', self_lane_id)

            # 计算每个采样点之间的间隔距离
            interval = 1.5  # 采样间隔为1.5m

            # 获取车道线的中心点和正交向量
            center_location = wp.transform.location
            forward_vector = wp.transform.rotation.get_forward_vector()

            # 将 forward_vector 转换为 carla.Vector3D
            forward_vector_carla = carla.Vector3D(forward_vector.x, forward_vector.y, forward_vector.z)
            #print('当前点前进方向 方向向量', forward_vector_carla)
            # 计算水平垂直向量
            horizontal_vector = carla.Vector3D(-forward_vector_carla.y, forward_vector_carla.x, 0)
            # print('当前点前进方向 垂直方向方向向量', horizontal_vector)
            # 归一化得到单位向量
            unit_horizontal_vector = horizontal_vector / math.sqrt(horizontal_vector.x**2 + horizontal_vector.y**2 + horizontal_vector.z**2)
            #print('当前点前进方向 垂直方向单位向量', unit_horizontal_vector)
            sampled_points = []

            #for i in range(-21 // 2, 21 // 2 + 1):
            for i in range(-10, 10):
                lateral_offset = i * interval
                #print('lateral_offset', lateral_offset)
                # 计算采样点在世界坐标系中的位置
                # sample_point_location_out = center_location + lateral_offset * forward_vector
                # print('当前点前进方向 采样位置', sample_point_location_out)
                sample_point_location = center_location + lateral_offset * unit_horizontal_vector
                #print('当前点前进方向 垂直方向采样位置', sample_point_location)
                sampled_points.append(sample_point_location)
            #可视化采样点
            for sampled_point in sampled_points:
                self._world.debug.draw_point(carla.Location(sampled_point.x, sampled_point.y, 1.5),
                                             size=0.05, color=carla.Color(0, 255, 255), life_time=0.5)

            sam_waypoints = []

            for sample_point in sampled_points:
                # 根据采样点的位置创建Location对象
                sample_point_location1 = carla.Location(sample_point.x, sample_point.y, sample_point.z)
                print('进入waypoint转换前定位位置', sample_point_location1)
                # 获取该位置对应的车道点
                sam_waypoint = self._map.get_waypoint(sample_point_location1)
                print('进入waypoint转换后定位位置', sam_waypoint.transform.location)
                sam_lane_id = sam_waypoint.lane_id
                print('当前采样点lane id', sam_lane_id)
                sam_waypoints.append(sam_waypoint)
            #统计整理这些waypoint的id
            #
            all_line_num = len(sam_waypoints)
            print('当前车道数量', all_line_num)
            all_line_width = all_line_num * self_lane_width
            print('当前总车道宽度', all_line_width)
            #获取当前车道左右各有几条车道    ！重点是种类
            left_lines = []
            right_lines = []
            lane_type_id = []
            lane_type_num = 0
            # for sam_waypoint1 in sam_waypoints:
            #     line_id_type = sam_waypoint[sam_waypoint1]
            #     if sam_waypoint1.lane_id != sam_waypoint[sam_waypoint1]:
            #         lane_type_num += 1
            #         line_id_type =
            #
            #     if sam_waypoint1.lane_id > 0:
            #         if sam_waypoint1.lane_id > self_lane_id:
            #             right_lines.append(sam_waypoint1)
            #         else:
            #             left_lines.append(sam_waypoint1)
            print('right_lines长度', len(right_lines))
            print('left_lines', len(left_lines))





            for sam_waypoint1 in sam_waypoints:
                self._world.debug.draw_point(carla.Location(sam_waypoint1.transform.location.x, sam_waypoint1.transform.location.y, 2),
                                             size=0.05, color=carla.Color(0, 0, 255), life_time=0.5)

            # for sampled_point in sampled_points:
            #     self._world.debug.draw_point(
            #         carla.Location(sampled_point.x,sampled_point.y, 2),
            #         size=0.05, color=carla.Color(0, 255, 255), life_time=0.5)
            print('sam_waypoints ', sam_waypoints)
            print('sam_waypoints 长度', len(sam_waypoints))

            # lanecnt=1
            #
            # while True:
            #     if wp.get_right_lane() is None:
            #         break
            #     else:
            #         if (wp.get_right_lane().lane_id>wp.lane_id ):
            #                 break
            #         wp=wp.get_right_lane()
            # right_road_boundary.append(wp)
            # while True:
            #     if wp.get_left_lane() is None:
            #         break
            #     else:
            #         if (wp.get_left_lane().lane_id < wp.lane_id):
            #             break
            #         wp=wp.get_left_lane()
            #         lanecnt=lanecnt+1
            # left_road_boundary.append(wp)
            # if lanecnt<4:
                # print(wp.lane_id)
                # print(lanecnt)
                # print('ss')

        # for boundary in right_road_boundary:
        #     self._world.debug.draw_point(carla.Location(boundary.transform.location.x, boundary.transform.location.y, 2),
        #                                  size=0.05, color=carla.Color(0, 255, 255), life_time=0.5)
        # for boundary in left_road_boundary:
        #     self._world.debug.draw_point(carla.Location(boundary.transform.location.x, boundary.transform.location.y, 2),
        #                                  size=0.05, color=carla.Color(0, 255, 255), life_time=0.5)


        # print("match_point_list[0]:",match_point_list[0])
        # transform = model3_actor.get_transform()
        # 不断更新观测视角的位置， update the position of spectator

        vehicle_loc = hero_transform.location