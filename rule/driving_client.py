from numba import prange

from drive_controller import DrivingController
import numpy as np


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False
        self.collision_flag = True
        self.collision_count = 0
        self.prev_to_middle = 0.0
        self.before_collision_throttle = 1
        self.full_throttling = True
        self.emergency_braking = False
        self.marina_emergency = False

        self.set_steering = 0.0
        self.set_throttle = 1.0
        self.set_brake = 0.0

        self.steering_by_middle = 0.0
        self.steering_by_angle = 0.0

        self.is_opponent_close = False
        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        if self.is_debug:
            print("=========================================================")
            print("to middle: {}".format(sensing_info.to_middle))

            print("collided: {}".format(sensing_info.collided))
            print("car speed: {} km/h".format(sensing_info.speed))

            print("is moving forward: {}".format(sensing_info.moving_forward))
            print("moving angle: {}".format(sensing_info.moving_angle))
            print("lap_progress: {}".format(sensing_info.lap_progress))

            print("track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################
        # print("collided: {}".format(sensing_info.collided))
        # Moving straight forward

        self.prev_to_middle = abs(sensing_info.to_middle)

        self.set_steering = 0.0
        self.set_throttle = 1.0
        self.set_brake = 0.0

        self.set_steering_with_no_obstacles(sensing_info)

        if sensing_info.speed > 120:
            dist = 80
        else:
            dist = 50
        if len(sensing_info.track_forward_obstacles) > 0 and sensing_info.track_forward_obstacles[0]['dist'] < dist:
            self.set_steering_with_obstacles(sensing_info)

        self.set_steering = self.steering_by_angle + self.steering_by_middle

        if abs(sensing_info.to_middle) > (self.half_road_limit - 2):
            if self.set_steering > 0:
                if sensing_info.to_middle < 0:
                    self.set_steering += 0.1
                else:
                    self.set_steering += -0.1
            else:
                if sensing_info.to_middle < 0:
                    self.set_steering += -0.1
                else:
                    self.set_steering += 0.1

        if abs(self.set_steering) > 1:
            if self.set_steering > 0:
                self.set_steering = 1
            else:
                self.set_steering = -1

        if len(sensing_info.opponent_cars_info) > 0 and sensing_info.opponent_cars_info[0]['dist'] > 4:
            self.is_opponent_close = True
        else:
            self.is_opponent_close = False

        if sensing_info.collided and self.collision_count == 0 and (not self.is_opponent_close):
            self.collision_count = 5
            self.before_collision_throttle *= -1
            self.set_throttle = self.before_collision_throttle
            self.set_brake = 0.0
            if sensing_info.to_middle > 0:
                if sensing_info.moving_angle > 0:
                    self.set_steering = 0.8
                else:
                    self.set_steering = -0.8
            else:
                if sensing_info.moving_angle > 0:
                    self.set_steering = -0.8
                else:
                    self.set_steering = 0.8
        elif self.collision_count > 0:
            self.collision_count -= 1
            self.set_throttle = self.before_collision_throttle
            self.set_brake = 0.0
            if sensing_info.to_middle > 0:
                if sensing_info.moving_angle > 0:
                    self.set_steering = 0.8
                else:
                    self.set_steering = -0.8
            else:
                if sensing_info.moving_angle > 0:
                    self.set_steering = -0.8
                else:
                    self.set_steering = 0.8
        else:
            if sensing_info.moving_forward and abs(sensing_info.moving_angle) < 90:
                self.before_collision_throttle = 1
            elif sensing_info.moving_forward and abs(sensing_info.moving_angle) >= 90:
                self.before_collision_throttle = -1
            elif (not sensing_info.moving_forward) and abs(sensing_info.moving_angle) < 90:
                self.before_collision_throttle = -1
            else:
                self.before_collision_throttle = 1

        if not sensing_info.moving_forward and self.collision_count == 0:
            self.set_steering = -1.0

        car_controls.steering = self.set_steering
        car_controls.throttle = self.set_throttle
        car_controls.brake = self.set_brake

        #print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))
        #print(sensing_info.track_forward_angles)
        #print(np.std(sensing_info.track_forward_angles))
        #print(sensing_info.speed)
        #print(sensing_info.track_forward_obstacles)
        if self.is_debug:
            print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle,
                                                              car_controls.brake))
        #
        # Editing area ends
        # ==========================================================#
        return car_controls

    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name

    def set_steering_with_no_obstacles(self, sensing_info):
        road_ran = 3
        ang = 120
        ang_num = 1

        if sensing_info.speed > 140:
            road_ran = 10
            ang = 75
            ang_num = 4
            if not self.full_throttling:
                ang_num = 3
        elif sensing_info.speed > 110:
            road_ran = 9
            ang = 85
            ang_num = 3
            if not self.full_throttling:
                ang_num = 2
        elif sensing_info.speed > 80:
            road_ran = 6
            ang = 95
            ang_num = 2
        elif sensing_info.speed > 50:
            road_ran = 3
            ang = 120
            ang_num = 1

        # set_throttle = 1.0
        # set_brake = 0.0

        #if self.marina_emergency:
        #    ang_num += 2

        self.steering_by_angle = (sensing_info.track_forward_angles[ang_num] - sensing_info.moving_angle) / ang
        self.steering_by_middle = (sensing_info.to_middle / 50) * -1
        # self.set_steering += self.steering_by_middle

        full_throttle = True
        emergency_brake = False
        is_emergency_direction_right = True
        emergency_start_index = 0

        for i in range(road_ran):
            f_road = abs(sensing_info.track_forward_angles[i])
            # print(f_road)
            if f_road > 50:
                full_throttle = False
            if f_road > 89:
                if sensing_info.track_forward_angles[i] < 0:
                    is_emergency_direction_right = False
                emergency_brake = True
                break

        self.full_throttling = full_throttle
        self.emergency_braking = emergency_brake
        self.marina_emergency = False

        absolute_angles = np.absolute(sensing_info.track_forward_angles)

        if not full_throttle:
            if sensing_info.speed > 130:
                self.set_throttle = 0.5
            if sensing_info.speed > 120:
                self.set_brake = 0.3
            """
            if np.max(sensing_info.track_forward_angles) > 50:
                self.steering_by_middle = ((sensing_info.to_middle-(self.half_road_limit/5)) / 50) * -1
            else:
                self.steering_by_middle = ((sensing_info.to_middle+(self.half_road_limit/5)) / 50) * -1
            """
            if np.std(sensing_info.track_forward_angles) > 28 and 90 > np.max(absolute_angles) and sensing_info.speed > 30:
                self.marina_emergency = True
                self.set_brake = 1.0
                self.set_throttle = 0.2
                if np.max(sensing_info.track_forward_angles) > 50:
                    self.steering_by_angle = self.steering_by_angle + 0.1
                else:
                    self.steering_by_angle = self.steering_by_angle - 0.1

        if emergency_brake:
            if np.std(sensing_info.track_forward_angles) > 25 and sensing_info.speed > 30:
                self.set_brake = 0.6
                """
                if is_emergency_direction_right:
                    self.steering_by_angle += (((self.half_road_limit / 2) / 20) * -1)
                else:
                    self.steering_by_angle += ((self.half_road_limit / 2) / 20)
                """
            else:
                if np.max(sensing_info.track_forward_angles) > 50:
                    self.steering_by_angle = self.steering_by_angle + 0.3
                else:
                    self.steering_by_angle = self.steering_by_angle - 0.3

    def set_steering_with_obstacles(self, sensing_info):
        #print("-------------set_steering_with_obstacles----------------")
        to_middle = sensing_info.to_middle

        obs_to_mid = sensing_info.track_forward_obstacles[0]['to_middle']
        diff = (to_middle - obs_to_mid)
        obs_dist = sensing_info.track_forward_obstacles[0]['dist']
        target_selected = False
        target = 0.0

        if abs(diff) < 4 or abs(obs_to_mid) < 2:
            to_be_target = [obs_to_mid-5, obs_to_mid+5]
            """
            print(obs_dist)
            print(int(obs_dist/10))
            if int(obs_dist/10) >= 1:
                print(np.std(sensing_info.track_forward_angles[0:int(obs_dist/10)]))
            """

            for i in range(2):
                if abs(to_be_target[i]) > (self.half_road_limit-1.25):
                    target = to_be_target[1-i]
                    target_selected = True
                    break
            
            if not target_selected:
                if len(sensing_info.opponent_cars_info) > 0 and sensing_info.opponent_cars_info[0]['dist'] > 5:
                    opponent_tomiddle = sensing_info.opponent_cars_info[0]['to_middle']
                    if abs(to_be_target[0]-opponent_tomiddle) < abs(to_be_target[1]-opponent_tomiddle):
                        target = to_be_target[0]
                        target_selected = True
                    else:
                        target = to_be_target[1]
                        target_selected = True

            if not target_selected:
                if len(sensing_info.track_forward_obstacles) > 1 and (sensing_info.track_forward_obstacles[1]['dist'] - obs_dist) < 30:
                    second_obs_to_mid = sensing_info.track_forward_obstacles[1]['to_middle']
                    if second_obs_to_mid < 2:
                        second_to_be_target = [second_obs_to_mid - 5, second_obs_to_mid + 5]
                        if abs(second_to_be_target[0] - sensing_info.track_forward_obstacles[1]['to_middle']) < abs(second_to_be_target[1] - sensing_info.track_forward_obstacles[1]['to_middle']):
                            target = second_to_be_target[0] + 3
                            target_selected = True
                        else:
                            target = second_to_be_target[1] - 3
                            target_selected = True

            if not target_selected:
                if abs(to_be_target[0] - to_middle) < abs(to_be_target[1] - to_middle):
                    target = to_be_target[0]
                    target_selected = True
                else:
                    target = to_be_target[1]
                    target_selected = True

        elif len(sensing_info.track_forward_obstacles) > 1 and (sensing_info.track_forward_obstacles[1]['dist'] - obs_dist) < 30:
            second_obs_tomiddle = sensing_info.track_forward_obstacles[1]['to_middle']
            if abs(second_obs_tomiddle - sensing_info.to_middle) < 4:
                to_be_target = [second_obs_tomiddle - 5, second_obs_tomiddle + 5]

                for i in range(2):
                    if abs(to_be_target[i]) > (self.half_road_limit - 1.25):
                        target = to_be_target[1 - i]
                        target_selected = True
                        break
                    if (to_be_target[i] - second_obs_tomiddle) * (sensing_info.track_forward_obstacles[0]['to_middle'] - second_obs_tomiddle) > 0:
                        if abs((to_be_target[i] - second_obs_tomiddle)) > abs(sensing_info.track_forward_obstacles[0]['to_middle'] - second_obs_tomiddle):
                            target = to_be_target[1 - i]
                            target_selected = True
                            break

                if not target_selected:
                    if len(sensing_info.opponent_cars_info) > 0 and sensing_info.opponent_cars_info[0]['dist'] > 5:
                        opponent_tomiddle = sensing_info.opponent_cars_info[0]['to_middle']
                        if abs(to_be_target[0] - opponent_tomiddle) < abs(to_be_target[1] - opponent_tomiddle):
                            target = to_be_target[0]
                            target_selected = True
                        else:
                            target = to_be_target[1]
                            target_selected = True

                if not target_selected:
                    if abs(to_be_target[0] - to_middle) < abs(to_be_target[1] - to_middle):
                        target = to_be_target[0]
                        target_selected = True
                    else:
                        target = to_be_target[1]
                        target_selected = True

        if sensing_info.speed > 120 and obs_dist > 50:
            target *= 1.5

        if obs_dist < 40 and abs(obs_to_mid - to_middle) < 2.5:
            if sensing_info.speed > 70:
                print("2---")
                self.set_brake = 0.5
            if sensing_info.speed > 120:
                print("3---")
                target *= 1.5
                self.set_brake = 1.0
                self.set_throttle = 0.5

            to_obs_angle = 0.0
            if obs_dist < 10:
                to_obs_angle = sensing_info.track_forward_angles[0]
            elif obs_dist < 20:
                to_obs_angle = np.std(sensing_info.track_forward_angles[0:1])
            elif obs_dist < 30:
                to_obs_angle = np.std(sensing_info.track_forward_angles[0:2])
            elif obs_dist < 40:
                to_obs_angle = np.std(sensing_info.track_forward_angles[0:3])

            if len(sensing_info.track_forward_obstacles) > 0:
                check_car_moving_angle = 5
            else:
                check_car_moving_angle = 3

            if abs(sensing_info.moving_angle) < check_car_moving_angle and to_obs_angle < 3: # 마리나 장애물 회피 검증 필요
                if sensing_info.speed < 50:
                    target *= 2.0
                    print("4---")
                else:
                    target *= 1.5
                    print("5---")


        if target_selected:
            #print("Target!! : {}".format(target))
            self.steering_by_middle = round(self.steer_val_by_to_middle(to_middle - target), 4)
            self.steering_by_angle = round(self.steer_by_forward_road(sensing_info), 4)


        """
        val = 0
        if abs(diff) < 3.5:
            # print("111111111111")
            if abs(obs_to_mid) < 1.5:
                # print("1111111111")
                temp = int(sensing_info.track_forward_obstacles[0]['dist'] / 10)
                temp2 = int(sensing_info.track_forward_obstacles[0]['dist'] % 10)
                # temp3 = 1 if temp2 > 5 else 0
                count = temp + temp2
                temp4 = False
                if count > 0:
                    temp4 = True
                    before_obs_angle = np.mean(sensing_info.track_forward_angles[2:count])
                if temp4:
                    if abs(before_obs_angle) > 50:  # and obs_dist > 10:, 왜 obs_dist 조건을 걸었던건지 이해가 안된다 ㅠㅠ # 코너링 중임
                        # print("333333333333")
                        self.set_throttle = 0.7
                        if sensing_info.speed > 80:
                            self.set_brake = 0.2

                        if before_obs_angle > 0:
                            if to_middle > 0:
                                to_middle = -1.5  # 오른쪽으로 코너링
                                # print("1")
                            else:
                                to_middle = 2.0  # 왼쪽으로 코너링
                                # print("2")
                        else:
                            if to_middle > 0:
                                to_middle = -2.0  # 오른쪽으로 코너링
                                # print("3")
                            else:
                                to_middle = 1.5  # 왼쪽으로 코너링
                                # print("4")
                    else:
                        if to_middle > 0:
                            to_middle = -1.0
                        else:
                            to_middle = 1.0
            else:
                # print("44444444444444")
                if sensing_info.track_forward_obstacles[0]['dist'] > 20:
                    val = -1.0 if diff > 0 else 1.0
                else:
                    val = -2.0 if diff > 0 else 2.0

        else:  # 현재 주행상 부딪히지 않으면서
            # print("222222222")
            temp = int(sensing_info.track_forward_obstacles[0]['dist'] / 10)
            temp2 = int(sensing_info.track_forward_obstacles[0]['dist'] % 10)
            # temp3 = 1 if temp2 > 5 else 0
            count = temp + temp2
            temp4 = False
            if count > 0:
                temp4 = True
                before_obs_angle = np.mean(sensing_info.track_forward_angles[2:count])
            if temp4:
                if abs(before_obs_angle) > 50:  # 코너링 중임
                    self.set_throttle = 0.7
                    if sensing_info.speed > 80:
                        self.set_brake = 0.2

                    if before_obs_angle > 0:
                        if to_middle > 0:
                            to_middle = -1.5  # 오른쪽으로 코너링
                            # print("1")
                        else:
                            to_middle = 2.0  # 왼쪽으로 코너링
                            # print("2")
                    else:
                        if to_middle > 0:
                            to_middle = -2.0  # 오른쪽으로 코너링
                            # print("3")
                        else:
                            to_middle = 1.5  # 왼쪽으로 코너링
                            # print("4")

            if abs(obs_to_mid) > 3.0:  # 장애물의 위치가 중앙이 아닌 경우에만 감속
                # print("주행 경로 아님, 장애물 중앙 아님")
                if not self.full_throttling:
                    # print(obs_dist)
                    if sensing_info.speed > 60:
                        self.set_throttle = 0
                        # print("1")
                    if sensing_info.speed > 50:
                        self.set_brake = 1
                        # print("2")
                elif self.emergency_braking:
                    # print("3")
                    if sensing_info.speed > 50:
                        self.set_brake = 1
                        self.set_throttle = 0

        # 두번째 장애물이 중간에 위치하며, 차량도 중간을 달리고있을때
        if len(sensing_info.track_forward_obstacles) > 1:
            second_to_middle = sensing_info.to_middle
            second_obs_to_mid = sensing_info.track_forward_obstacles[1]['to_middle']
            second_diff = (second_to_middle - second_obs_to_mid)
            second_obs_dist = sensing_info.track_forward_obstacles[1]['dist'] - sensing_info.track_forward_obstacles[0]['dist']
            if abs(second_diff) < 3.5 and abs(second_obs_to_mid) < 1.5 and second_obs_dist < 30:
                # print("111")
                if second_to_middle > 0:
                    to_middle = -3.0
                else:
                    to_middle = 3.0

        if sensing_info.speed > 90 and abs(diff) < 2 and obs_dist < 30:
            # print("5555555555555555")
            # val *= 2.8
            # if val != 0 and obs_dist < 30:
            self.set_brake = 0.7
            self.set_throttle = 0.5
        elif sensing_info.speed > 75:
            val *= 1.7

        to_middle += val
        

        self.steering_by_middle = round(self.steer_val_by_to_middle(to_middle), 4)
        self.steering_by_angle = round(self.steer_by_forward_road(sensing_info), 4)
        """
    def steer_by_forward_road(self, sensing_info):
        return (sensing_info.track_forward_angles[0] - sensing_info.moving_angle) / 60

    def steer_val_by_to_middle(self, to_middle):
        steering = abs(to_middle) / 50
        if to_middle > 0:
            steering *= -1
        return steering


if __name__ == '__main__':
    client = DrivingClient()
    client.run()