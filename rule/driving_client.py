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

        if len(sensing_info.track_forward_obstacles) > 0 and sensing_info.track_forward_obstacles[0]['dist'] < 40:
            self.set_steering_with_obstacles(sensing_info)

        self.set_steering = self.steering_by_angle + self.steering_by_middle

        if abs(sensing_info.to_middle) > (self.half_road_limit-2):
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

        if sensing_info.collided and self.collision_count == 0:
            self.collision_count = 5
            self.before_collision_throttle *= -1
            self.set_throttle = self.before_collision_throttle
            if sensing_info.to_middle > 0:
                self.set_steering = 0.0
            else:
                self.set_steering = -0.0
        elif self.collision_count > 0:
            self.collision_count -= 1
            self.set_throttle = self.before_collision_throttle
            if sensing_info.to_middle > 0:
                self.set_steering = 0.0
            else:
                self.set_steering = -0.0
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

        if self.marina_emergency:
            ang_num += 4

        self.steering_by_angle = (sensing_info.track_forward_angles[ang_num] - sensing_info.moving_angle) / ang
        self.steering_by_middle = (sensing_info.to_middle / 50) * -1
        #self.set_steering += self.steering_by_middle

        full_throttle = True
        emergency_brake = False
        is_emergency_direction_right = True
        emergency_start_index = 0

        for i in range(road_ran):
            f_road = abs(sensing_info.track_forward_angles[i])
            #print(f_road)
            if f_road > 50:
                full_throttle = False
                emergency_start_index = i
            if f_road > 90:
                if sensing_info.track_forward_angles[i] < 0:
                    is_emergency_direction_right = False
                emergency_brake = True
                break

        self.full_throttling = full_throttle
        self.emergency_braking = emergency_brake
        self.marina_emergency = False

        if not full_throttle:
            if sensing_info.speed > 130:
                self.set_throttle = 0.5
            if sensing_info.speed > 120:
                self.set_brake = 0.3
            if np.std(sensing_info.track_forward_angles) > 30 and (not emergency_brake):
                self.marina_emergency = True
                self.set_brake = 0.9

        if emergency_brake:
            if np.std(sensing_info.track_forward_angles) > 25:
                self.set_brake = 0.6
                """
                if is_emergency_direction_right:
                    self.steering_by_angle += (((self.half_road_limit / 2) / 20) * -1)
                else:
                    self.steering_by_angle += ((self.half_road_limit / 2) / 20)
                """
            else:
                if self.set_steering > 0:
                    self.steering_by_angle = self.steering_by_angle + 0.3
                else:
                    self.steering_by_angle = self.steering_by_angle - 0.3

    def set_steering_with_obstacles(self, sensing_info):
        to_middle = sensing_info.to_middle

        obs_to_mid = sensing_info.track_forward_obstacles[0]['to_middle']
        diff = (to_middle - obs_to_mid)
        obs_dist = sensing_info.track_forward_obstacles[0]['dist']

        val = 0
        if abs(diff) < 3.5:
            if abs(obs_to_mid) < 1.5:
                if to_middle > 0:
                    to_middle = -0.5
                else:
                    to_middle = 0.5
            else:
                val = -1 if diff > 0 else 1
        else: # 현재 주행상 부딪히지 않으면서
            if abs(obs_to_mid) > 3.0: # 장애물의 위치가 중앙이 아닌 경우에만 감속
                #print("주행 경로 아님, 장애물 중앙 아님")
                if not self.full_throttling:
                    # print(obs_dist)
                    if sensing_info.speed > 60:
                        self.set_throttle = 0
                        #print("1")
                    if sensing_info.speed > 50:
                        self.set_brake = 1
                        #print("2")
                elif self.emergency_braking:
                        #print("3")
                        self.set_brake = 1
                        self.set_throttle = 0

        # 두번째 장애물이 중간에 위치하며, 차량도 중간을 달리고있을때
        if len(sensing_info.track_forward_obstacles) > 1:
            second_to_middle = sensing_info.to_middle
            second_obs_to_mid = sensing_info.track_forward_obstacles[1]['to_middle']
            second_diff = (second_to_middle - second_obs_to_mid)
            second_obs_dist = sensing_info.track_forward_obstacles[1]['dist'] - sensing_info.track_forward_obstacles[0]['dist']
            if abs(second_diff) < 3.5 and abs(second_obs_to_mid) < 1.5 and second_obs_dist < 30:
                #print("111")
                if second_to_middle > 0:
                    to_middle = -3.0
                else:
                    to_middle = 3.0

        if sensing_info.speed > 90 and abs(diff) < 2 and obs_dist < 30:
            #val *= 2.8
            #if val != 0 and obs_dist < 30:
            self.set_brake = 1
            self.set_throttle = 0.5
        elif sensing_info.speed > 75:
            val *= 1.7


        to_middle += val

        self.steering_by_middle = round(self.steer_val_by_to_middle(to_middle), 4)
        self.steering_by_angle = round(self.steer_by_forward_road(sensing_info), 4)

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