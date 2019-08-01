from drive_controller import DrivingController


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
        #print("collided: {}".format(sensing_info.collided))
        # Moving straight forward

        self.prev_to_middle = abs(sensing_info.to_middle)

        road_ran = 3
        ang = 120
        ang_num = 1

        if sensing_info.speed > 110:
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

        set_throttle = 1.0
        set_brake = 0.0

        set_steering = (sensing_info.track_forward_angles[ang_num] - sensing_info.moving_angle) / ang
        middle_add = (sensing_info.to_middle / 50) * -1
        set_steering += middle_add

        full_throttle = True
        emergency_brake = False
        is_emergency_direction_right = True
        emergency_start_index = 0

        for i in range(road_ran):
            f_road = abs(sensing_info.track_forward_angles[i])
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

        if not full_throttle:
            if sensing_info.speed > 130:
                set_throttle = 0.5
            if sensing_info.speed > 120:
                set_brake = 1.0

        if emergency_brake:
            if emergency_start_index > 4:
                if is_emergency_direction_right:
                    print("emergency_right")
                    set_brake = 1.0
                    set_steering += (((self.half_road_limit / 2) / 20) * -1)
                else:
                    set_steering += ((self.half_road_limit / 2) / 20)
            else:
                if set_steering > 0:
                    set_steering = set_steering + 0.3
                else:
                    set_steering = set_steering - 0.3

        to_middle = sensing_info.to_middle

        if len(sensing_info.track_forward_obstacles) > 0 and sensing_info.track_forward_obstacles[0]['dist'] < 40:
            obs_to_mid = sensing_info.track_forward_obstacles[0]['to_middle']
            diff = (to_middle - obs_to_mid)
            obs_dist = sensing_info.track_forward_obstacles[0]['dist']

            val = 0

            if abs(obs_to_mid) < 1:
                to_middle = -2.5
            elif diff < 3.6:
                val = -1 if obs_to_mid < 0 else 1

            if sensing_info.speed > 75:
                val *= 1.7
            elif sensing_info.speed > 90:
                val *= 2.6
                if val > 0 and obs_dist < 30:
                    set_brake = 1
                    set_throttle = 0.5
            to_middle += val

            str_val = round(self.steer_val_by_to_middle(to_middle), 4)
            str_val2 = round(self.steer_by_forward_road(sensing_info), 4)

            final_str = str_val + str_val2
            if final_str > 1:

                final_str = 1

            set_steering = final_str

        if sensing_info.collided and self.collision_count == 0:
            self.collision_count = 5
            self.before_collision_throttle *= -1
            set_throttle = self.before_collision_throttle

            if sensing_info.to_middle > 0:
                set_steering = 0.5
            else:
                set_steering = -0.5

        elif self.collision_count > 0:
            self.collision_count -= 1
            set_throttle = self.before_collision_throttle
            if sensing_info.to_middle > 0:
                set_steering = 0.5
            else:
                set_steering = -0.5
        else:
            if sensing_info.moving_forward and abs(sensing_info.moving_angle) < 90:
                self.before_collision_throttle = 1
            elif sensing_info.moving_forward and abs(sensing_info.moving_angle) >= 90:
                self.before_collision_throttle = -1
            elif (not sensing_info.moving_forward) and abs(sensing_info.moving_angle) < 90:
                self.before_collision_throttle = -1
            else:
                self.before_collision_throttle = 1

        if not sensing_info.moving_forward:
            set_steering = -1.0

        car_controls.steering = set_steering
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake

        print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle,
                                                          car_controls.brake))
        print(sensing_info.track_forward_angles)
        if self.is_debug:
            print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

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

    def get_steering_no_obstacles_no_curve(self, sensing_info):
        steering = (sensing_info.to_middle * -1) / self.half_road_limit

        if steering > 0.15:
            steering = 0.15
        elif steering < -0.15:
            steering = -0.15
        return steering

    def steer_by_forward_road(self, sensing_info):
        return (sensing_info.track_forward_angles[0] - sensing_info.moving_angle) / 60

    def steer_val_by_to_middle(self, to_middle):
        steering = abs(to_middle) / 40
        if to_middle > 0:
            steering *= -1
        return steering

if __name__ == '__main__':
    client = DrivingClient()
    client.run()

    """
        forword_obstacles = sensing_info.track_forward_obstacles
        forword_angles = sensing_info.track_forward_angles


        if len(forword_obstacles) == 0:
            angle_sum = 0;
            for i in range(len(forword_angles)):
                if i > 5:
                    break
                angle_sum += (forword_angles[i] - sensing_info.moving_angle)

            if angle_sum > 350:
                if sensing_info.speed > 60:
                    car_controls.throttle = 0
                car_controls.steering = 0.5
            elif angle_sum > 300:
                car_controls.throttle = 0.2
                car_controls.steering = 0.4
            elif angle_sum > 250:
                car_controls.throttle = 0.4
                car_controls.steering = 0.3
            elif angle_sum > 200:
                car_controls.throttle = 0.6
                car_controls.steering = 0.2
            elif angle_sum < -350:
                if sensing_info.speed > 60:
                    car_controls.throttle = 0
                car_controls.steering = -0.5
            elif angle_sum < -300:
                car_controls.throttle = 0.2
                car_controls.steering = -0.4
            elif angle_sum < -250:
                car_controls.throttle = 0.4
                car_controls.steering = -0.3
            elif angle_sum < -200:
                car_controls.throttle = 0.6
                car_controls.steering = -0.2
            else:
                car_controls.steering = self.get_steering_no_obstacles_no_curve(sensing_info)
        else:
            count = 0
            for obstacle in forword_obstacles:
                if obstacle['dist'] > 15:
                    break
                else:
                    count += 1
                    diff_to_middle = sensing_info.to_middle - obstacle['to_middle']
                    if abs(diff_to_middle) < 2.25:
                        if diff_to_middle > 0:
                            car_controls.steering = 0.3
                        else:
                            car_controls.steering = -0.3

                        if sensing_info.speed > 70:
                            car_controls.brake = 0.5
            if count == 0:
                car_controls.steering = self.get_steering_no_obstacles_no_curve(sensing_info)

        if sensing_info.speed > 60:
            car_controls.throttle = 0

        if (self.half_road_limit + 1.25) < abs(sensing_info.to_middle):
            if sensing_info.to_middle > 0:
                car_controls.steering = -0.3
            else:
                car_controls.steering = 0.3

        if abs(sensing_info.moving_angle) > 45:
            if sensing_info.moving_angle > 0:
                car_controls.steering = -0.3
            else:
                car_controls.steering = 0.3

        if sensing_info.collided and self.collision_count == 0:
            self.collision_count = 5
            car_controls.throttle = -1
            if sensing_info.to_middle > 0:
                car_controls.steering = 0.5
            else:
                car_controls.steering = -0.5
        elif self.collision_count > 0:
            self.collision_count -= 1
            car_controls.throttle = -1
            if sensing_info.to_middle > 0:
                car_controls.steering = 0.5
            else:
                car_controls.steering = -0.5
                
                bjsds.kim
    """
