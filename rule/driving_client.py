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
        car_controls.steering = 0
        car_controls.throttle = 1
        car_controls.brake = 0

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

        print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle,
                                                          car_controls.brake))

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


if __name__ == '__main__':
    client = DrivingClient()
    client.run()
