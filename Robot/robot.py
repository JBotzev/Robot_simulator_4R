import pygame
import math
import numpy as np


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self, startpos, width, gfx):
        self.gfx = gfx
        self.m2p = 3779.52  # Meter to pixels
        self.w = width/2    # distance between the center of robot and the wheel
        self.r = width/2 + 10   # Radius of rotation


        self.x = startpos[0]    # Present x coordinate of the robot after estimation applied
        self.y = startpos[1]    # Present y coordinate of the robot after estimation applied
        self.x_est = 0    # x coordinate of the robot after estimation applied
        self.y_est = 0    # y coordinate of the robot after estimation applied
        self.heading = math.pi / 2
        self.cov_matrix = None

        self.heading_kf = self.heading
        self.heading_est = 0
        self.x_kf = self.x    # x coordinate of the robot after estimation applied
        self.y_kf = self.y    # y coordinate of the robot after estimation applied

        self.x_prev = startpos[0]    # x coordinate of the robot after estimation applied
        self.y_prev = startpos[1]     # y coordinate of the robot after estimation applied
        self.heading_diff = 0
        self.rot_next = 0

        self.vl = 0
        self.vr = 0
        self.last_vl = 0
        self.last_vr = 0
        self.acc = 0.005
        # self.last_x = self.x
        # self.last_y = self.y

        self.maxspeed = 0.01 * self.m2p
        self.minspeed = self.acc * self.m2p
        self.distances = []
        self.min_obs_dist = 80
        self.min_dust_dist = 35
        self.count_down = 5
        self.closest_obs_size = 0
        self.obs_dist = 0
        self.slide_bool = True
        # TestEnviornment Variables
        self.score = 0
        self.stop = True

    def kalman_filter(self, state_vector, P, z, u, dt):
        """
        Implements the Kalman filter for state estimation of an object moving
        with a model-based motion model and measured with noisy sensors.

        Arguments:
        state_vector -- initial state vector (x_position, y_position, current_direction)
        P -- initial covariance matrix
        z -- estimation of the robot position through the sensors
        u -- control input vector (dvelocity, dangle)
        dt -- time step

        u = should be a 2D vector [velocity, omega(this is the rotation, so an angle I guess/hope)]


        Returns:
        Tuple containing:
        - Estimated state vector (x, y, angle)
        - Updated covariance matrix
        """

        if P is None:
            sigma_x = 3
            P = np.array([[sigma_x ** 2,0,0],
                          [0,sigma_x ** 2,0],
                          [0,0,sigma_x ** 2]])
            # if we dont have a P matrix we should make one for the beginning but play around with the values

        # Define measurement model matrix: H
        A = np.array([[1,0,0],
                     [0,1,0],
                     [0,0,1]])

        C = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        # Define process noise covariance matrix: Q
        sigma_v = 0.1  # velocity noise WE SHOULD PLAY AROUND WITH THIS ONE
        sigma_w = 0.25 # angular velocity noise
        R = np.array([[sigma_v ** 2, 0, 0],
                      [0, sigma_v ** 2, 0],
                      [0, 0, sigma_w ** 2]])

        # Define measurement noise covariance matrix: Q
        sigma_z = 10  # measurement noise WE SHOULD PLAY AROUND WITH THIS ONE THIS ONE INCREASES THE RADIUS OF ELLIPSE
        Q = np.array([[sigma_z ** 2, 0, 0],
                      [0, sigma_z ** 2, 0],
                      [0, 0, sigma_z ** 2]])

        B = np.array([[dt * math.cos(state_vector[2]), 0],
                      [dt* math.sin(state_vector[2]), 0],
                      [0, dt]])

        # Prediction step
        x_ = A.dot(state_vector) + B.dot(u) # calculated with the velocity based model
        P = A.dot(P).dot(A.T) + R
        print('P:', P)

        # Correction step
        z = np.array(z).reshape(3, 1)
        z = z.reshape(3,)# reshape z to (2, 1) arra
        y = z - C.dot(x_)  # extract x and y coordinates from x
        S = C.dot(P).dot(C.T) + Q

        K = P.dot(C.T).dot(np.linalg.inv(S))
        x = x_ + K.dot(y)
        self.cov_matrix = (np.eye(3) - K.dot(C)).dot(P)
        print('end cov_matr:', self.cov_matrix)
        self.x_kf = x[0]
        self.y_kf = x[1]
        self.heading_kf = x[2]
        #print('KF_position:', self.x_kf, self.y_kf)
        #print('real_position:', self.x, self.y)



    # def avoid_obstacles(self, point_cloud, dt):
    #
    #     self.distances = []
    #     closest_obs = None
    #     dist = np.inf
    #     self.closest_obs_size = 0
    #     self.obs_dist = 0
    #
    #     # closest_points = []
    #     if len(point_cloud) >= 1:
    #         for point in point_cloud:   # Figuring out closest obstacle
    #             distance_to_obs = distance([self.x, self.y], point)
    #             self.distances.append(distance_to_obs)
    #             # Collect distances to obstacles
    #             if dist > distance_to_obs:
    #                 dist = distance_to_obs
    #                 closest_obs = (point, dist)
    #                 self.obs_dist = closest_obs[1]
    #                 # Count collisions
    #                 if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
    #                     self.closest_obs_size += 1
    #
    #         # Resetting last saved speeds based on number of collisions
    #         if self.closest_obs_size >= 2:
    #             self.slide_bool = False
    #
    #         # Slide and Stopping conditions when collision occurs
    #         if closest_obs[1] < self.min_obs_dist:
    #             if self.last_vl + self.last_vr != 0 and self.closest_obs_size >= 1 and self.stop == True:
    #                 self.stop_robot()
    #                 # print('STOP SPEED')
    #             hor_wall = True
    #             # Slide if robot is hitting only 1 wall
    #             if self.closest_obs_size == 1:
    #                 if np.abs(self.x - closest_obs[0][0]) > np.abs(self.y - closest_obs[0][1]):
    #                     hor_wall = True
    #                 if np.abs(self.x - closest_obs[0][0]) <= np.abs(self.y - closest_obs[0][1]):
    #                     hor_wall = False
    #                 if self.slide_bool:
    #                     self.slide(hor_wall)
    #         else:
    #             self.count_down = 5
    #
    #
    #         # self.last_vl = 0
    #         # self.last_vr = 0
    #
    #     if self.closest_obs_size == 0:
    #         # print('RESET SLIDE BOOL')
    #         self.slide_bool = True
    #         self.stop = True
    #         self.last_vl = self.vl
    #         self.last_vr = self.vr
    #     # print('LAST VL + VR : ', self.last_vr + self.last_vl, ' --- collisions: ', self.closest_obs_size)
    # # Sliding conditions
    # def slide(self, left_right_walls):
    #
    #     # print('obstacle size: ',self.closest_obs_size)
    #     deg = math.degrees(self.heading)
    #     if self.last_vr + self.last_vl > 0:
    #         # print("FORWARD")
    #         if left_right_walls:
    #             if (0 < deg < 90) or (-270 > deg > -360) or (180 > deg > 90) or (-180 > deg > -270):
    #                 self.y -= 0.5
    #                 # print('right wall, angled up')
    #             if (180 < deg < 270) or (-90 > deg > -180) or (270 < deg) or (0 > deg > -90):
    #                 self.y += 0.5
    #                 # print('left wall angled down')
    #         else:
    #             if (-90 < deg < 90) or (deg < -270) or (deg > 270):
    #                 self.x += 0.5
    #                 # print('top wall, angled right')
    #             if (90 < deg < 270) or (-90 > deg > -270):
    #                 self.x -= 0.5
    #                 # print('top wall, angled left')
    #     elif self.last_vr + self.last_vl < 0:
    #         # print("BACKWARD")
    #         if left_right_walls:
    #             if (0 < deg < 180) or (deg < -180):
    #                 self.y += 0.5
    #                 # print('go down')
    #             if (deg > 180) or (0 > deg > -180):
    #                 self.y -= 0.5
    #                 # print('go up')
    #         else:
    #             if (-90 < deg < 90) or (deg < -270) or (deg > 270):
    #                 self.x -= 0.5
    #                 # print('left')
    #             if (90 < deg < 270) or (-90 > deg > -270):
    #                 self.x += 0.5
    #                 # print('right')
    #
    # def stop_robot(self):
    #     if self.stop:
    #         self.vr = 0
    #         self.vl = 0
    #         self.stop = False
    #
    # def move_backward(self):
    #     self.vr = - self.minspeed
    #     self.vl = - self.minspeed
    #
    # def move_forward(self):
    #     self.vr = self.minspeed
    #     self.vl = self.minspeed

    def kinematics(self, dt):   # Velocity motion model
        self.vl = min(self.vl, self.maxspeed)
        self.vr = min(self.vr, self.maxspeed)

        self.x_prev = self.x
        self.y_prev = self.y

        heading_old = self.heading
        velocity_old = (self.vl + self.vr) / 2
        self.heading_diff = heading_old - self.heading
        self.est_next_rot()
        self.velocity_diff = (self.vl + self.vr) / 2 - velocity_old

        self.x += ((self.vl + self.vr) / 2) * math.cos(self.heading) * dt
        self.y -= ((self.vl + self.vr) / 2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt
        # if self.heading <= 0:   # recalculate in the range of 0 to 360 degrees
        #     self.heading += 2 * math.pi
        # if self.heading >= 2 * math.pi:
        #     self.heading = 0
        # if self.heading <= -math.pi:   # recalculate in the range of 0 to 180 and 0 to -180 degrees
        #     self.heading += 2 * math.pi
        # if self.heading >= math.pi:
        #     self.heading -= 2 * math.pi


    def controls(self, key):
        # W: positive increment of left wheel motor speed
        if key == pygame.K_w:
            # if self.last_vr + self.last_vl < 0 or self.closest_obs_size == 0:
                self.vl += self.acc * self.m2p
                self.vr += self.acc * self.m2p

        if key == pygame.K_s:
            # if self.last_vr + self.last_vl > 0 or self.closest_obs_size == 0:
                self.vl -= self.acc * self.m2p
                self.vr -= self.acc * self.m2p

        if key == pygame.K_a:
            # if self.last_vr + self.last_vl < 0 or self.closest_obs_size == 0:
                self.vl -= self.acc * self.m2p * 0.5

        if key == pygame.K_d:
            # if self.last_vr + self.last_vl < 0 or self.closest_obs_size == 0:
                self.vr -= self.acc * self.m2p * 0.5

        if key == pygame.K_SPACE:
            self.vl = 0
            self.vr = 0


    # EVO and NN functions ------------------------------------------------------------
    # def update_score(self):
    #     v = (abs(self.vl) + abs(self.vr)) / 2 / self.maxspeed
    #     d_v = 1 - abs(self.vl - self.vr) / (abs(self.minspeed) + self.maxspeed)
    #
    #     # 100 is the sensor_range
    #     i = abs(self.min_obs_dist) / 100
    #     score_this_frame = v * math.sqrt(d_v) * i
    #     self.score += score_this_frame
    #     return self.score
    # # End bot control methods

    def set_motor_speed_percentage(self, l_motor, r_motor):
        m_speed = self.maxspeed
        self.vr = r_motor * m_speed
        self.vl = l_motor * m_speed

    # LOCALIZATION ROBOT via BEACONS ----------------------------------------------------
    def calc_bearing(self, beacon):
        # beta = math.atan2(beacon[1], beacon[0]) # Angle between x-axis and beacon vector
        alpha = math.atan2((self.y - beacon[1]), (self.x - beacon[0]))
        # We do the following to match the reference with heading reference
        # if alpha >= 0:
        #     alpha = math.pi - alpha
        # if alpha < 0:
        #     alpha = -alpha - math.pi

        # if alpha < math.radians(-165):
        #     alpha = math.radians(-175)
        # elif alpha > math.radians(165):
        #     alpha = math.radians(175)

        # elif alpha == -math.pi or alpha == math.pi:
        #     alpha = 0


        # x2 = self.x + 40 * math.cos(self.heading) # heading's edge
        # y2 = self.y - 40 * math.sin(self.heading)
        # heading_x = math.atan2(y2, x2) # Angle between x-axis and heading
        # #alpha = beta - heading_x
        # alpha = math.atan2((y2 - beacon[1]), (x2 - beacon[0]))
        #
        # #bearing = beta - alpha
        #
        # bearing = beta - heading_x
        return alpha

    def calc_beacons_in_range(self, beacons):
        beacon_dists = []
        beacon_angles = []
        beacons_in_range = []

        for beacon in beacons:
            dist = distance((self.x, self.y), (beacon[0], beacon[1]))
            if dist < 700:
                beacon_dists.append(dist)
                beacons_in_range.append(beacon)

                bearing = self.calc_bearing(beacon)
                # if bearing < math.radians(-170):
                #     bearing = math.radians(-170)
                beacon_angles.append(bearing)

        return beacon_dists, beacon_angles, beacons_in_range

    def est_position(self, beacons, beacon_dist, beacon_angles, heading, dt):
        x_robot = []
        y_robot = []
        angle_robot = []
        #print("beac ang", beacon_angles)
        for i in range(len(beacons)):   # Estimate position of robot from beacon angle and distance
            theta = beacon_angles[i]
            dist = beacon_dist[i]
            x_robot.append(int(beacons[i][0] + dist * math.cos(theta)))
            y_robot.append(int(beacons[i][1] + dist * math.sin(theta)))

            # # Calculate angle between beacon and reference axis (e.g., x-axis)
            # dx = x_robot[-1] - beacons[i][0]
            # dy = y_robot[-1] - beacons[i][1]
            # beacon_axis_angle = math.atan2(dy, dx)
            # # # Estimate robot's heading angle relative to the reference axis
            # est_robot_angle = beacon_axis_angle - theta
            # angle_robot.append(est_robot_angle)


        # We take the avg of all the estimates from the beacons
        self.x_est = np.mean(x_robot)  # * dt
        self.y_est = np.mean(y_robot)  # * dt
        #self.heading_est = (np.mean(angle_robot) - heading) % (2 * math.pi)  # Normalize the angle
        # print("Estimated positions", self.x_est, self.y_est)
        # print('real position:', self.x, self.y)
        # print("Estimated heading", self.heading_est)
        # print("New est angle", angle_robot)

        # print(beacon_angles)
        # print("x, y", x_robot,y_robot)
        # print("estimate",np.mean(x_robot), np.mean(y_robot))

    # def est_next_pos(self, avg_beac_ang, avg_beac_dist, dt):
    #     # we use the present estimated coordinates
    #     # and average beacon angle and average beacon length
    #     avg_beac_dist = 30
    #     print("avg Beacon dist", avg_beac_dist)
    #     self.x_next = self.x + avg_beac_dist * math.cos(avg_beac_ang) * dt    # multiply with dt if dist is too large
    #     self.y_next = self.y - avg_beac_dist * math.sin(avg_beac_ang) * dt
    #     print("present x, y", self.x, self.y)
    #     print("next x, y", self.x_next, self.y_next)

    def est_next_rot(self):   # We use odometry equations
        # We take the distance between the present and next coordinates of the robot
        # we consider this as the arc lenght of rotation
        # then we use s = r*theta to calculate theta
        #arc = distance([self.x, self.y], [self.x_next, self.y_next])
        arc = distance([self.x, self.y], [self.x_prev, self.y_prev])
        self.rot_next = arc / self.r
        #print("arc, rot", arc, math.degrees(self.rot_next))

        # Apply new rotation to the heading
        #rot_num = abs(self.heading // (2 * math.pi))
        if self.heading <= -math.pi:   # recalculate in the range of 0 to 180 and 0 to -180 degrees
            self.heading += 2 * math.pi
        if self.heading > math.pi:
            self.heading -= 2 * math.pi

        #diff = self.heading - avg_beac_ang
        if self.heading_diff > 0:
            self.heading_est = self.heading - self.rot_next
        else:
            self.heading_est = self.heading + self.rot_next
#
#     def apply_est(self, avg_beac_ang, avg_beac_dist, dt):
#         self.est_next_pos(avg_beac_ang, avg_beac_dist, dt)
#         self.est_next_rot()
#
#         # Apply new estimated position
#         self.x = self.x_next
#         self.y = self.y_next
#
#         # Apply new rotation to the heading
#         #rot_num = abs(self.heading // (2 * math.pi))
#         if self.heading <= -math.pi:   # recalculate in the range of 0 to 180 and 0 to -180 degrees
#             self.heading += 2 * math.pi
#         if self.heading > math.pi:
#             self.heading -= 2 * math.pi
#
#         diff = self.heading - avg_beac_ang
#         if diff > 0:
#             self.heading -= self.rot_next
#         else:
#             self.heading += self.rot_next
#
#
#
# ##########################################
#     # Rough testing
#     def trilateration2(self):
#         alpha = math.atan2((self.y - 200), (self.x - 200))
#         if alpha > 0:
#             alpha = math.pi - alpha
#         elif alpha < 0:
#             alpha = -alpha + math.pi
#         elif alpha == 0:
#             alpha = math.pi
#         elif alpha == -math.pi or alpha == math.pi:
#             alpha = 0
#         print("Angle", math.degrees(alpha))
#
# ##########################################
