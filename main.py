import math

import numpy as np
import pygame
import pickle
from Robot.robot import Robot
from Graphics.graphics import Graphics
from Sensors.infrared import Ultrasonic

MAP_DIMENSIONS = (600, 1200)

########################

# Load the original PNG image
original_image = pygame.image.load('Robot1.png')

# Create a new Surface with the desired size
new_size = (80, 80)
new_image = pygame.Surface(new_size, pygame.SRCALPHA)

# Scale the original image and blit it onto the new Surface
scaled_image = pygame.transform.smoothscale(original_image, new_size)
new_image.blit(scaled_image, (0, 0))

# Save the new image as a PNG file
pygame.image.save(new_image, 'new.png')

#######################

# the environment graphics

gfx = Graphics(MAP_DIMENSIONS, 'new.png')
#env = TestEnvironment()

# the robot
start = (100, 400)
robot = Robot(start, new_size[0] , gfx) # 0.1 * 3779.52

# the sensor
sensor_range = 320, math.radians(180)
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

running = True
we_play = True

if we_play:
    # simulation Loop
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                robot.controls(event.key)

        dt = (pygame.time.get_ticks() - last_time) / 1000
        last_time = pygame.time.get_ticks()

        gfx.draw_maze()
        gfx.draw_beacons()


        robot.kinematics(dt)
        beacon_dists, beacon_angles, beacons_in_range = robot.calc_beacons_in_range(gfx.beacon_coords)
        beacons, distances, angles = ultra_sonic.sense_obstacles(robot.x, robot.y, beacons_in_range, beacon_angles, beacon_dists)
        # print("new angles", angles)
        # print("Distances", distances)
        gfx.draw_beacon_sensor(beacons, [robot.x, robot.y])
        #print(robot.x_kf, robot.y_kf, robot.heading_kf)
        robot.est_position(beacons, distances, angles, robot.heading, dt)    # Estimate current position with beacons
        robot.kalman_filter([robot.x_kf, robot.y_kf, robot.heading_kf], robot.cov_matrix, [robot.x_est, robot.y_est, robot.heading_est], [robot.velocity_diff, robot.heading_diff], dt)  # Estimate current position with Kalman Filter
        gfx.draw_robot(robot.x, robot.y, robot.heading)  # Draw Robot
        gfx.draw_trace([robot.x, robot.y])  # Trace original robot position
        robot_pos_est = (float(robot.x_kf), float(robot.y_kf))
        #print("real pos", robot.x, robot.y)
        #print("KF pos", robot_pos_est)
        gfx.draw_trace_kf(robot_pos_est)  # Trace estimated robot position
        gfx.draw_covariance_ellipse([robot.x_kf, robot.y_kf], [robot.x, robot.y], robot.cov_matrix, color=gfx.red)

        pygame.display.update()
else:
    pass
