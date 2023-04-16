import random
import pygame
import math
import numpy as np
from Robot.robot import Robot


class Graphics:
    def __init__(self, dimensions, robot_img_path):
        pygame.init()
        #COLORS
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.mint = (0, 130, 127)
        self.yel = (255, 255, 0)

        # load Robot image
        self.robot = pygame.image.load(robot_img_path)
        self.height, self.width = dimensions    # Screen dimension

        # window settings
        pygame.display.set_caption("AI AVENGERS_Robot Simulator")
        self.map = pygame.display.set_mode((self.width, self.height))
        # self.instruction_surface = pygame.Surface((self.height-600, self.width))
        # self.map.blit(self.instruction_surface,((0, 600)))
        # self.instruction_surface.fill(self.black)

        self.robot_path = []
        self.kf_path = []
        self.ellipse_path = []
        # Dust generation
        self.dust_coords = []
        for i in range(100): # Create random dust
            x = random.randint(50, 1100)
            y1 = random.randint(50, 250)
            y2 = random.randint(350, 500)
            self.dust_coords.append((x, y1))
            self.dust_coords.append((x, y2))

        # Beacon generation
        self.beacon_coords = [(200, 600),(200, 200),(400, 400),(400, 0),(1000, 400),(700,200),
                              (1200,200),(1200,600),(0,0),(0,600),(1200,0)]


        self.width = []
        self.height = []
        self.count = 0
        self.map.blit(self.map, (0, 0))

    def draw_instruction(self):
        # Display instruction for control
        font1 = pygame.font.Font('freesansbold.ttf', 32)
        font2 = pygame.font.Font('freesansbold.ttf', 20)
        text1 = font1.render('Controls:', True, self.black)
        text2 = font2.render("'W' = Increment Velocity", True, self.black)
        text3 = font2.render("'S' = Decrement Velocity", True, self.black)
        text4 = font2.render("'A' = Rotate Left", True, self.black)
        text5 = font2.render("'D' = Rotate Right", True, self.black)
        self.map.blit(text1, (20, 600))
        self.map.blit(text2, (20, 650))
        self.map.blit(text3, (500, 650))
        self.map.blit(text4, (20, 700))
        self.map.blit(text5, (500, 700))

    def draw_map(self):
        self.map.fill(self.white)
        pygame.draw.rect(self.map, self.black, pygame.Rect(0, 0, 1200, 600), 12)

    def draw_beacons(self):
        for coordinate in self.beacon_coords:
            pygame.draw.circle(self.map, self.blue,(coordinate[0], coordinate[1]),5)
        return self.beacon_coords

    def draw_maze(self):
        self.map.fill(self.white)
        pygame.draw.rect(self.map, self.black, pygame.Rect(0, 0, 1200, 600), 1)
        pygame.draw.line(self.map, self.black, (200, 600), (200, 200), 2)
        pygame.draw.line(self.map, self.black, (400, 400), (400, 0), 2)
        pygame.draw.line(self.map, self.black, (400, 400), (1000, 400), 2)
        pygame.draw.line(self.map, self.black, (700, 200), (1200, 200), 2)

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud, robot_pose):
        for point in point_cloud:
            #pygame.draw.circle(self.map, self.red, point, 3, 0)
            # draw point cloud
            pygame.draw.circle(self.map, self.red, point, 3, 0)

            # draw line from robot to point
            pygame.draw.line(self.map, self.black, robot_pose, point, 1)

            # calculate distance between robot and point
            dx = robot_pose[0] - point[0]
            dy = robot_pose[1] - point[1]
            distance = int(math.sqrt(dx * dx + dy * dy))

            # draw text showing distance
            font = pygame.font.Font('freesansbold.ttf', 12)
            text = font.render(str(distance), True, self.red)
            text_rect = text.get_rect(center=((robot_pose[0] + point[0]) // 2, (robot_pose[1] + point[1]) // 2))
            self.map.blit(text, text_rect)

    def draw_beacon_sensor(self, beacons, robot_pose):
        # draw line from robot to point
        for beacon in beacons:
            pygame.draw.line(self.map, self.black, robot_pose, beacon, 1)

    def draw_dust(self):
        for coordinate in self.dust_coords:
            pygame.draw.rect(self.map, self.blue, (coordinate[0], coordinate[1], 30, 30))
        return self.dust_coords

    def remove_dust(self, dust_coor):
        # print('Cloud: ', self.dust_coords)
        # print('sensor: ', dust_coor)
        threshold = 35
        for coord in self.dust_coords:
            for coord2 in dust_coor:
                dist = abs(coord[0] - coord2[0]) + abs(coord[1] - coord2[1])
                if dist <= threshold:
                    # print('remove: ',coord)
                    # print('original: ',self.dust_coords)
                    self.dust_coords.remove(coord)
                    break

    def draw_trace(self, robot_pos):
        # print('Color: ', color)
        self.robot_path.append(robot_pos)
        for coord in self.robot_path:
            pygame.draw.circle(self.map, self.green, coord, 2)

    def draw_trace_kf(self, robot_pos):
        # print('Color: ', color)
        self.kf_path.append(robot_pos)
        dotted_line = self.kf_path
        dotted_line = dotted_line[0::1]

        for coord in dotted_line:
            pygame.draw.circle(self.map, self.red, coord, 1)

    def draw_covariance_ellipse(self, pos, pos_original, cov_matrix, color=(255, 255, 0)):
        if np.linalg.norm(np.array([pos[0],pos[1]]) - np.array([pos_original[0], pos_original[1]])) > 38:
            self.ellipse_path.append(pos)
        ellipses = self.ellipse_path
        ellipses = ellipses[0::50]
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix[0:2, 0:2])
        width, height = 2 * np.sqrt(eigenvalues)
        print(eigenvalues, eigenvectors)

        for coord in ellipses:
            pygame.draw.ellipse(self.map, color,
                                pygame.Rect(coord[0] - width / 2, coord[1] - height / 2, 10 * width, 10 * height), 2)


