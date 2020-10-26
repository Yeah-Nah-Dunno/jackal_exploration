#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.path as mpltPath
import math
from scipy.spatial import distance
import tf

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

cost_update = OccupancyGridUpdate()
result = MoveBaseActionResult()
newPose = PoseStamped()
q = Quaternion()


class Frontier:
    def __init__(self):
        self.br = CvBridge()
        self.flagm = 0
        self.init = 0
        self.newcost = []
        self.image = None
        self.loop_rate = rospy.Rate(1)
        self.nodes = []
        self.endPoints = []
        self.listener = tf.TransformListener()
        self.track = []
        self.endPoints = []
        self.grid = OccupancyGrid()
        self.costmap = OccupancyGrid()
        self.locX = 0
        self.locY = 0
        self.track_list = []
        self.originX = 0
        self.originY = 0
        self.res = 0
        self.start = 1
        self.goalX = 0
        self.goalY = 0
        self.result = 0
        self.goal = [0, 0]

        self.map = rospy.Subscriber('/map_matrix', OccupancyGrid, self.edgeDetection, queue_size=1)

        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCb, queue_size=1)
        self.global_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costCb, queue_size=1)
        self.globalup_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.costUpCb, queue_size=1)

        self.result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.sendNavCb)
        self.feedback_sub = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.feedbackCb)

        self.location = rospy.Subscriber('/odometry/filtered', Odometry, self.referenceFrame, queue_size=1)
        # self.location = rospy.Subscriber('/tf/mid_mount', Odometry, self.referenceFrame, queue_size=1)


        self.pub_image = rospy.Publisher('/image', Image, queue_size=1)
        self.pub = rospy.Publisher('/Frontier_Extraction', OccupancyGrid, queue_size=1)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)


    def referenceFrame(self, data):
        # x and y location of the robot in /odometry cartesian coordinates
        self.locX = data.pose.pose.position.x
        self.locY = data.pose.pose.position.y



    # def costCb(self, data):
    #     # Where the global map [0, 0] is located in the image matrix
    #     self.res = data.info.resolution
    #     self.originX = data.info.origin.position.x
    #     self.originY = data.info.origin.position.y
    #
    #     # If the current robot position in /odometry is not already in track_list, add it.
    #     if [self.locX, self.locY] not in self.track_list:
    #         self.track_list.append([self.locX, self.locY])
    #
    #     self.grid = data
    #     width = self.grid.info.width
    #     height = self.grid.info.height
    #     pnts = self.grid.data
    #     cnt = 0
    #     pic_mat = []
    #
    #     for x in range(height):
    #         pic_x = []
    #         for y in range(width):
    #             p = pnts[cnt]
    #             pic_x.append(p)
    #             cnt += 1
    #         pic_mat.append(pic_x)
    #


    # def costUpCb(self, data):
    #     map_update = data
    #     width = map_update.width
    #     height = map_update.height
    #     pnts = map_update.data
    #     gx = cost_update.x
    #     gy = cost_update.y
    #     cnt = 0
    #     # Extract a picture matrix from the 1 dimension map.data list
    #     for x in range(height):
    #         for y in range(width):
    #             g = pnts[cnt]
    #             self.newcost[x + gy][y + gx] = g
    #             cnt += 1




    def costCb(self,data):
        self.flagm = 1
        # rospy.loginfo('costCb')
        self.newcost = []
        self.costmap = data
        # self.grid = data

        width = self.costmap.info.width
        height = self.costmap.info.height
        pnts = self.costmap.data

        cnt = 0
        for x in range(height):
            newcost_x = []
            for y in range(width):
                newcost_x.append(pnts[cnt])
                cnt += 1
            self.newcost.append(newcost_x)

        self.init = 1



    def mapCb(self, data):
        # rospy.loginfo('mapCb')
        self.grid = data
        if self.flagm == 1:

            self.res = data.info.resolution
            self.originX = data.info.origin.position.x
            self.originY = data.info.origin.position.y
        self.flagm = 0
        # if self.init == 1:
            # Where the global map [0, 0] is located in the image matrix



            #
            # width = self.grid.info.width
            # height = self.grid.info.height
            # pnts = self.grid.data
            # gpnts = self.costmap.data
            # cnt = 0
            # pic_mat = []
            # for i in range(height):
            #     pic_x = []
            #     for j in range(width):
            #         p = pnts[cnt]
            #         g = gpnts[cnt]
            #         if (g > 50):
            #             p = 0
            #         else:
            #             if (p == -1):
            #                 p = 50
            #             elif (p == 0):
            #                 p = 100
            #             else:
            #                 p = 0
            #         pic_x.append(g)
            #         cnt += 1
            #     pic_mat.append(pic_x)




    def costUpCb(self, data):

        cost_update = data
        # If the current robot position in /odometry is not already in track_list, add it.
        if [self.locX, self.locY] not in self.track_list:
            self.track_list.append([self.locX, self.locY])

        if self.init == 1 and self.flagm == 0:
            # rospy.loginfo('costUpCb')
            gpnts = cost_update.data
            gwidth = cost_update.width
            gheight = cost_update.height
            gx = cost_update.x
            gy = cost_update.y
            cnt = 0
            for x in range(gheight):
                for y in range(gwidth):
                    g = gpnts[cnt]
                    self.newcost[x + gy][y + gx] = g
                    cnt += 1

            pic_mat = np.array(self.newcost).astype(np.uint8)
            self.showMap(pic_mat)


            # self.image = pic_mat
            # if self.image is not None:
            #     self.pub_image.publish(self.br.cv2_to_imgmsg(self.image))
            #


    ####################################################################################################
####################################################################################################
####################################################################################################
    #
    # def costUpCb(self,data):
    #     self.flagg = 1
    #     cost_update = data
    #
    #     if self.init == 1 and self.flagm == 0:
    #         rospy.loginfo('costUpCb')
    #         gpnts = cost_update.data
    #         gwidth = cost_update.width
    #         gheight = cost_update.height
    #         gx = cost_update.x
    #         gy = cost_update.y
    #         cnt = 0
    #         for x in range(gheight):
    #             for y in range(gwidth):
    #                 g = gpnts[cnt]
    #                 self.newcost[x+gy][y+gx] = g
    #                 cnt += 1
    #         self.flagm = 1
    #
    #
    # def costCb(self,data):
    #     rospy.loginfo('costCb')
    #     self.newcost = []
    #     self.flagm = 1
    #     self.costmap = data
    #
    #     width = self.costmap.info.width
    #     height = self.costmap.info.height
    #     pnts = self.costmap.data
    #     cnt = 0
    #
    #     for x in range(height):
    #         newcost_x = []
    #         for y in range(width):
    #             newcost_x.append(pnts[cnt])
    #             cnt+=1
    #         self.newcost.append(newcost_x)
    #
    #
    #
    # def mapCb(self, data):
    #     self.grid = data
    #
    #     if self.flagm == 1:
    #         self.init = 1
    #         rospy.loginfo('mapCb')
    #         width = self.grid.info.width
    #         height = self.grid.info.height
    #         pnts = self.grid.data
    #         gpnts = self.costmap.data
    #         cnt = 0
    #         pic_mat = []
    #
    #         for x in range(height):
    #             pic_x = []
    #             for y in range(width):
    #                 p = pnts[cnt]
    #                 g = gpnts[cnt]
    #
    #                 if (g > 50):
    #                     p = 0
    #                 else:
    #                     if (p == -1):
    #                         p = 50
    #                     elif (p == 0):
    #                         p = 100
    #                     else:
    #                         p = 0
    #                 pic_x.append(p)
    #                 cnt += 1
    #             pic_mat.append(pic_x)
    #
    #         pic_mat = np.array(pic_mat).astype(np.uint8)
    #         self.image = pic_mat
    #         if self.image is not None:
    #             self.pub_image.publish(self.br.cv2_to_imgmsg(self.image))
    #         self.flagm = 0

####################################################################################################
####################################################################################################
####################################################################################################


    def showMap(self, data):
        pic_mat = data
        endPoints = []

        # Convert to numpy array for HoughCircles to find potential wall ends
        pic_mat = np.array(pic_mat).astype(np.uint8)
        # Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
        accum_size = 1
        # Minimum distance between the centers of the detected circles.
        minDist = 40
        # First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
        param1 = 10
        # Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
        param2 = 15
        # Minimum circle radius
        minRadius = 10
        # Maximum circle radius
        maxRadius = 20

        endPoints = cv2.HoughCircles(pic_mat, cv2.HOUGH_GRADIENT, accum_size, minDist,
                                  param1=param1, param2=param2,
                                  minRadius=minRadius, maxRadius=maxRadius)
        # endPoints = cv2.HoughCircles(pic_mat, cv2.HOUGH_GRADIENT, 1, 40, param1=10, param2=15, minRadius=10, maxRadius=20)



        if endPoints.shape[1] is not None:
            for i in range(endPoints.shape[1]):
                c = endPoints[0, i, :]

                center = (np.round(c[0]), np.round(c[1]))
                radius = np.round(c[2])
                # if pic_mat[np.round(c[1]), np.round(c[0])] == 0:
                if np.linalg.norm(np.array([600., 600.]) - center) < 5000.:
                    cv2.circle(pic_mat, center, 3, (150, 0, 0), -1, 8, 0)
                    cv2.circle(pic_mat, center, radius, (150, 0, 0), 3, 8, 0)


        for i in range(len(self.track_list)-1):
            # draw a line of width == robot from last point to next.
            x = abs(int((self.track_list[i][0] - self.originX) / self.res))
            y = abs(int((self.track_list[i][1] - self.originY) / self.res))
            x1 = abs(int((self.track_list[i+1][0] - self.originX) / self.res))
            y1 = abs(int((self.track_list[i+1][1] - self.originY) / self.res))
            cv2.circle(pic_mat, (x, y), 30, (100, 0, 0), -1, 8, 0)
            cv2.line(pic_mat, (x, y), (x1, y1), (100, 0, 0), 40)


        self.nodes = []
        for i in range(len(endPoints[0])):
            for j in range(len(endPoints[0])):
                if (endPoints[0][i, 0] == endPoints[0][j, 0]) and (endPoints[0][i, 1] == endPoints[0][j, 1]):
                    pass
                else:
                    a = (endPoints[0][i, 0], endPoints[0][i, 1])
                    b = (endPoints[0][j, 0], endPoints[0][j, 1])

                    dist = distance.euclidean(a, b)
                    dist = dist / 2
                    ang = math.atan2((endPoints[0][j, 1] - endPoints[0][i, 1]), (endPoints[0][j, 0] - endPoints[0][i, 0]))
                    if dist < 100:
                        # This not working, implement quadrant identification?
                        x = int(round(endPoints[0][i, 0] + dist * math.cos(ang)))
                        y = int(round(endPoints[0][i, 1] + dist * math.sin(ang)))

                        if pic_mat[y, x] == 0:
                            self.nodes.append([x, y])
                            cv2.circle(pic_mat, (x, y), 3, (255, 0, 0), -1, 8, 0)

        # If the current goal is no longer an option, find a new goal.
        if self.goal not in self.nodes:
            self.result = 1

        # Display the map in RViz
        self.image = pic_mat
        if self.image is not None:
            self.pub_image.publish(self.br.cv2_to_imgmsg(self.image))

        min_dist = 1000000
        nextpnt = []
        for i in range(len(self.nodes)):
            x = abs(round((self.locX - self.originX) / self.res, 1))
            y = abs(round((self.locY - self.originY) / self.res, 1))
            a = (x, y)
            b = (self.nodes[i][0], self.nodes[i][1])
            dist = distance.euclidean(a, b)

            if dist < min_dist:
                nextX = round(self.nodes[i][0] * self.res + self.originX, 2)
                nextY = round(self.nodes[i][1] * self.res + self.originY, 2)
                nextXCart = self.nodes[i][0]
                nextYCart = self.nodes[i][1]
                min_dist = dist

            nextpnt = [nextX, nextY]
            # rospy.loginfo(pic_mat[nextpnt])


        # ((abs(round(self.locX, 2) - self.goalX) <= 0.5) and (abs(round(self.locY, 2) - self.goalY) <= 0.5))
        if (self.start == 1)\
            or (self.result == 1)\
            and (nextpnt):

            self.start = 0
            self.flagm = 0
            rospy.loginfo("I'm going")
            rospy.loginfo(nextpnt)
            self.goalSend(nextpnt)
            rospy.sleep(1)
            # cv2.circle(pic_mat, (self.goal), 5, (100, 0, 0), -1, 8, 0)
            self.goal = [nextXCart, nextYCart]
            self.result = 0

            # self.goalX = nextX
            # self.goalY = nextY



    def feedbackCb(self, data):
        pass

    def sendNavCb(self, data):
        result = data
        if result.status.text == "Goal reached.":
            rospy.loginfo(result.status.text)
            self.result = 1
        else:
            self.result = 0

        if result.status.text == "Failed":
            rospy.loginfo(result.status.text)
            self.result = 1
            self.taboo.append(self.goal)





    def goalSend(self, data):
        q = quaternion_from_euler(0, 0, 0, 'sxyz')
        newPose.header.stamp = rospy.Time.now()
        newPose.header.frame_id = "map"
        newPose.pose.position.x = data[0]
        newPose.pose.position.y = data[1]
        newPose.pose.orientation.x = q[0]
        newPose.pose.orientation.y = q[1]
        newPose.pose.orientation.z = q[2]
        newPose.pose.orientation.w = q[3]
        self.pub_goal.publish(newPose)


    def edgeDetection(self, data):
        pass
        # self.pub.publish(data)

    def GoalNodes(self, data):
        pass


if __name__ == "__main__":
    rospy.init_node('Frontier_Exploration')  # make node
    f = Frontier()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
