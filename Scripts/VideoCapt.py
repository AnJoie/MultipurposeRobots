# import the necessary packages
import argparse
import math
import socket
import imutils
import time
import cv2
import sys

import numpy as np
from scipy.spatial.transform import Rotation

UDP_IP_ADDRESS = "127.0.0.1"
UDP_PORT_WRITE = 5555

cap = cv2.VideoCapture("http://localhost:8080")

def int_r(num):
    num = int(num + (0.5 if num > 0 else -0.5))
    return num

def setup():

    clientSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    return  clientSock


def send(clientSock,linear,angular):
    v = str(round(float(linear)))
    w = str(round(float(angular)))

    msg_str = v + ' ' + w

    clientSock.sendto(msg_str.encode('utf-8'), (UDP_IP_ADDRESS, UDP_PORT_WRITE))


def get_velocity(x,y):

    theta = 0.0
    if x < 0 and y < 0:
        theta = math.atan(y / x) - math.pi
    elif x < 0:
        theta = math.atan(y / x) + math.pi
    else:
        theta = math.atan(y/x)


    k_v = 0.5
    k_h = 4.0
    distance = math.sqrt(x ** 2 + y**2)
    linear_velocity = k_v * math.sqrt(x ** 2 + y**2) * (10+1/distance)
    angular_velocity = k_h * theta
    print("Theta: ", theta * 180 / math.pi)
    return linear_velocity,angular_velocity

def get_distance(x,y):
    return math.sqrt(x ** 2 + y**2)


def draw_borders(markerCorner,markerID,rvec,tvec):
    # loop over the detected ArUCo corners

        # extract the marker corners (which are always returned
        # in top-left, top-right, bottom-right, and bottom-left
        # order)

        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # draw the bounding box of the ArUCo detection
        if markerID == 0:
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the
        else:
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
        # ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

        cv2.circle(frame, (960, 540), 4, (0, 0, 255), -1)

        # draw the ArUco marker ID on the frame
        # cv2.putText(frame, str(markerID),
        #             (topLeft[0], topLeft[1] - 15),
        #             cv2.FONT_HERSHEY_SIMPLEX,
        #             0.5, (0, 255, 0), 2)

        # cv2.putText(frame, f"{newpose_x} {newpose_y}",
        #             (topLeft[0] - 30, topLeft[1] - 15),
        #             cv2.FONT_HERSHEY_SIMPLEX,
        #             0.5, (255, 255, 255), 2)
        # for i,rvec in enumerate(rvecs):

        # tvec = tvecs[i]
        cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.1)


is_first_frame = True

if __name__ == "__main__":

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    # initialize the video stream and allow the camera sensor to warm up
    print("[INFO] starting video stream...")

    cameraMatrix = np.array([[5212.1, 0, 960], [0, 5212.1, 540], [0, 0, 1]])
    distCoeffs = np.zeros([0, 0, 0, 0])
    markerLength=0.035

    prev_frame = 0
    current_frame = prev_frame

    counter = 0

    clientSock = setup()

    prev_theta = 0
    prev_distance = 0

    while True:


        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 1000 pixels
        # frame = imutils.resize(frame, width=1000)
        # detect ArUco markers in the input frame
        ret, frame = cap.read()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
                                                           arucoDict, parameters=arucoParams)
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)


        f = open('speed_py.txt', 'a')
        fx = open ('x_py.txt', 'a')
        fy = open('y_py.txt', 'a')

        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            try:
                robot_index = list(ids).index(0)
            except ValueError:
                print("robot not found")
                robot_index = 0

            rmat, _ = cv2.Rodrigues(rvecs[robot_index][0])

            translation = [tvecs[robot_index][0][0], tvecs[robot_index][0][1], 0]
            robot_rotation = rvecs[robot_index][0]

            if robot_index == 1:

                cube_rvec = rvecs[0][0]
                cube_tvec = tvecs[0][0]

                cube_x_est = cube_tvec[0]
                cube_y_est = cube_tvec[1]

                newpose_x, newpose_y, _ = rmat @ (np.array([cube_x_est, cube_y_est, 0]) - translation)

                v,w = get_velocity(newpose_x,newpose_y)

                print(np.linalg.norm(np.array([[newpose_x, newpose_y]])))
                send(clientSock,v,w)

                # prev_theta = theta
                # prev_distance = distance



            elif robot_index == 0 and len(ids) != 1:

                cube_rvec = rvecs[1][0]
                cube_tvec = tvecs[1][0]

                cube_x_est = cube_tvec[0]
                cube_y_est = cube_tvec[1]

                newpose_x, newpose_y, _ = rmat @ (np.array([cube_x_est, cube_y_est, 0]) - translation)

                v, w = get_velocity(newpose_x, newpose_y)
                print(np.linalg.norm(np.array([[newpose_x, newpose_y]])))
                send(clientSock, v, w)

                # prev_theta = theta
                # prev_distance = distance


            else:
                cube_rvec = rvecs[0][0]
                cube_tvec = tvecs[0][0]

                cube_x_est = cube_tvec[0]
                cube_y_est = cube_tvec[1]

                newpose_x, newpose_y, _ = rmat @ (np.array([cube_x_est, cube_y_est, 0]) - translation)

                print(np.linalg.norm(np.array([[newpose_x, newpose_y]])))
                # send(clientSock,prev_distance,prev_theta)




            # if not is_first_frame and (frame != prev_frame).any():
            #
            #
            #
            #         distnace = np.linalg.norm(np.array([[translation[0]], [translation[1]]]) - robot_location_prev)
            #         v = distnace * 10 / 0.03
            #         angle = np.linalg.norm(robot_rotation) - np.linalg.norm(robot_rotation_prev)
            #         w = angle / 0.03
            #         counter += 1
            #         fx.write(f"{translation[0]*10}" + '\n')
            #         fy.write(f"{translation[1]*10}" + '\n')



                # print (v)
                # print (w)




            aruco_numbers = [i for i in range(len(ids))]

            for (corner,markerID,aruco_number) in zip(corners,ids,aruco_numbers):

                rvec = rvecs[aruco_number][0]
                tvec = tvecs[aruco_number][0]

                draw_borders(corner,markerID,rvec,tvec)

                # show the output frame
            cv2.imshow("Frame", frame)



        # do a bit of cleanup

        else:
            cv2.imshow("Frame", frame)


        if not is_first_frame:
            prev_frame = frame


        # robot_rotation_prev = robot_rotation
        # robot_location_prev = np.array([[translation[0]], [translation[1]]])

        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == 27:
            break

        is_first_frame = False



    cv2.destroyAllWindows()
    f.close()

