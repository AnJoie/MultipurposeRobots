import cv2
import numpy as np
import os
import glob
import math

from scipy.spatial.transform import Rotation
def draw_borders(markerCorner,markerID,rvec,tvec,frame):
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
if __name__ == "__main__":
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    # initialize the video stream and allow the camera sensor to warm up

    cameraMatrix = np.array([[1.53692614e+03, 0.00000000e+00, 9.60766918e+02],
                             [0.00000000e+00, 1.53697638e+03, 5.38382719e+02],
                             [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    distCoeffs = np.array([[2.24131975e-03, 1.48727621e-02, -1.91783238e-04, 1.60971006e-05,
                            4.84568385e-03]])
    markerLength = 0.035
    # Извлечение пути отдельного изображения, хранящегося в данном каталоге
    images = glob.glob('data/img/Test/*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img,
                                                           arucoDict, parameters=arucoParams)
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)
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

            r_robot = Rotation.from_rotvec(robot_rotation)
            angles_robot = r_robot.as_euler('zxy')
            phi_est_robot = np.pi + angles_robot[0] - 0.01

            if robot_index == 1:

                cube_rvec = rvecs[0][0]
                cube_tvec = tvecs[0][0]

                cube_x_est = cube_tvec[0]
                cube_y_est = cube_tvec[1]

                newpose_x, newpose_y, _ = rmat @ (np.array([cube_x_est, cube_y_est, 0]) - translation)


                theta = 0.0
                if newpose_x < 0 and newpose_y < 0:
                    theta = (math.atan(newpose_y / newpose_x) - math.pi)
                elif newpose_x < 0:
                    theta = (math.atan(newpose_y / newpose_x) + math.pi)
                else:
                    theta = math.atan(newpose_y / newpose_x)

                r_cube = Rotation.from_rotvec(cube_rvec)
                angles_cube = r_cube.as_euler('zxy')
                phi_est_cube = np.pi + angles_cube[0] - 0.01

                print("Theta_local: ", theta * 180 / math.pi)
                # print("Theta_global: ", (phi_est_cube - phi_est_robot)*180/np.pi)
                # print(newpose_x, newpose_y)
                print(cube_x_est, cube_y_est)



            elif robot_index == 0 and len(ids) != 1:

                cube_rvec = rvecs[1][0]
                cube_tvec = tvecs[1][0]

                cube_x_est = cube_tvec[0]
                cube_y_est = cube_tvec[1]

                r_cube = Rotation.from_rotvec(cube_rvec)
                angles_cube = r_cube.as_euler('zxy')
                phi_est_cube = np.pi + angles_cube[0] - 0.01
                newpose_x, newpose_y, _ = rmat @ (np.array([cube_x_est, cube_y_est, 0]) - translation)

                theta = 0.0
                if newpose_x < 0 and newpose_y < 0:
                    theta = (math.atan(newpose_y / newpose_x) - math.pi)
                elif newpose_x < 0:
                    theta = (math.atan(newpose_y / newpose_x) + math.pi)
                else:
                    theta = math.atan(newpose_y / newpose_x)

                print("Theta_local: ", theta * 180 / math.pi)
                #
                # print("Theta_global: ", (phi_est_cube - phi_est_robot)*180/np.pi)
                # print(newpose_x, newpose_y)
                print(cube_x_est, cube_y_est)


            else:
                cube_rvec = rvecs[0][0]
                cube_tvec = tvecs[0][0]

                cube_x_est = cube_tvec[0]
                cube_y_est = cube_tvec[1]

                newpose_x, newpose_y, _ = rmat @ (np.array([cube_x_est, cube_y_est, 0]) - translation)

                print(np.linalg.norm(np.array([[newpose_x, newpose_y]])))

        aruco_numbers = [i for i in range(len(ids))]

        for (corner, markerID, aruco_number) in zip(corners, ids, aruco_numbers):
            rvec = rvecs[aruco_number][0]
            tvec = tvecs[aruco_number][0]

            draw_borders(corner, markerID, rvec, tvec,img)
        cv2.imshow('img', img)
        cv2.waitKey(0)

    cv2.destroyAllWindows()
