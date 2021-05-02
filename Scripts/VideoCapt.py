import cv2
import urllib.request
import numpy as np

cap = cv2.VideoCapture("http://localhost:8080")


if __name__ == "__main__":
    while True:
        ret, frame = cap.read()
        cv2.imshow('Video', frame)

        if cv2.waitKey(1) == 27:

            exit(0)
    cap.release()
    cv2.destroyAllWindows()

    # while (True):
    #     ret, frame = cap.read()
    #     cv2.imshow('frame', frame)
    #
    #     # the 'q' button is set as the
    #     # quitting button you may use any
    #     # desired button of your choice
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    # cap.release()
    # cv2.destroyAllWindows()


    # stream = urllib.request.urlopen('http://localhost:8080')
    # bytes = bytes()
    # while True:
    #     bytes += stream.read(1024)
    #     a = bytes.find(b'\xff\xd8')
    #     b = bytes.find(b'\xff\xd9')
    #     if a != -1 and b != -1:
    #         jpg = bytes[a:b + 2]
    #         bytes = bytes[b + 2:]
    #         i = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
    #         cv2.imshow('i', i)
    #         if cv2.waitKey(1) == 27:
    #             exit(0)