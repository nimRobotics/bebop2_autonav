import cv2
cam = cv2.VideoCapture('tcp://192.168.42.1:21')
running=True
while running:
    ret,frame = cam.read()
    if ret:
        cv2.imwrite("frame.png",frame)

    else:
        print("error")
cam.release()
cv2.destroyAllWindows()
