import numpy as np
import cv2
import cv2.aruco as aruco

# Select type of aruco marker (size)
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

# Create an image from the marker
# second param is ID number
# last param is total image size
for x in list(range(101,131)):
    img = aruco.drawMarker(aruco_dict, x, 800)
    cv2.imwrite(str(x)+".jpg", img)
    
# Display the image to us
cv2.imshow('frame', img)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()
