import cv2
import os
from natsort import natsorted


image_folder = "/home/vassil/TU_Delft/Internship/catkin_ws/src/pinocchio_test/video_IK"
video_name = 'video.avi'

images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
images = natsorted(images)[100::5]
# print(images)
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape
print(height,width)

video = cv2.VideoWriter(video_name, 0, 15, (width,height))

for image in images:
    video.write(cv2.imread(os.path.join(image_folder, image)))

cv2.destroyAllWindows()
video.release()