import rospy
from std_msgs.msg import String
import cv2
import numpy as np

def detect_red_object(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 100, 60])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    object_center = None

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > 500:  
            x, y, w, h = cv2.boundingRect(largest_contour)
            object_center = (x + w // 2, y + h // 2)

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, object_center, 5, (0, 0, 255), -1)

    return frame, object_center

rospy.init_node('object_tracking_gripper_node', anonymous=True)
pub = rospy.Publisher('movement_commands', String, queue_size=10)

cap = cv2.VideoCapture(4) 

object_detected = False
prev_center = None

while not rospy.is_shutdown() and cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Tidak dapat membaca frame dari kamera.")
        break

    height, width, _ = frame.shape

    cv2.line(frame, (width // 2, 0), (width // 2, height), (0, 255, 0), 1) 
    cv2.line(frame, (0, height // 2), (width, height // 2), (0, 255, 0), 1)  

    if not object_detected:
        frame, object_center = detect_red_object(frame)

        if object_center:
            object_detected = True
            prev_center = object_center
    else:
        frame, current_center = detect_red_object(frame)

        if current_center:
            dx = current_center[0] - width // 2
            dy = current_center[1] - height // 2

            if abs(dx) < 15 and abs(dy) < 15:  
                rospy.loginfo("Objek berada di tengah. Menggerakkan gripper...")
                pub.publish("turunkan_gripper")
                rospy.sleep(1)
                pub.publish("tutup_gripper")
                rospy.sleep(1)
                pub.publish("angkat_gripper")
                rospy.sleep(1)

                object_detected = False  
            else:
                if dx > 10:
                    pub.publish("gerak_kanan")
                elif dx < -10:
                    pub.publish("gerak_kiri")
                if dy > 10:
                    pub.publish("gerak_bawah")
                elif dy < -10:
                    pub.publish("gerak_atas")

            prev_center = current_center
        else:
            object_detected = False
            prev_center = None
    cv2.imshow("Deteksi Objek Merah", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
