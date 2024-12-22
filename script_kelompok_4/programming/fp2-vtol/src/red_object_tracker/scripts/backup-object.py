#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import cv2
import numpy as np

# Inisialisasi node ROS
rospy.init_node('object_detection_node', anonymous=True)
pub = rospy.Publisher('movement_commands', String, queue_size=10)

# Fungsi untuk mendeteksi objek merah
def detect_red_object(frame):
    # Ubah frame ke HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Rentang warna merah dalam HSV
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    # Masking untuk rentang warna merah
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Gabungkan dua rentang merah
    mask = mask1 + mask2
    height, width, _ = frame.shape
    offset_line = 140

    # Garis bantu hijau pada sumbu X dan Y
    cv2.line(frame, (width // 2, 0), (width // 2, height), (0, 255, 0), 1)  # Garis vertikal
    cv2.line(frame, (0, height // 2 + offset_line), (width, height // 2 + offset_line), (0, 255, 0), 1)  # Garis horizontal


    # Temukan kontur objek merah
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Pilih kontur terbesar
        largest_contour = max(contours, key=cv2.contourArea)
        (x, y, w, h) = cv2.boundingRect(largest_contour)

        # Hitung titik tengah objek
        center_x = x + w // 2
        center_y = y + h // 2 

        # Gambarkan persegi panjang dan titik tengah
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

        return center_x, center_y, True

    return None, None, False

# Inisialisasi kamera
cap = cv2.VideoCapture(4)  # Pilih indeks kamera sesuai kebutuhan
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

initial_command = "gerak bawah"
rospy.loginfo(f"Sending initial command: {initial_command}")
pub.publish(initial_command)


# Variabel untuk melacak status objek
object_detected = False
gripper_closed = False
offset_camera = 140

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logerr("Tidak dapat membaca frame dari kamera.")
        break

    center_x, center_y, detected = detect_red_object(frame)
    if detected:
        object_detected = True  # Objek terdeteksi, ubah status
        camera_center_y = frame_height // 2 + offset_camera

        if abs(center_x - frame_width // 2) < 20 and abs(center_y - camera_center_y) < 20 and not gripper_closed:
             if not gripper_closed:
                rospy.loginfo("Object is centered. Sending command: tutup gripper")
                pub.publish("tutup gripper")
                gripper_closed = True  # Tandai bahwa gripper sudah ditutup
                rospy.sleep(10)
                pub.publish("buka gripper")
    else:
        if not object_detected:
            # Jika objek belum terdeteksi, tetap kirim "gerak bawah"
            rospy.loginfo(f"Sending command: {initial_command}")
            pub.publish(initial_command)

    if detected:
        # Hitung pergeseran dari titik tengah kamera
        offset_x = center_x - frame_width // 2
        offset_y = center_y - camera_center_y 

        # Tentukan arah pergerakan untuk sumbu X dan Y
        direction_x = None
        direction_y = None

        if offset_x > 20:
            direction_x = "gerak kiri"
        elif offset_x < -20:
            direction_x = "gerak kanan"

        if offset_y > 20:
            direction_y = "gerak atas"
        elif offset_y < -20:
            direction_y = "gerak bawah"
        elif offset_y == 0:
            direction_y = "tetap y"
        elif offset_x == 0:
            direction_x = "tetap x"


        # Kirim perintah ke Node 2
        if direction_x or direction_y:
            command = f"{direction_x or 'tetap x'} {direction_y or 'tetap y'}"
            rospy.loginfo(f"Detected object. Sending command: {command}")
            pub.publish(command)

    # Tampilkan frame
    cv2.imshow("Red Object Detection", frame)

    # Tekan 'q' untuk keluar
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
