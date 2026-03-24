import cv2
import cv2.aruco as aruco
import numpy as np
import serial
import time
import matplotlib.pyplot as plt


def adjust_gamma(image, gamma=0.5):
    # Darkens the image slightly so ArUco markers are easier to detect under bright lights
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)]).astype("uint8")
    return cv2.LUT(image, table)


def get_marker_center(frame, aruco_dict):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = adjust_gamma(gray, gamma=0.5)

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    if ids is not None:
        for i in range(len(ids)):
            cx = int(np.mean(corners[i][0][:, 0]))
            cy = int(np.mean(corners[i][0][:, 1]))
            return cx, cy, corners, ids

    return None, None, None, None


# ---- Camera setup ----
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FPS, 100)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
cap.set(cv2.CAP_PROP_EXPOSURE, -9)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

if not cap.isOpened():
    raise Exception("Camera not found. Check the index in VideoCapture(0).")

# ---- Serial setup ----
ser = serial.Serial('COM4', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to finish rebooting after serial connect

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

frame_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_center = (frame_width // 2, frame_height // 2)

# ---- State variables ----
calibrated      = False
calibrated_cx   = 0
cx_calibration  = []   # stores cx readings during the 300-frame calibration phase
cx_live         = []   # stores cx readings after calibration for the final plot
marker_flag     = 0    # 0 = send full setup data once, 1 = send only cx from now on
CALIBRATION_FRAMES = 300


while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        continue

    # Draw a red dot at the image center so we can see the reference point
    cv2.circle(frame, frame_center, 5, (0, 0, 255), -1)

    cx, cy, corners, ids = get_marker_center(frame, aruco_dict)

    if ids is not None and cx is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)  # Blue dot on the marker

        if not calibrated:
            # Collect 300 readings to find a stable center reference
            cx_calibration.append(cx)
            print(f"Calibrating... {len(cx_calibration)}/{CALIBRATION_FRAMES}  cx={cx}")

            if len(cx_calibration) >= CALIBRATION_FRAMES:
                calibrated_cx = int(np.mean(cx_calibration))
                calibrated    = True
                print(f"Calibration done. Center locked at cx = {calibrated_cx}")

                # Show a plot of the calibration readings so you can judge stability
                plt.figure("Calibration")
                plt.plot(cx_calibration, color='blue')
                plt.axhline(calibrated_cx, color='red', linestyle='--', label=f"Mean = {calibrated_cx}")
                plt.title("Calibration — marker cx over 300 frames")
                plt.xlabel("Frame")
                plt.ylabel("cx (pixels)")
                plt.legend()
                plt.grid()
                plt.show()

                # Send the one-time setup packet to Arduino (flag=0)
                msg = f"0 {frame_width} {calibrated_cx} {cx}\n"
                ser.write(msg.encode('utf-8'))
                marker_flag = 1

        else:
            # Normal tracking — send only the current marker position
            msg = f"1 0 0 {cx}\n"
            ser.write(msg.encode('utf-8'))
            cx_live.append(cx)

    cv2.imshow("Anti-Sway Vision", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# ---- Cleanup ----
cap.release()
cv2.destroyAllWindows()
ser.close()

if cx_live:
    plt.figure("Live Tracking")
    plt.plot(cx_live, color='green', marker='x')
    plt.title("Marker cx after calibration (live tracking)")
    plt.xlabel("Frame")
    plt.ylabel("cx (pixels)")
    plt.grid()
    plt.show()
