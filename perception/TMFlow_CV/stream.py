import cv2

stream_url = "rtsp://<robot-ip>/stream1"   # or http://<robot-ip>/mjpeg

cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Cannot open stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow("TM5 Camera", frame)
    if cv2.waitKey(1) == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()