import cv2
import os
import time

SAVE_FOLDER = "raw_images"
CAMERA_INDEX = 1

os.makedirs(SAVE_FOLDER, exist_ok=True)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Could not open webcam.")
    exit()

image_number = len(
    [
        file for file in os.listdir(SAVE_FOLDER)
        if file.lower().endswith((".jpg", ".png", ".jpeg"))
    ]
)

print()
print("Face Dataset Capture")
print("--------------------")
print("SPACE = save image")
print("A     = automatic capture")
print("Q     = quit")
print()

automatic_capture = False
last_capture_time = 0

AUTO_CAPTURE_INTERVAL = 0.5

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to read webcam.")
        break

    display = frame.copy()

    mode_text = (
        "AUTO CAPTURE"
        if automatic_capture
        else "MANUAL CAPTURE"
    )

    cv2.putText(
        display,
        mode_text,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    cv2.putText(
        display,
        f"Images: {image_number}",
        (20, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    cv2.putText(
        display,
        "SPACE save | A auto | Q quit",
        (20, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 0),
        2
    )

    cv2.imshow(
        "Jerison Training Image Capture",
        display
    )

    current_time = time.time()

    if (
        automatic_capture and
        current_time - last_capture_time >=
        AUTO_CAPTURE_INTERVAL
    ):
        filename = os.path.join(
            SAVE_FOLDER,
            f"jerison_{image_number:05d}.jpg"
        )

        cv2.imwrite(filename, frame)

        print(f"Saved {filename}")

        image_number += 1
        last_capture_time = current_time

    key = cv2.waitKey(1) & 0xFF

    if key == ord(" "):
        filename = os.path.join(
            SAVE_FOLDER,
            f"jerison_{image_number:05d}.jpg"
        )

        cv2.imwrite(filename, frame)

        print(f"Saved {filename}")

        image_number += 1

    elif key == ord("a") or key == ord("A"):
        automatic_capture = not automatic_capture

        print(
            "Automatic capture:",
            "ON" if automatic_capture else "OFF"
        )

        last_capture_time = time.time()

    elif key == ord("q") or key == ord("Q"):
        break

cap.release()
cv2.destroyAllWindows()

print()
print(f"Finished. Total images: {image_number}")