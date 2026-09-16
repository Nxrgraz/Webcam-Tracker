import cv2
import os
import urllib.request
import shutil

# ============================================================
# SETTINGS
# ============================================================

INPUT_FOLDER = "raw_images"

OUTPUT_IMAGE_FOLDER = "dataset/images/all"
OUTPUT_LABEL_FOLDER = "dataset/labels/all"
PREVIEW_FOLDER = "label_previews"

MODEL_FILE = "face_detection_yunet_2023mar.onnx"

MODEL_URL = (
    "https://github.com/opencv/opencv_zoo/raw/main/"
    "models/face_detection_yunet/"
    "face_detection_yunet_2023mar.onnx"
)

CLASS_ID = 0

# Ignore extremely weak face detections.
CONFIDENCE_THRESHOLD = 0.75

# If multiple faces somehow appear, use the largest one.
USE_LARGEST_FACE_ONLY = True


# ============================================================
# CREATE FOLDERS
# ============================================================

os.makedirs(OUTPUT_IMAGE_FOLDER, exist_ok=True)
os.makedirs(OUTPUT_LABEL_FOLDER, exist_ok=True)
os.makedirs(PREVIEW_FOLDER, exist_ok=True)


# ============================================================
# DOWNLOAD YUNET FACE DETECTOR IF NEEDED
# ============================================================

if not os.path.exists(MODEL_FILE):
    print("YuNet face detector not found.")
    print("Downloading model...")

    try:
        urllib.request.urlretrieve(
            MODEL_URL,
            MODEL_FILE
        )

        print("Model downloaded.")
    except Exception as error:
        print("ERROR downloading model:")
        print(error)
        exit()


# ============================================================
# CREATE FACE DETECTOR
# ============================================================

detector = cv2.FaceDetectorYN.create(
    MODEL_FILE,
    "",
    (320, 320),
    CONFIDENCE_THRESHOLD,
    0.3,
    5000
)


# ============================================================
# HELPER FUNCTION
# CONVERT NORMAL BOX TO YOLO BOX
# ============================================================

def convert_to_yolo(
    x,
    y,
    width,
    height,
    image_width,
    image_height
):
    center_x = (
        x + width / 2.0
    ) / image_width

    center_y = (
        y + height / 2.0
    ) / image_height

    normalized_width = (
        width / image_width
    )

    normalized_height = (
        height / image_height
    )

    center_x = max(
        0.0,
        min(1.0, center_x)
    )

    center_y = max(
        0.0,
        min(1.0, center_y)
    )

    normalized_width = max(
        0.0,
        min(1.0, normalized_width)
    )

    normalized_height = max(
        0.0,
        min(1.0, normalized_height)
    )

    return (
        center_x,
        center_y,
        normalized_width,
        normalized_height
    )


# ============================================================
# GET INPUT IMAGES
# ============================================================

valid_extensions = (
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp"
)

image_files = [
    file
    for file in os.listdir(INPUT_FOLDER)
    if file.lower().endswith(valid_extensions)
]

image_files.sort()


if len(image_files) == 0:
    print()
    print("ERROR: No images found.")
    print()
    print(
        f"Put your photos inside: {INPUT_FOLDER}"
    )
    exit()


print()
print("========================================")
print("AUTOMATIC FACE LABELING")
print("========================================")
print()
print(
    f"Images found: {len(image_files)}"
)
print()


successful = 0
failed = 0
multiple_faces = 0


# ============================================================
# PROCESS EVERY IMAGE
# ============================================================

for image_number, filename in enumerate(
    image_files,
    start=1
):
    input_path = os.path.join(
        INPUT_FOLDER,
        filename
    )

    image = cv2.imread(
        input_path
    )

    if image is None:
        print(
            f"[{image_number}/{len(image_files)}] "
            f"Could not read {filename}"
        )

        failed += 1
        continue


    image_height, image_width = (
        image.shape[:2]
    )


    # --------------------------------------------------------
    # Tell YuNet current image size
    # --------------------------------------------------------

    detector.setInputSize(
        (
            image_width,
            image_height
        )
    )


    # --------------------------------------------------------
    # DETECT FACES
    # --------------------------------------------------------

    result, faces = detector.detect(
        image
    )


    if faces is None or len(faces) == 0:
        print(
            f"[{image_number}/{len(image_files)}] "
            f"NO FACE: {filename}"
        )

        failed += 1
        continue


    # --------------------------------------------------------
    # IF THERE ARE MULTIPLE FACES
    # --------------------------------------------------------

    if len(faces) > 1:
        multiple_faces += 1

        print(
            f"[{image_number}/{len(image_files)}] "
            f"WARNING: {len(faces)} faces found in "
            f"{filename}"
        )


    # --------------------------------------------------------
    # PICK LARGEST FACE
    # --------------------------------------------------------

    if USE_LARGEST_FACE_ONLY:
        largest_face = None
        largest_area = 0

        for face in faces:
            x = int(face[0])
            y = int(face[1])

            width = int(face[2])
            height = int(face[3])

            area = (
                width *
                height
            )

            if area > largest_area:
                largest_area = area
                largest_face = face

        selected_faces = [
            largest_face
        ]

    else:
        selected_faces = faces


    # --------------------------------------------------------
    # CREATE YOLO LABEL FILE
    # --------------------------------------------------------

    base_name = os.path.splitext(
        filename
    )[0]

    label_path = os.path.join(
        OUTPUT_LABEL_FOLDER,
        base_name + ".txt"
    )


    preview = image.copy()


    with open(
        label_path,
        "w"
    ) as label_file:

        for face in selected_faces:

            x = int(
                face[0]
            )

            y = int(
                face[1]
            )

            width = int(
                face[2]
            )

            height = int(
                face[3]
            )

            confidence = float(
                face[-1]
            )


            # -----------------------------------------------
            # KEEP BOX INSIDE IMAGE
            # -----------------------------------------------

            x = max(
                0,
                x
            )

            y = max(
                0,
                y
            )

            width = min(
                width,
                image_width - x
            )

            height = min(
                height,
                image_height - y
            )


            # -----------------------------------------------
            # CONVERT TO YOLO FORMAT
            # -----------------------------------------------

            (
                center_x,
                center_y,
                normalized_width,
                normalized_height
            ) = convert_to_yolo(
                x,
                y,
                width,
                height,
                image_width,
                image_height
            )


            label_file.write(
                f"{CLASS_ID} "
                f"{center_x:.6f} "
                f"{center_y:.6f} "
                f"{normalized_width:.6f} "
                f"{normalized_height:.6f}\n"
            )


            # -----------------------------------------------
            # DRAW PREVIEW BOX
            # -----------------------------------------------

            cv2.rectangle(
                preview,
                (x, y),
                (
                    x + width,
                    y + height
                ),
                (0, 255, 0),
                2
            )

            cv2.putText(
                preview,
                f"jerison_face {confidence:.2f}",
                (
                    x,
                    max(
                        y - 10,
                        20
                    )
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )


    # --------------------------------------------------------
    # COPY ORIGINAL IMAGE
    # --------------------------------------------------------

    output_image_path = os.path.join(
        OUTPUT_IMAGE_FOLDER,
        filename
    )

    shutil.copy2(
        input_path,
        output_image_path
    )


    # --------------------------------------------------------
    # SAVE PREVIEW
    # --------------------------------------------------------

    preview_path = os.path.join(
        PREVIEW_FOLDER,
        filename
    )

    cv2.imwrite(
        preview_path,
        preview
    )


    successful += 1

    print(
        f"[{image_number}/{len(image_files)}] "
        f"LABELED: {filename}"
    )


# ============================================================
# FINISHED
# ============================================================

print()
print("========================================")
print("AUTOMATIC LABELING FINISHED")
print("========================================")
print()

print(
    f"Successfully labeled: {successful}"
)

print(
    f"No face / failed:     {failed}"
)

print(
    f"Multiple-face images: {multiple_faces}"
)

print()

print(
    "Images:"
)

print(
    OUTPUT_IMAGE_FOLDER
)

print()

print(
    "Labels:"
)

print(
    OUTPUT_LABEL_FOLDER
)

print()

print(
    "Box previews:"
)

print(
    PREVIEW_FOLDER
)

print()

print(
    "IMPORTANT:"
)

print(
    "Open label_previews and quickly check "
    "that the green boxes are correct before training."
)
