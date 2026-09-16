from pathlib import Path
import random
import shutil

SOURCE_IMAGES = Path("dataset/images/all")
SOURCE_LABELS = Path("dataset/labels/all")

TRAIN_IMAGES = Path("dataset/images/train")
VAL_IMAGES = Path("dataset/images/val")

TRAIN_LABELS = Path("dataset/labels/train")
VAL_LABELS = Path("dataset/labels/val")

TRAIN_RATIO = 0.80
RANDOM_SEED = 42

IMAGE_EXTENSIONS = {
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp"
}

print()
print("======================================")
print("PREPARING YOLO DATASET")
print("======================================")
print()

for folder in [
    TRAIN_IMAGES,
    VAL_IMAGES,
    TRAIN_LABELS,
    VAL_LABELS
]:
    if folder.exists():
        shutil.rmtree(folder)

    folder.mkdir(
        parents=True,
        exist_ok=True
    )

images = [
    image
    for image in SOURCE_IMAGES.iterdir()
    if image.suffix.lower() in IMAGE_EXTENSIONS
]

images.sort()

valid_pairs = []

for image in images:
    label = (
        SOURCE_LABELS /
        f"{image.stem}.txt"
    )

    if not label.exists():
        print(
            f"WARNING: Missing label for {image.name}"
        )
        continue

    valid_pairs.append(
        (image, label)
    )

print(
    f"Valid image/label pairs: {len(valid_pairs)}"
)

if len(valid_pairs) == 0:
    print("ERROR: No valid training data found.")
    exit()

random.seed(RANDOM_SEED)
random.shuffle(valid_pairs)

train_count = int(
    len(valid_pairs) *
    TRAIN_RATIO
)

train_pairs = (
    valid_pairs[:train_count]
)

val_pairs = (
    valid_pairs[train_count:]
)

for image, label in train_pairs:
    shutil.copy2(
        image,
        TRAIN_IMAGES / image.name
    )

    shutil.copy2(
        label,
        TRAIN_LABELS / label.name
    )

for image, label in val_pairs:
    shutil.copy2(
        image,
        VAL_IMAGES / image.name
    )

    shutil.copy2(
        label,
        VAL_LABELS / label.name
    )

print()
print(
    f"Training images: {len(train_pairs)}"
)

print(
    f"Validation images: {len(val_pairs)}"
)

yaml_contents = """path: dataset
train: images/train
val: images/val

names:
  0: jerison_face
"""

with open(
    "data_v2.yaml",
    "w"
) as file:
    file.write(
        yaml_contents
    )

print()
print("Created data_v2.yaml")
print()
print("======================================")
print("DATASET READY")
print("======================================")