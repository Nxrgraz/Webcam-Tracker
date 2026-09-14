# Robotic Webcam Face Tracker

A 2-axis robotic webcam platform that detects a face in real time and physically pans and tilts a Logitech C270 to keep the target centered.

The project combines **C++**, **OpenCV DNN**, a custom **YOLO ONNX model**, **Arduino**, **serial communication**, **SG90 servos**, and a custom **3D-printed pan-tilt mechanism** designed in CAD.

## What it does

- Captures live video from an external webcam with OpenCV
- Runs a custom YOLO model for face detection
- Tracks the detected face center relative to the center of the frame
- Smooths noisy detections before commanding the servos
- Uses proportional control, a dead zone, and limited servo step size to reduce jitter
- Sends pan and tilt commands from the PC to an Arduino over serial at 115200 baud
- Supports both automatic tracking and manual keyboard control
- Includes the latest Revision M mechanical design and 3D-printable parts

## System overview

```text
Logitech C270
     |
     v
OpenCV video capture
     |
     v
YOLO face detection
     |
     v
Smoothed face position
     |
     v
Pan/Tilt controller
     |
     v
USB serial
     |
     v
Arduino Uno
   /     \
  v       v
Pan SG90  Tilt SG90
     \   /
      v v
3D-printed pan-tilt mount
```

## Hardware

- Arduino Uno
- Logitech C270 webcam
- 2x SG90 micro servos
- 3D-printed Revision M pan-tilt assembly
- USB connection for the webcam
- USB connection for the Arduino
- Regulated 5 V servo power supply recommended for reliable operation

### Servo wiring

| Function | Arduino pin |
| --- | --- |
| Pan servo signal | D9 |
| Tilt servo signal | D10 |
| Servo ground | Common GND |
| Servo power | 5 V regulated supply |

The Arduino ground and external servo-power ground must be connected together.

## Software

### PC tracker

`Camera.cpp` handles:

- Camera capture through OpenCV
- Letterboxing to 640 x 640 for YOLO inference
- ONNX inference through `cv::dnn`
- Confidence filtering and non-maximum suppression
- Exponential smoothing of the detected face position
- Pan and tilt control
- Serial communication with the Arduino
- Automatic and manual operating modes

Current controller values are tuned for smoother movement:

```text
Confidence threshold: 0.40
Dead zone:            50 px
Pan Kp:               0.015
Tilt Kp:              0.015
Maximum servo step:   2 degrees
Smoothing alpha:      0.15
Servo update period:  50 ms
```

### Arduino controller

`Arduino_Camera_Tracker.ino` receives commands in this format:

```text
panAngle,tiltAngle
```

For example:

```text
92,88
```

The Arduino constrains each command to 0-180 degrees and writes the requested positions to the pan and tilt servos.

## Manual controls

The tracker starts in automatic mode.

| Key | Action |
| --- | --- |
| `M` | Toggle automatic/manual mode |
| `W` / `S` | Tilt control |
| `A` / `D` | Pan control |
| `R` | Return pan and tilt to 90 degrees |
| `Q` or `Esc` | Quit |

Click the OpenCV camera window before using the keyboard controls so that it has focus.

## Face model

The tracker is designed to use a custom single-person YOLO model exported to ONNX.

The personal trained model is intentionally **not committed to this public repository**. By default, `Camera.cpp` expects a local file named:

```text
Jerison_face.onnx
```

Place the model beside the executable or change the model path in `Camera.cpp`.

The repository may also contain a base YOLO ONNX file used during development, but the personalized tracker uses the custom model above.

## Mechanical design

The current mechanical design is **Revision M**, located in:

```text
C270_Robotic_Arm_RevM/
```

Revision M includes:

- A thicker 10 mm arm to reduce sideways flex
- Direct access to the original servo shaft screws
- A 27 x 15 mm retaining-cap opening
- A side-mounted Logitech C270 cradle so the upper servo produces real camera tilt
- Separate pan and tilt axes
- Printable STL files and STEP assemblies
- Assembly, wiring, motion, and engineering-check documentation

The base should be secured to a rigid work surface for reliable operation. The CAD package also documents expected servo travel, mechanical assumptions, and assembly details.

## Repository structure

```text
Webcam-Tracker/
|
|-- Camera.cpp
|   Main C++ vision and tracking application
|
|-- Arduino_Camera_Tracker.ino
|   Arduino servo controller
|
|-- C270_Robotic_Arm_RevM/
|   Latest CAD, STL, STEP, assembly, and documentation files
|
|-- yolov8n.onnx
|   Base YOLO model used during development
|
|-- .gitignore
|   Excludes build output, training data, and the personal face model
|
`-- README.md
```

## Running the project

1. Connect the Arduino Uno by USB.
2. Upload `Arduino_Camera_Tracker.ino` using the Arduino IDE.
3. Connect the Logitech webcam to another USB port.
4. Confirm the Arduino COM port in Windows Device Manager or the Arduino IDE.
5. Update the COM port in `Camera.cpp` if necessary. The current source is configured for `COM7`.
6. Confirm the external webcam index in `Camera.cpp`. The current source uses camera index `1`.
7. Place the custom `Jerison_face.onnx` model in the program's working directory.
8. Build the C++ application with OpenCV and the serial library linked.
9. Run the tracker and click the camera window to use keyboard controls.

## Dependencies

- C++17-compatible compiler
- OpenCV with DNN support
- Arduino Servo library
- `serial` C++ library for PC-to-Arduino communication
- Arduino IDE for uploading the microcontroller firmware

## Current development status

The project has working camera capture, YOLO inference, serial communication, pan/tilt servo control, manual control, automatic tracking, smoothing, and a completed Revision M mechanical design.

The main remaining work is control tuning and hardware refinement, particularly improving tracking smoothness, reducing mechanical jitter/backlash, and validating the final pan/tilt limits under the actual webcam load.

## Why I built it

I built this project to combine computer vision, embedded control, mechanical design, and real-time hardware integration in one system. It gave me hands-on experience moving from a trained vision model to a physical closed-loop robotic platform, including debugging camera selection, serial communication, servo behavior, mechanical stiffness, and tracking stability.
