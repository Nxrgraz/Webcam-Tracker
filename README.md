# AI Pan Tilt Face Tracker

A real time two axis vision tracking system that detects and follows a specific face using a custom YOLO model, C++, OpenCV, model predictive control, and Arduino controlled pan and tilt servos.

The system combines computer vision, target motion estimation, feedback control, and embedded hardware to keep a detected face near the center of the camera frame while reducing overshoot, jitter, and unnecessary servo movement.

## Overview

A webcam continuously captures video while a custom YOLO ONNX model detects the target face.

The software measures the target's position relative to the center of the image and estimates its movement across the frame. Independent predictive controllers for the pan and tilt axes then determine how the camera should move.

Instead of reacting only to the current image error, the controller predicts how the target and camera are expected to move over the next several control steps.

The resulting commands are transmitted over serial to an Arduino controlling the two servo motors.

## System Pipeline

```text
Webcam
   |
   v
Camera Frame
   |
   v
Image Preprocessing and Letterboxing
   |
   v
Custom YOLO Face Detector
   |
   v
Non Maximum Suppression
   |
   v
Target Selection and Tracking
   |
   v
Face Position Smoothing
   |
   v
Image Error Calculation
   |
   v
Target Motion Observer
   |
   +----------------------+
   |                      |
   v                      v
Target Velocity      Servo Motion Model
   |                      |
   +-----------+----------+
               |
               v
       Future Motion Prediction
               |
        +------+------+
        |             |
        v             v
     Pan MPC       Tilt MPC
        |             |
        +------+------+
               |
               v
     Velocity and Acceleration
             Limits
               |
               v
       Servo Angle Commands
               |
               v
            Arduino
          /         \
         v           v
    Pan Servo    Tilt Servo
```

## Features

### Custom Face Detection

The tracker uses a custom YOLO model exported to ONNX and executed through OpenCV DNN.

The detector runs on a letterboxed 640 × 640 image while preserving the original camera aspect ratio.

Detections below the confidence threshold are rejected and Non Maximum Suppression is used to remove overlapping detections.

### Target Association

When tracking begins, the highest confidence detection is selected.

Once a target has been established, subsequent detections are compared with the previous smoothed target position. The closest detection is selected to reduce unnecessary switching between detected objects.

### Target Position Smoothing

Raw bounding box positions from the detector can change slightly even when the target is stationary.

An exponential smoothing filter is applied to the detected face center before the position is used by the controller.

This reduces camera movement caused by small detection fluctuations.

### Predictive Motion Estimation

The tracker estimates the image velocity of the target from consecutive measurements.

The observer also estimates how much apparent image motion is being produced by movement of the camera itself.

Conceptually,

```text
Target Image Velocity =
Observed Image Velocity
-
Camera Induced Image Velocity
```

This helps distinguish actual target motion from image movement caused by the pan and tilt servos.

### Model Predictive Control

Automatic tracking uses independent predictive controllers for the pan and tilt axes.

For each control cycle, the controller evaluates a range of possible servo velocity commands and simulates their effect over a future prediction horizon.

Each candidate command is scored using a cost function that considers

```text
Tracking error
Control effort
Change in control command
Terminal tracking error
```

The command producing the lowest predicted cost is selected.

This allows the system to react not only to where the target currently is, but also to where it is moving.

### Servo Dynamics Model

The predictive controller contains a simplified first order model of each servo axis.

The model includes

```text
Servo response time
Command gain
Estimated control delay
Pixels of image movement per degree of camera rotation
Maximum servo velocity
Maximum servo acceleration
```

Pan and tilt are modeled separately because the mechanical response of the two axes is different.

The current model parameters are initial estimates and are intended to be refined using experimental measurements.

### Delay Compensation

Camera processing, neural network inference, serial communication, and servo movement introduce delay.

The controller maintains a queue of previous commands to approximate this delay when predicting future system behavior.

This prevents the controller from assuming that a newly issued servo command affects the camera immediately.

### Predictive Tracking Activation

Tracking does not depend only on position error.

The system also considers estimated target velocity.

If a target begins moving quickly, tracking can activate before the target moves far from the center of the image.

This improves response to moving targets compared with a purely position based dead zone.

### Tracking Hysteresis

Different thresholds are used for starting and stopping movement.

For example, the pan axis requires a larger error to begin tracking than it requires to stop tracking.

This prevents rapid switching between moving and stopping when the target is close to the center of the frame.

### Stationary Hold

When the target remains close to the image center and nearly motionless for several consecutive control cycles, the tracker enters a stationary hold state.

During stationary hold

```text
Pan velocity is set to zero
Tilt velocity is set to zero
Predictive velocity states are reset
Servo targets are synchronized with the physical servo commands
```

The hold is released immediately when the target moves far enough or fast enough.

This reduces continuous servo hunting caused by small camera and detector fluctuations.

### Motion Constraints

Servo motion is limited by both velocity and acceleration constraints.

Current software limits are approximately

```text
Maximum velocity:      18 degrees per second
Maximum acceleration:  45 degrees per second squared
```

These limits help produce smoother physical movement and prevent aggressive controller commands.

### Servo Safety Limits

Software limits prevent the mechanism from commanding unsafe angles.

```text
Pan:   15 to 165 degrees
Tilt:  60 to 120 degrees
```

### Target Loss Handling

When the face is temporarily lost, servo velocities are gradually reduced instead of stopping abruptly.

After the target has been missing for several frames, the tracking and prediction states are reset.

This prevents stale target information from affecting the next detection.

### Manual Control

The tracker can be switched between automatic MPC tracking and manual control.

```text
M     Toggle automatic and manual mode

W     Tilt camera up
S     Tilt camera down
A     Pan camera left
D     Pan camera right

R     Smoothly return both axes to center

Q     Quit
ESC   Quit
```

### Smooth Center Reset

Pressing `R` disables automatic tracking and gradually returns both axes to

```text
Pan:  90 degrees
Tilt: 90 degrees
```

rather than immediately snapping the servos to their center positions.

## Visual Debugging

The OpenCV display includes

```text
Detected face bounding box
Detection confidence
Raw face center
Smoothed face center
Camera frame center
Tracking dead zone
Predicted future target position
Current pan angle
Current tilt angle
Number of detections
Current operating mode
```

The predicted target location is displayed separately from the current target location so the motion observer can be inspected visually.

## Controller Debug Output

The application also prints controller information to the terminal, including

```text
Pan target angle
Pan angle sent to Arduino
Tilt target angle
Tilt angle sent to Arduino
Horizontal image error
Vertical image error
Estimated target velocity
Estimated camera velocity
MPC pan command
MPC tilt command
Stationary hold status
```

This information can be recorded during testing and used to tune the predictive model.

## Hardware

The system is designed around

```text
Webcam
Arduino
Two axis pan tilt mechanism
Two servo motors
Computer running the C++ vision and control application
```

The computer performs neural network inference and predictive control while the Arduino receives pan and tilt angle commands and drives the physical servos.

## Software

The project uses

```text
C++
OpenCV
OpenCV DNN
ONNX
YOLO
Serial communication
Arduino
Model Predictive Control
```

## Communication

The computer communicates with the Arduino through a serial connection at

```text
115200 baud
```

Servo commands are transmitted using the format

```text
panAngle,tiltAngle
```

For example

```text
92.35,87.72
```

The controller avoids transmitting another command when both servo angles have changed by less than a small threshold.

## Current Control Configuration

The controller currently runs at approximately 20 Hz.

The MPC prediction horizon contains 10 control steps, corresponding to roughly 0.5 seconds of predicted motion.

Pan and tilt use separate dynamic models and delay estimates.

These values are currently starting estimates rather than experimentally identified plant parameters.

## Current Development Direction

The next major control improvement is experimental system identification.

Future testing can be used to measure

```text
Actual servo response time
Pan and tilt command delay
Pixels per degree of camera movement
Servo command gain
Response differences across servo angles
```

These measurements can then replace the initial model estimates used by the predictive controller.

Additional future improvements include

```text
Automatic controller parameter identification
Performance logging and plotting
Quantitative tracking error evaluation
Improved target reacquisition
Hardware specific servo calibration
Higher performance inference
More advanced coupled pan and tilt control
```

## Project Goal

The goal of this project is not simply to move a webcam toward a detected face.

It is to explore how computer vision, machine learning, state estimation, feedback control, and embedded hardware can be integrated into a complete real time robotic tracking system.
