#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>
#include <limits>
#include <sstream>
#include <iomanip>
#include "serial/serial.h"

void sendServoCommand(
    serial::Serial& arduino,
    float panAngle,
    float tiltAngle,
    float& previousPanAngle,
    float& previousTiltAngle)
{
    // Send sub-degree commands. This removes the 1-degree staircase
    // that can make a hobby servo sit still and then suddenly jump.
    constexpr float minimumCommandChange = 0.05f;

    if (
        std::isfinite(previousPanAngle) &&
        std::isfinite(previousTiltAngle) &&
        std::abs(panAngle - previousPanAngle) <
            minimumCommandChange &&
        std::abs(tiltAngle - previousTiltAngle) <
            minimumCommandChange
    )
    {
        return;
    }

    std::ostringstream command;

    command
        << std::fixed
        << std::setprecision(2)
        << panAngle
        << ","
        << tiltAngle
        << "\n";

    arduino.write(command.str());

    previousPanAngle = panAngle;
    previousTiltAngle = tiltAngle;
}

int main()
{
    // CAMERA
    constexpr int cameraIndex = 1;

    cv::VideoCapture cap(cameraIndex, cv::CAP_DSHOW);

    if (!cap.isOpened())
    {
        std::cerr << "ERROR: Could not open webcam.\n";
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_ZOOM, 0);

    std::cout << "Camera opened successfully.\n";

    // YOLO MODEL
    cv::dnn::Net net =
        cv::dnn::readNetFromONNX("Jerison_face.onnx");

    if (net.empty())
    {
        std::cerr << "ERROR: Could not load Jerison_face.onnx\n";
        return 1;
    }

    std::cout << "YOLO model loaded successfully.\n";

    // ARDUINO
    serial::Serial arduino;

    const std::string arduinoPort = "COM7";

    try
    {
        arduino.setPort(arduinoPort);
        arduino.setBaudrate(115200);

        serial::Timeout timeout =
            serial::Timeout::simpleTimeout(1000);

        arduino.setTimeout(timeout);
        arduino.open();
    }
    catch (const std::exception& error)
    {
        std::cerr
            << "Arduino connection error: "
            << error.what()
            << '\n';

        return 1;
    }

    if (!arduino.isOpen())
    {
        std::cerr << "ERROR: Arduino not connected.\n";
        return 1;
    }

    std::cout << "Arduino connected on " << arduinoPort << ".\n";

    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );

    // YOLO SETTINGS
    constexpr int inputWidth = 640;
    constexpr int inputHeight = 640;

    constexpr float confidenceThreshold = 0.50f;
    constexpr float nmsThreshold = 0.45f;

    // FACE POSITION SMOOTHING
    constexpr float smoothingAlpha = 0.07f;

    float smoothedFaceX = 0.0f;
    float smoothedFaceY = 0.0f;

    bool haveSmoothedTarget = false;

    // TRACKING HYSTERESIS
    // The axis starts moving only when the error is clearly outside center,
    // then stops only after it gets much closer to center.
    // This prevents stationary face noise from making the servos hunt.
    constexpr float panStartError = 45.0f;
    constexpr float panStopError = 18.0f;

    constexpr float tiltStartError = 40.0f;
    constexpr float tiltStopError = 15.0f;

    bool panTrackingActive = false;
    bool tiltTrackingActive = false;

    // PID GAINS
    // Pan is tuned separately because this axis was sluggish/occasionally reversed.
    float panKp = 0.022f;
    float panKi = 0.0015f;
    float panKd = 0.0020f;

    float tiltKp = 0.024f;
    float tiltKi = 0.0020f;
    float tiltKd = 0.0030f;

    // PID STATE
    float previousErrorX = 0.0f;
    float previousErrorY = 0.0f;

    float integralX = 0.0f;
    float integralY = 0.0f;

    float filteredDerivativeX = 0.0f;
    float filteredDerivativeY = 0.0f;

    bool pidInitialized = false;

    // DERIVATIVE FILTER
    constexpr float derivativeAlpha = 0.20f;

    // SERVO COMMANDS
    // panCommand / tiltCommand are the continuous PID targets.
    // panServoCommand / tiltServoCommand are what is actually sent.
    float panCommand = 90.0f;
    float tiltCommand = 90.0f;

    float panServoCommand = 90.0f;
    float tiltServoCommand = 90.0f;

    // SHARED DUAL-AXIS OUTPUT CLOCK
    // Pan keeps its 2-degree mechanical stepping.
    // Tilt keeps its smooth continuous PID command.
    // Both are sent together on the same 50 ms output tick.
    constexpr float panStepDegrees = 2.0f;
    constexpr int sharedOutputIntervalMs = 50;

    auto previousCoordinatedStepTime =
        std::chrono::steady_clock::now();

    // SERVO VELOCITIES
    float panVelocity = 0.0f;
    float tiltVelocity = 0.0f;

    // MOTION LIMITS
    constexpr float maximumVelocity = 18.0f;
    constexpr float maximumAcceleration = 45.0f;

    // SAFE SERVO LIMITS
    constexpr float panMinimum = 15.0f;
    constexpr float panMaximum = 165.0f;

    constexpr float tiltMinimum = 60.0f;
    constexpr float tiltMaximum = 120.0f;

    // CONTROL LOOP RATE
    constexpr int controlIntervalMs = 50;

    auto previousControlTime =
        std::chrono::steady_clock::now();

    // MANUAL MODE
    bool manualMode = false;

    constexpr float manualServoStep = 2.0f;

    // SMOOTH CENTER RESET
    bool smoothResetActive = false;
    constexpr float resetPanTarget = 90.0f;
    constexpr float resetTiltTarget = 90.0f;
    constexpr float resetSpeed = 10.0f; // degrees per second

    auto previousResetTime =
        std::chrono::steady_clock::now();

    // TARGET LOSS
    int lostFrames = 0;

    constexpr int resetAfterLostFrames = 10;

    // PREVIOUS SERVO COMMANDS
    float previousPanAngle =
        std::numeric_limits<float>::quiet_NaN();

    float previousTiltAngle =
        std::numeric_limits<float>::quiet_NaN();

    bool printedOutputShape = false;

    cv::Mat frame;

    while (true)
    {
        // READ CAMERA
        if (!cap.read(frame) || frame.empty())
        {
            std::cerr << "ERROR: Failed to read webcam frame.\n";
            break;
        }

        // LETTERBOX IMAGE
        float scale =
            std::min(
                static_cast<float>(inputWidth) /
                    static_cast<float>(frame.cols),

                static_cast<float>(inputHeight) /
                    static_cast<float>(frame.rows)
            );

        int resizedWidth =
            static_cast<int>(
                std::round(frame.cols * scale)
            );

        int resizedHeight =
            static_cast<int>(
                std::round(frame.rows * scale)
            );

        cv::Mat resized;

        cv::resize(
            frame,
            resized,
            cv::Size(resizedWidth, resizedHeight)
        );

        int padX =
            (inputWidth - resizedWidth) / 2;

        int padY =
            (inputHeight - resizedHeight) / 2;

        int padRight =
            inputWidth -
            resizedWidth -
            padX;

        int padBottom =
            inputHeight -
            resizedHeight -
            padY;

        cv::Mat letterboxed;

        cv::copyMakeBorder(
            resized,
            letterboxed,
            padY,
            padBottom,
            padX,
            padRight,
            cv::BORDER_CONSTANT,
            cv::Scalar(114, 114, 114)
        );

        // YOLO INPUT
        cv::Mat blob;

        cv::dnn::blobFromImage(
            letterboxed,
            blob,
            1.0 / 255.0,
            cv::Size(inputWidth, inputHeight),
            cv::Scalar(),
            true,
            false
        );

        net.setInput(blob);

        // RUN YOLO
        std::vector<cv::Mat> outputs;

        net.forward(
            outputs,
            net.getUnconnectedOutLayersNames()
        );

        if (outputs.empty())
        {
            std::cerr << "ERROR: YOLO returned no output.\n";
            break;
        }

        cv::Mat output = outputs[0];

        if (!printedOutputShape)
        {
            std::cout << "YOLO output: ";

            for (int i = 0; i < output.dims; i++)
            {
                std::cout << output.size[i];

                if (i < output.dims - 1)
                    std::cout << " x ";
            }

            std::cout << '\n';

            printedOutputShape = true;
        }

        if (output.dims != 3)
        {
            std::cerr << "ERROR: Unexpected YOLO output.\n";
            break;
        }

        // PARSE YOLO OUTPUT
        int dimensions = 0;
        int detections = 0;

        cv::Mat detectionData;

        if (output.size[1] < output.size[2])
        {
            dimensions = output.size[1];
            detections = output.size[2];

            cv::Mat raw(
                dimensions,
                detections,
                CV_32F,
                output.ptr<float>()
            );

            cv::transpose(raw, detectionData);
        }
        else
        {
            detections = output.size[1];
            dimensions = output.size[2];

            detectionData =
                cv::Mat(
                    detections,
                    dimensions,
                    CV_32F,
                    output.ptr<float>()
                );
        }

        if (dimensions != 5)
        {
            std::cerr
                << "ERROR: Expected 5 values per YOLO detection.\n";

            break;
        }

        // EXTRACT DETECTIONS
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;

        for (int i = 0; i < detections; i++)
        {
            const float* data =
                detectionData.ptr<float>(i);

            float centerX = data[0];
            float centerY = data[1];
            float width = data[2];
            float height = data[3];
            float confidence = data[4];

            if (confidence < confidenceThreshold)
                continue;

            float leftLetterbox =
                centerX -
                width / 2.0f;

            float topLetterbox =
                centerY -
                height / 2.0f;

            float originalLeft =
                (leftLetterbox - padX) / scale;

            float originalTop =
                (topLetterbox - padY) / scale;

            float originalWidth =
                width / scale;

            float originalHeight =
                height / scale;

            cv::Rect box(
                static_cast<int>(
                    std::round(originalLeft)
                ),

                static_cast<int>(
                    std::round(originalTop)
                ),

                static_cast<int>(
                    std::round(originalWidth)
                ),

                static_cast<int>(
                    std::round(originalHeight)
                )
            );

            box &=
                cv::Rect(
                    0,
                    0,
                    frame.cols,
                    frame.rows
                );

            if (box.width <= 0 || box.height <= 0)
                continue;

            boxes.push_back(box);
            scores.push_back(confidence);
        }

        // NON MAXIMUM SUPPRESSION
        std::vector<int> indices;

        cv::dnn::NMSBoxes(
            boxes,
            scores,
            confidenceThreshold,
            nmsThreshold,
            indices
        );

        // FRAME CENTER
        int frameCenterX =
            frame.cols / 2;

        int frameCenterY =
            frame.rows / 2;

        cv::circle(
            frame,
            cv::Point(
                frameCenterX,
                frameCenterY
            ),
            6,
            cv::Scalar(255, 0, 0),
            -1
        );

        // DEAD ZONE
        cv::rectangle(
            frame,
            cv::Point(
                frameCenterX -
                    static_cast<int>(panStopError),

                frameCenterY -
                    static_cast<int>(tiltStopError)
            ),

            cv::Point(
                frameCenterX +
                    static_cast<int>(panStopError),

                frameCenterY +
                    static_cast<int>(tiltStopError)
            ),

            cv::Scalar(255, 255, 0),
            2
        );

        // TARGET SELECTION
        int targetIndex = -1;

        if (!indices.empty())
        {
            if (!haveSmoothedTarget)
            {
                float highestConfidence = -1.0f;

                for (int index : indices)
                {
                    if (scores[index] > highestConfidence)
                    {
                        highestConfidence =
                            scores[index];

                        targetIndex =
                            index;
                    }
                }
            }
            else
            {
                float smallestDistance =
                    std::numeric_limits<float>::max();

                for (int index : indices)
                {
                    const cv::Rect& box =
                        boxes[index];

                    float candidateX =
                        box.x +
                        box.width / 2.0f;

                    float candidateY =
                        box.y +
                        box.height / 2.0f;

                    float dx =
                        candidateX -
                        smoothedFaceX;

                    float dy =
                        candidateY -
                        smoothedFaceY;

                    float distanceSquared =
                        dx * dx +
                        dy * dy;

                    if (distanceSquared < smallestDistance)
                    {
                        smallestDistance =
                            distanceSquared;

                        targetIndex =
                            index;
                    }
                }
            }
        }

        // TARGET FOUND
        if (targetIndex != -1)
        {
            lostFrames = 0;

            cv::Rect box =
                boxes[targetIndex];

            float targetConfidence =
                scores[targetIndex];

            cv::rectangle(
                frame,
                box,
                cv::Scalar(0, 255, 0),
                2
            );

            std::string label =
                "jerison_face " +
                cv::format(
                    "%.2f",
                    targetConfidence
                );

            cv::putText(
                frame,
                label,
                cv::Point(
                    box.x,
                    std::max(
                        20,
                        box.y - 10
                    )
                ),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 0),
                2
            );

            // RAW FACE POSITION
            float rawFaceX =
                box.x +
                box.width / 2.0f;

            float rawFaceY =
                box.y +
                box.height / 2.0f;

            cv::circle(
                frame,
                cv::Point(
                    static_cast<int>(rawFaceX),
                    static_cast<int>(rawFaceY)
                ),
                4,
                cv::Scalar(0, 0, 255),
                -1
            );

            // SMOOTH FACE POSITION
            if (!haveSmoothedTarget)
            {
                smoothedFaceX = rawFaceX;
                smoothedFaceY = rawFaceY;

                haveSmoothedTarget = true;
            }
            else
            {
                smoothedFaceX =
                    smoothingAlpha * rawFaceX +
                    (1.0f - smoothingAlpha) *
                    smoothedFaceX;

                smoothedFaceY =
                    smoothingAlpha * rawFaceY +
                    (1.0f - smoothingAlpha) *
                    smoothedFaceY;
            }

            int faceCenterX =
                static_cast<int>(
                    std::round(smoothedFaceX)
                );

            int faceCenterY =
                static_cast<int>(
                    std::round(smoothedFaceY)
                );

            cv::circle(
                frame,
                cv::Point(
                    faceCenterX,
                    faceCenterY
                ),
                6,
                cv::Scalar(0, 255, 255),
                -1
            );

            cv::line(
                frame,
                cv::Point(
                    frameCenterX,
                    frameCenterY
                ),
                cv::Point(
                    faceCenterX,
                    faceCenterY
                ),
                cv::Scalar(255, 255, 0),
                2
            );

            // AUTO PD CONTROL
            if (!manualMode)
            {
                auto now =
                    std::chrono::steady_clock::now();

                auto elapsedMs =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds
                    >(
                        now -
                        previousControlTime
                    ).count();

                if (elapsedMs >= controlIntervalMs)
                {
                    float dt =
                        std::chrono::duration<float>(
                            now -
                            previousControlTime
                        ).count();

                    previousControlTime =
                        now;

                    // Prevent a long pause / lost target from producing
                    // one huge control step when the face is detected again.
                    dt =
                        std::clamp(
                            dt,
                            0.001f,
                            0.10f
                        );

                    float errorX =
                        static_cast<float>(
                            faceCenterX -
                            frameCenterX
                        );

                    float errorY =
                        static_cast<float>(
                            faceCenterY -
                            frameCenterY
                        );

                    // HYSTERESIS
                    // Do not continuously switch between moving/stopping when
                    // the detected face jitters by a few pixels.
                    float absoluteErrorX =
                        std::abs(errorX);

                    float absoluteErrorY =
                        std::abs(errorY);

                    if (panTrackingActive)
                    {
                        if (absoluteErrorX <= panStopError)
                            panTrackingActive = false;
                    }
                    else
                    {
                        if (absoluteErrorX >= panStartError)
                            panTrackingActive = true;
                    }

                    if (tiltTrackingActive)
                    {
                        if (absoluteErrorY <= tiltStopError)
                            tiltTrackingActive = false;
                    }
                    else
                    {
                        if (absoluteErrorY >= tiltStartError)
                            tiltTrackingActive = true;
                    }

                    if (!panTrackingActive)
                    {
                        errorX = 0.0f;
                        integralX = 0.0f;
                    }

                    if (!tiltTrackingActive)
                    {
                        errorY = 0.0f;
                        integralY = 0.0f;
                    }

                    // INITIALIZE PID
                    if (!pidInitialized)
                    {
                        previousErrorX = errorX;
                        previousErrorY = errorY;

                        filteredDerivativeX = 0.0f;
                        filteredDerivativeY = 0.0f;

                        pidInitialized = true;
                    }

                    // ERROR VELOCITY
                    float rawDerivativeX =
                        (errorX -
                         previousErrorX) /
                        dt;

                    float rawDerivativeY =
                        (errorY -
                         previousErrorY) /
                        dt;

                    // FILTER DERIVATIVE
                    filteredDerivativeX =
                        derivativeAlpha *
                        rawDerivativeX +
                        (1.0f -
                         derivativeAlpha) *
                        filteredDerivativeX;

                    filteredDerivativeY =
                        derivativeAlpha *
                        rawDerivativeY +
                        (1.0f -
                         derivativeAlpha) *
                        filteredDerivativeY;

                    // INTEGRAL
                    integralX +=
                        errorX *
                        dt;

                    integralY +=
                        errorY *
                        dt;

                    // ANTI WINDUP
                    integralX =
                        std::clamp(
                            integralX,
                            -1000.0f,
                            1000.0f
                        );

                    integralY =
                        std::clamp(
                            integralY,
                            -1000.0f,
                            1000.0f
                        );

                    // PID OUTPUT = DESIRED SERVO VELOCITY

                    // PAN PID
                    // Physical mapping confirmed:
                    // increasing pan angle = camera RIGHT
                    // decreasing pan angle = camera LEFT
                    float panP =
                        panKp * errorX;

                    // Integral removes a persistent centering error, but
                    // cap its contribution so it cannot wind up and cause hunting.
                    float panI =
                        std::clamp(
                            panKi * integralX,
                            -0.75f,
                            0.75f
                        );

                    float panD =
                        panKd *
                        filteredDerivativeX;

                    // Keep D as damping only. Do not let a noisy derivative
                    // overpower P and reverse the desired pan direction.
                    float maximumPanD =
                        std::abs(panP) * 0.50f;

                    panD =
                        std::clamp(
                            panD,
                            -maximumPanD,
                            maximumPanD
                        );

                    float desiredPanVelocity =
                        panP +
                        panI +
                        panD;

                    // TILT PID
                    float tiltP =
                        tiltKp * errorY;

                    float tiltI =
                        std::clamp(
                            tiltKi * integralY,
                            -1.20f,
                            1.20f
                        );

                    float tiltD =
                        tiltKd *
                        filteredDerivativeY;

                    // Keep tilt derivative from overpowering the direction
                    // set by the proportional term.
                    float maximumTiltD =
                        std::abs(tiltP) * 0.50f;

                    tiltD =
                        std::clamp(
                            tiltD,
                            -maximumTiltD,
                            maximumTiltD
                        );

                    float desiredTiltVelocity =
                        tiltP +
                        tiltI +
                        tiltD;

                    // PHYSICAL DIRECTION CORRECTION:
                    // Pan is physically reversed, so invert pan.
                    // Tilt is NOT reversed:
                    // face below center -> positive errorY -> increase tilt angle -> camera DOWN
                    desiredPanVelocity =
                        -desiredPanVelocity;

                    // Use only a very small minimum velocity. The integral
                    // term can now build gently against persistent error instead
                    // of forcing a large sudden kick.
                    constexpr float minimumPanVelocity = 1.0f;

                    if (
                        errorX != 0.0f &&
                        std::abs(desiredPanVelocity) <
                            minimumPanVelocity
                    )
                    {
                        desiredPanVelocity =
                            errorX > 0.0f
                            ? -minimumPanVelocity
                            : minimumPanVelocity;
                    }

                    // Small minimum tilt velocity only. Integral action
                    // handles persistent offset without aggressive hunting.
                    constexpr float minimumTiltVelocity = 0.8f;

                    if (
                        errorY != 0.0f &&
                        std::abs(desiredTiltVelocity) <
                            minimumTiltVelocity
                    )
                    {
                        desiredTiltVelocity =
                            errorY > 0.0f
                            ? minimumTiltVelocity
                            : -minimumTiltVelocity;
                    }

                    // MAXIMUM SPEED
                    desiredPanVelocity =
                        std::clamp(
                            desiredPanVelocity,
                            -maximumVelocity,
                            maximumVelocity
                        );

                    desiredTiltVelocity =
                        std::clamp(
                            desiredTiltVelocity,
                            -maximumVelocity,
                            maximumVelocity
                        );

                    // ACCELERATION LIMIT
                    float maximumVelocityChange =
                        maximumAcceleration *
                        dt;

                    float panVelocityChange =
                        desiredPanVelocity -
                        panVelocity;

                    float tiltVelocityChange =
                        desiredTiltVelocity -
                        tiltVelocity;

                    panVelocityChange =
                        std::clamp(
                            panVelocityChange,
                            -maximumVelocityChange,
                            maximumVelocityChange
                        );

                    tiltVelocityChange =
                        std::clamp(
                            tiltVelocityChange,
                            -maximumVelocityChange,
                            maximumVelocityChange
                        );

                    panVelocity +=
                        panVelocityChange;

                    tiltVelocity +=
                        tiltVelocityChange;

                    // IF CENTERED, SLOW TO A STOP
                    if (errorX == 0.0f)
                    {
                        panVelocity *= 0.45f;

                        if (
                            std::abs(
                                panVelocity
                            ) <
                            0.25f
                        )
                        {
                            panVelocity = 0.0f;
                        }
                    }

                    if (errorY == 0.0f)
                    {
                        tiltVelocity *= 0.40f;

                        if (
                            std::abs(
                                tiltVelocity
                            ) <
                            0.25f
                        )
                        {
                            tiltVelocity = 0.0f;
                        }
                    }

                    // VELOCITY -> POSITION
                    panCommand +=
                        panVelocity *
                        dt;

                    tiltCommand +=
                        tiltVelocity *
                        dt;

                    // SAFE LIMITS
                    float unclampedPan =
                        panCommand;

                    float unclampedTilt =
                        tiltCommand;

                    panCommand =
                        std::clamp(
                            panCommand,
                            panMinimum,
                            panMaximum
                        );

                    tiltCommand =
                        std::clamp(
                            tiltCommand,
                            tiltMinimum,
                            tiltMaximum
                        );

                    // STOP VELOCITY AT HARD LIMITS
                    if (panCommand != unclampedPan)
                        panVelocity = 0.0f;

                    if (tiltCommand != unclampedTilt)
                        tiltVelocity = 0.0f;

                    // SHARED OUTPUT UPDATE
                    // Do NOT scale tilt by pan error. That made tilt weak whenever
                    // pan had the larger error. Pan is stepped for the mechanics,
                    // while tilt follows its already-smoothed PID target directly.
                    auto coordinatedNow =
                        std::chrono::steady_clock::now();

                    auto coordinatedElapsedMs =
                        std::chrono::duration_cast<
                            std::chrono::milliseconds
                        >(
                            coordinatedNow -
                            previousCoordinatedStepTime
                        ).count();

                    if (
                        coordinatedElapsedMs >=
                            sharedOutputIntervalMs
                    )
                    {
                        float panDifference =
                            panCommand -
                            panServoCommand;

                        if (
                            std::abs(panDifference) >=
                                0.05f
                        )
                        {
                            float panStep =
                                std::clamp(
                                    panDifference,
                                    -panStepDegrees,
                                    panStepDegrees
                                );

                            panServoCommand +=
                                panStep;

                            panServoCommand =
                                std::clamp(
                                    panServoCommand,
                                    panMinimum,
                                    panMaximum
                                );
                        }

                        // Preserve the good tilt behavior from the earlier
                        // version. tiltCommand is already velocity/acceleration
                        // limited by the PID controller, so no extra stepping
                        // or proportional scaling is needed here.
                        tiltServoCommand =
                            std::clamp(
                                tiltCommand,
                                tiltMinimum,
                                tiltMaximum
                            );

                        previousCoordinatedStepTime =
                            coordinatedNow;

                        sendServoCommand(
                            arduino,
                            panServoCommand,
                            tiltServoCommand,
                            previousPanAngle,
                            previousTiltAngle
                        );
                    }

                    std::cout
                        << std::fixed
                        << std::setprecision(2)
                        << "PanTarget="
                        << panCommand
                        << " PanSent="
                        << panServoCommand
                        << " TiltTarget="
                        << tiltCommand
                        << " TiltSent="
                        << tiltServoCommand
                        << " ErrorX="
                        << errorX
                        << " ErrorY="
                        << errorY
                        << " PanVel="
                        << panVelocity
                        << " TiltVel="
                        << tiltVelocity
                        << " Conf="
                        << targetConfidence
                        << '\n';

                    previousErrorX =
                        errorX;

                    previousErrorY =
                        errorY;
                }
            }
        }
        else
        {
            lostFrames++;

            panTrackingActive = false;
            tiltTrackingActive = false;

            // Reset control timing while the face is missing.
            // Otherwise dt keeps growing and the first detection can jump
            // straight to a servo hard limit.
            previousControlTime =
                std::chrono::steady_clock::now();

            // GRADUALLY STOP IF FACE IS LOST
            panVelocity *= 0.45f;
            tiltVelocity *= 0.40f;

            if (std::abs(panVelocity) < 0.5f)
                panVelocity = 0.0f;

            if (std::abs(tiltVelocity) < 0.5f)
                tiltVelocity = 0.0f;

            if (
                lostFrames >=
                resetAfterLostFrames
            )
            {
                haveSmoothedTarget = false;
                pidInitialized = false;

                integralX = 0.0f;
                integralY = 0.0f;

                filteredDerivativeX = 0.0f;
                filteredDerivativeY = 0.0f;
            }
        }

        // DISPLAY
        int displayedPan =
            static_cast<int>(
                std::round(
                    panServoCommand
                )
            );

        int displayedTilt =
            static_cast<int>(
                std::round(
                    tiltServoCommand
                )
            );

        cv::putText(
            frame,
            "Pan: " +
            std::to_string(
                displayedPan
            ),
            cv::Point(20, 35),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2
        );

        cv::putText(
            frame,
            "Tilt: " +
            std::to_string(
                displayedTilt
            ),
            cv::Point(20, 65),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2
        );

        cv::putText(
            frame,
            "Detections: " +
            std::to_string(
                indices.size()
            ),
            cv::Point(20, 95),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2
        );

        std::string modeText =
            manualMode
            ? "MANUAL | WASD | R smooth center | M auto"
            : "AUTO PID | M manual";

        cv::putText(
            frame,
            modeText,
            cv::Point(20, 125),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 255),
            2
        );

        cv::imshow(
            "Custom YOLO Face Pan Tilt Tracker",
            frame
        );

        // KEYBOARD INPUT
        int key =
            cv::waitKey(1) &
            0xFF;

        // TOGGLE MANUAL/AUTO
        if (
            key == 'm' ||
            key == 'M'
        )
        {
            manualMode =
                !manualMode;

            panVelocity = 0.0f;
            tiltVelocity = 0.0f;

            // Start from the actual positions that were last sent.
            panCommand = panServoCommand;
            tiltCommand = tiltServoCommand;

            pidInitialized = false;

            integralX = 0.0f;
            integralY = 0.0f;

            panTrackingActive = false;
            tiltTrackingActive = false;

            previousControlTime =
                std::chrono::steady_clock::now();

            std::cout
                << (
                    manualMode
                    ? "MANUAL MODE\n"
                    : "AUTO PID MODE\n"
                );
        }

        // R = smoothly return both axes to center.
        // Switch to manual so AUTO does not fight the reset.
        if (
            key == 'r' ||
            key == 'R'
        )
        {
            manualMode = true;
            smoothResetActive = true;

            panVelocity = 0.0f;
            tiltVelocity = 0.0f;

            panCommand = panServoCommand;
            tiltCommand = tiltServoCommand;

            pidInitialized = false;
            integralX = 0.0f;
            integralY = 0.0f;

            previousResetTime =
                std::chrono::steady_clock::now();

            std::cout
                << "SMOOTH RESET STARTED\n";
        }

        // MANUAL MODE
        if (manualMode)
        {
            bool changed = false;

            if (
                key == 'w' ||
                key == 'W'
            )
            {
                smoothResetActive = false;

                // W = camera UP
                tiltCommand -=
                    manualServoStep;

                changed = true;
            }

            if (
                key == 's' ||
                key == 'S'
            )
            {
                smoothResetActive = false;

                // S = camera DOWN
                tiltCommand +=
                    manualServoStep;

                changed = true;
            }

            if (
                key == 'a' ||
                key == 'A'
            )
            {
                smoothResetActive = false;

                // A = camera LEFT
                panCommand -=
                    manualServoStep;

                changed = true;
            }

            if (
                key == 'd' ||
                key == 'D'
            )
            {
                smoothResetActive = false;

                // D = camera RIGHT
                panCommand +=
                    manualServoStep;

                changed = true;
            }

            if (smoothResetActive)
            {
                auto resetNow =
                    std::chrono::steady_clock::now();

                float resetDt =
                    std::chrono::duration<float>(
                        resetNow -
                        previousResetTime
                    ).count();

                previousResetTime =
                    resetNow;

                resetDt =
                    std::clamp(
                        resetDt,
                        0.0f,
                        0.05f
                    );

                float maximumResetStep =
                    resetSpeed *
                    resetDt;

                auto moveToward =
                    [](
                        float current,
                        float target,
                        float maximumStep
                    )
                    {
                        float difference =
                            target -
                            current;

                        if (
                            std::abs(difference) <=
                            maximumStep
                        )
                        {
                            return target;
                        }

                        return current +
                            (
                                difference > 0.0f
                                ? maximumStep
                                : -maximumStep
                            );
                    };

                float oldPan =
                    panCommand;

                float oldTilt =
                    tiltCommand;

                panCommand =
                    moveToward(
                        panCommand,
                        resetPanTarget,
                        maximumResetStep
                    );

                tiltCommand =
                    moveToward(
                        tiltCommand,
                        resetTiltTarget,
                        maximumResetStep
                    );

                if (
                    panCommand != oldPan ||
                    tiltCommand != oldTilt
                )
                {
                    changed = true;
                }

                if (
                    panCommand == resetPanTarget &&
                    tiltCommand == resetTiltTarget
                )
                {
                    smoothResetActive = false;

                    std::cout
                        << "SMOOTH RESET COMPLETE\n";
                }
            }

            panCommand =
                std::clamp(
                    panCommand,
                    panMinimum,
                    panMaximum
                );

            tiltCommand =
                std::clamp(
                    tiltCommand,
                    tiltMinimum,
                    tiltMaximum
                );

            if (changed)
            {
                // Manual commands update both actual outputs immediately.
                panServoCommand =
                    panCommand;

                tiltServoCommand =
                    tiltCommand;

                previousCoordinatedStepTime =
                    std::chrono::steady_clock::now();

                sendServoCommand(
                    arduino,
                    panServoCommand,
                    tiltServoCommand,
                    previousPanAngle,
                    previousTiltAngle
                );

                std::cout
                    << std::fixed
                    << std::setprecision(2)
                    << "MANUAL PanSent="
                    << panServoCommand
                    << " TiltSent="
                    << tiltServoCommand
                    << '\n';
            }
        }

        // QUIT
        if (
            key == 'q' ||
            key == 'Q' ||
            key == 27
        )
        {
            break;
        }
    }

    if (arduino.isOpen())
        arduino.close();

    cap.release();
    cv::destroyAllWindows();

    return 0;
}