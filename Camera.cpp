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
#include "serial/serial.h"

void sendServoCommand(
    serial::Serial& arduino,
    int panAngle,
    int tiltAngle,
    int& previousPanAngle,
    int& previousTiltAngle)
{
    if (panAngle == previousPanAngle &&
        tiltAngle == previousTiltAngle)
        return;

    std::string command =
        std::to_string(panAngle) + "," +
        std::to_string(tiltAngle) + "\n";

    arduino.write(command);

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

    try
    {
        arduino.setPort("COM7");
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

    std::cout << "Arduino connected on COM7.\n";

    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );

    // YOLO SETTINGS
    constexpr int inputWidth = 640;
    constexpr int inputHeight = 640;

    constexpr float confidenceThreshold = 0.50f;
    constexpr float nmsThreshold = 0.45f;

    // FACE POSITION SMOOTHING
    constexpr float smoothingAlpha = 0.10f;

    float smoothedFaceX = 0.0f;
    float smoothedFaceY = 0.0f;

    bool haveSmoothedTarget = false;

    // DEAD ZONE
    constexpr float deadZone = 30.0f;

    // PID GAINS
    // Kp reacts to current face-position error.
    // Ki removes small persistent offsets.
    // Kd damps motion and reacts to how quickly the error is changing.
    float panKp = 0.030f;
    float panKi = 0.0005f;
    float panKd = 0.003f;

    float tiltKp = 0.030f;
    float tiltKi = 0.0005f;
    float tiltKd = 0.003f;

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
    float panCommand = 90.0f;
    float tiltCommand = 90.0f;

    // SERVO VELOCITIES
    float panVelocity = 0.0f;
    float tiltVelocity = 0.0f;

    // MOTION LIMITS
    constexpr float maximumVelocity = 35.0f;
    constexpr float maximumAcceleration = 100.0f;

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

    // TARGET LOSS
    int lostFrames = 0;

    constexpr int resetAfterLostFrames = 10;

    // PREVIOUS SERVO COMMANDS
    int previousPanAngle = -1;
    int previousTiltAngle = -1;

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
                    static_cast<int>(deadZone),

                frameCenterY -
                    static_cast<int>(deadZone)
            ),

            cv::Point(
                frameCenterX +
                    static_cast<int>(deadZone),

                frameCenterY +
                    static_cast<int>(deadZone)
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

                    if (dt <= 0.0f)
                        dt = 0.001f;

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

                    // DEAD ZONE
                    if (std::abs(errorX) < deadZone)
                    {
                        errorX = 0.0f;
                        integralX = 0.0f;
                    }

                    if (std::abs(errorY) < deadZone)
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
                    float desiredPanVelocity =
                        panKp * errorX +
                        panKi * integralX +
                        panKd *
                        filteredDerivativeX;

                    float desiredTiltVelocity =
                        tiltKp * errorY +
                        tiltKi * integralY +
                        tiltKd *
                        filteredDerivativeY;

                    // HARDWARE DIRECTION:
                    // Increasing pan angle moves the camera RIGHT.
                    // Increasing tilt angle moves the camera DOWN.
                    // Therefore positive image error should produce
                    // positive servo velocity on both axes.

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
                        panVelocity *= 0.70f;

                        if (
                            std::abs(
                                panVelocity
                            ) <
                            0.5f
                        )
                        {
                            panVelocity = 0.0f;
                        }
                    }

                    if (errorY == 0.0f)
                    {
                        tiltVelocity *= 0.70f;

                        if (
                            std::abs(
                                tiltVelocity
                            ) <
                            0.5f
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

                    int panAngle =
                        static_cast<int>(
                            std::round(
                                panCommand
                            )
                        );

                    int tiltAngle =
                        static_cast<int>(
                            std::round(
                                tiltCommand
                            )
                        );

                    sendServoCommand(
                        arduino,
                        panAngle,
                        tiltAngle,
                        previousPanAngle,
                        previousTiltAngle
                    );

                    std::cout
                        << "Pan="
                        << panAngle
                        << " Tilt="
                        << tiltAngle
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

            // GRADUALLY STOP IF FACE IS LOST
            panVelocity *= 0.70f;
            tiltVelocity *= 0.70f;

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
                    panCommand
                )
            );

        int displayedTilt =
            static_cast<int>(
                std::round(
                    tiltCommand
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
            ? "MANUAL | W up S down A left D right | R center | M auto"
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

            pidInitialized = false;

            integralX = 0.0f;
            integralY = 0.0f;

            std::cout
                << (
                    manualMode
                    ? "MANUAL MODE\n"
                    : "AUTO PID MODE\n"
                );
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
                // W = camera UP
                // S = camera DOWN
                tiltCommand +=
                    manualServoStep;

                changed = true;
            }

            if (
                key == 's' ||
                key == 'S'
            )
            {
                tiltCommand -=
                    manualServoStep;

                changed = true;
            }

            if (
                key == 'a' ||
                key == 'A'
            )
            {
                // A = camera LEFT
                // D = camera RIGHT
                panCommand +=
                    manualServoStep;

                changed = true;
            }

            if (
                key == 'd' ||
                key == 'D'
            )
            {
                panCommand -=
                    manualServoStep;

                changed = true;
            }

            if (
                key == 'r' ||
                key == 'R'
            )
            {
                panCommand = 90.0f;
                tiltCommand = 90.0f;

                changed = true;
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
                int panAngle =
                    static_cast<int>(
                        std::round(
                            panCommand
                        )
                    );

                int tiltAngle =
                    static_cast<int>(
                        std::round(
                            tiltCommand
                        )
                    );

                sendServoCommand(
                    arduino,
                    panAngle,
                    tiltAngle,
                    previousPanAngle,
                    previousTiltAngle
                );

                std::cout
                    << "MANUAL Pan="
                    << panAngle
                    << " Tilt="
                    << tiltAngle
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