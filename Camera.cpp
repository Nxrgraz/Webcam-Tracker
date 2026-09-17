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
#include <deque>
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

struct AxisMPCConfig
{
    float tauSeconds;
    float commandGain;
    float delaySeconds;
    float pixelsPerDegree;
    float cameraEffectSign;

    float qError;
    float rControl;
    float sCommandChange;

    float maximumVelocity;
    float maximumAcceleration;

    int horizonSteps;
    float candidateStep;
};

struct AxisPredictiveState
{
    float modelVelocity = 0.0f;
    float targetImageVelocity = 0.0f;

    // Filtered raw image velocity. This is used only to determine whether
    // the detected face is genuinely stationary rather than reacting to
    // small YOLO / camera jitter.
    float measuredImageVelocity = 0.0f;

    float previousMeasuredError = 0.0f;
    float previousCommand = 0.0f;

    int delaySteps = 0;
    bool initialized = false;

    std::deque<float> delayQueue;
};

void resetAxisPredictiveState(
    AxisPredictiveState& state,
    float measuredError,
    const AxisMPCConfig& config,
    float nominalDt)
{
    state.modelVelocity = 0.0f;
    state.targetImageVelocity = 0.0f;
    state.measuredImageVelocity = 0.0f;
    state.previousMeasuredError = measuredError;
    state.previousCommand = 0.0f; 

    state.delaySteps =
        std::max(
            0,
            static_cast<int>(
                std::round(
                    config.delaySeconds /
                    nominalDt
                )
            )
        );

    state.delayQueue.clear();

    for (int i = 0; i < state.delaySteps; ++i)
        state.delayQueue.push_back(0.0f);

    state.initialized = true;
}

void advanceAxisObserver(
    AxisPredictiveState& state,
    float measuredError,
    float dt,
    const AxisMPCConfig& config,
    float nominalDt)
{
    if (!state.initialized)
    {
        resetAxisPredictiveState(
            state,
            measuredError,
            config,
            nominalDt
        );

        return;
    }

    float delayedCommand =
        state.previousCommand;

    if (state.delaySteps > 0)
    {
        if (state.delayQueue.empty())
            state.delayQueue.push_back(
                state.previousCommand
            );

        delayedCommand =
            state.delayQueue.front();

        state.delayQueue.pop_front();
    }

    float a =
        std::exp(
            -dt /
            std::max(
                config.tauSeconds,
                0.001f
            )
        );

    float b =
        config.commandGain *
        (1.0f - a);

    state.modelVelocity =
        a * state.modelVelocity +
        b * delayedCommand;

    float relativeImageVelocity =
        (
            measuredError -
            state.previousMeasuredError
        ) /
        std::max(dt, 0.001f);

    constexpr float measuredVelocityAlpha = 0.18f;

    state.measuredImageVelocity =
        measuredVelocityAlpha *
        relativeImageVelocity +
        (1.0f - measuredVelocityAlpha) *
        state.measuredImageVelocity;

    float cameraImageVelocity =
        config.cameraEffectSign *
        config.pixelsPerDegree *
        state.modelVelocity;

    float measuredTargetImageVelocity =
        relativeImageVelocity -
        cameraImageVelocity;

    measuredTargetImageVelocity =
        std::clamp(
            measuredTargetImageVelocity,
            -700.0f,
            700.0f
        );

    constexpr float observerAlpha = 0.14f;

    state.targetImageVelocity =
        observerAlpha *
        measuredTargetImageVelocity +
        (1.0f - observerAlpha) *
        state.targetImageVelocity;

    state.previousMeasuredError =
        measuredError;
}

void queueAxisCommand(
    AxisPredictiveState& state,
    float command)
{
    if (state.delaySteps > 0)
        state.delayQueue.push_back(command);

    state.previousCommand =
        command;
}

float solveAxisMPC(
    float measuredError,
    const AxisPredictiveState& state,
    const AxisMPCConfig& config,
    float dt)
{
    float bestCommand = 0.0f;
    float bestCost =
        std::numeric_limits<float>::infinity();

    float step =
        std::max(
            config.candidateStep,
            0.5f
        );

    for (
        float candidate =
            -config.maximumVelocity;
        candidate <=
            config.maximumVelocity +
            0.001f;
        candidate += step
    )
    {
        float simulatedError =
            measuredError;

        float simulatedVelocity =
            state.modelVelocity;

        float simulatedCommand =
            state.previousCommand;

        std::deque<float> simulatedDelay =
            state.delayQueue;

        float cost = 0.0f;

        for (
            int k = 0;
            k < config.horizonSteps;
            ++k
        )
        {
            float previousSimulatedCommand =
                simulatedCommand;

            float maximumCommandChange =
                config.maximumAcceleration *
                dt;

            simulatedCommand +=
                std::clamp(
                    candidate -
                        simulatedCommand,
                    -maximumCommandChange,
                    maximumCommandChange
                );

            float appliedCommand =
                simulatedCommand;

            if (state.delaySteps > 0)
            {
                simulatedDelay.push_back(
                    simulatedCommand
                );

                appliedCommand =
                    simulatedDelay.front();

                simulatedDelay.pop_front();
            }

            float a =
                std::exp(
                    -dt /
                    std::max(
                        config.tauSeconds,
                        0.001f
                    )
                );

            float b =
                config.commandGain *
                (1.0f - a);

            simulatedVelocity =
                a * simulatedVelocity +
                b * appliedCommand;

            float predictedErrorRate =
                state.targetImageVelocity +
                config.cameraEffectSign *
                config.pixelsPerDegree *
                simulatedVelocity;

            simulatedError +=
                predictedErrorRate *
                dt;

            float normalizedError =
                simulatedError /
                100.0f;

            float normalizedControl =
                simulatedCommand /
                std::max(
                    config.maximumVelocity,
                    0.1f
                );

            float normalizedCommandChange =
                (
                    simulatedCommand -
                    previousSimulatedCommand
                ) /
                std::max(
                    config.maximumVelocity,
                    0.1f
                );

            cost +=
                config.qError *
                    normalizedError *
                    normalizedError +
                config.rControl *
                    normalizedControl *
                    normalizedControl +
                config.sCommandChange *
                    normalizedCommandChange *
                    normalizedCommandChange;
        }

        float terminalError =
            simulatedError /
            100.0f;

        cost +=
            2.0f *
            config.qError *
            terminalError *
            terminalError;

        if (cost < bestCost)
        {
            bestCost = cost;
            bestCommand = candidate;
        }
    }

    return bestCommand;
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
    constexpr float smoothingAlpha = 0.16f;

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

    // STATIONARY HOLD
    // Once the face has remained near center and nearly motionless for a
    // few control cycles, freeze both axes. This prevents detector noise
    // and observer noise from causing tiny continuous servo corrections.
    bool stationaryHold = false;
    int stationaryHoldCounter = 0;

    constexpr int stationaryHoldFrames = 5; // about 0.25 s at 20 Hz

    constexpr float stationaryEnterErrorX = 28.0f;
    constexpr float stationaryEnterErrorY = 22.0f;

    constexpr float stationaryExitErrorX = 50.0f;
    constexpr float stationaryExitErrorY = 40.0f;

    constexpr float stationaryEnterVelocityX = 28.0f; // pixels / second
    constexpr float stationaryEnterVelocityY = 24.0f;

    constexpr float stationaryExitVelocityX = 70.0f;
    constexpr float stationaryExitVelocityY = 60.0f;

    // LEGACY PID CONSTANTS (AUTO uses predictive MPC)
    // Pan is tuned separately because this axis was sluggish/occasionally reversed.
    float panKp = 0.022f;
    float panKi = 0.0015f;
    float panKd = 0.0020f;

    float tiltKp = 0.024f;
    float tiltKi = 0.0020f;
    float tiltKd = 0.0030f;

    // LEGACY PID STATE (kept for reset/manual compatibility)
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
    constexpr float nominalControlDt = 0.050f;

    auto previousControlTime =
        std::chrono::steady_clock::now();

    // IMAGE-PLANE PREDICTIVE MPC
    //
    // These are safe STARTING estimates, not measured plant parameters.
    // Tune tau, delay, and pixelsPerDegree later from logged experiments.
    AxisMPCConfig panMPC
    {
        0.24f,
        1.00f,
        0.10f,
        18.0f,
        +1.0f,
        1.00f,
        0.06f,
        0.28f,
        maximumVelocity,
        maximumAcceleration,
        10,
        3.0f
    };

    AxisMPCConfig tiltMPC
    {
        0.16f,
        1.00f,
        0.08f,
        18.0f,
        -1.0f,
        1.00f,
        0.07f,
        0.30f,
        maximumVelocity,
        maximumAcceleration,
        10,
        3.0f
    };

    AxisPredictiveState panPredictiveState;
    AxisPredictiveState tiltPredictiveState;

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

            // AUTO PREDICTIVE MPC CONTROL
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

                    advanceAxisObserver(
                        panPredictiveState,
                        errorX,
                        dt,
                        panMPC,
                        nominalControlDt
                    );

                    advanceAxisObserver(
                        tiltPredictiveState,
                        errorY,
                        dt,
                        tiltMPC,
                        nominalControlDt
                    );

                    // STATIONARY HOLD DETECTION
                    //
                    // Enter only after several consecutive quiet frames.
                    // Exit immediately when the face moves far enough or fast
                    // enough, so real-time tracking responsiveness is retained.
                    bool faceNearCenter =
                        std::abs(errorX) <= stationaryEnterErrorX &&
                        std::abs(errorY) <= stationaryEnterErrorY;

                    bool faceNearlyStill =
                        std::abs(
                            panPredictiveState.measuredImageVelocity
                        ) <= stationaryEnterVelocityX &&
                        std::abs(
                            tiltPredictiveState.measuredImageVelocity
                        ) <= stationaryEnterVelocityY;

                    bool faceClearlyMoved =
                        std::abs(errorX) >= stationaryExitErrorX ||
                        std::abs(errorY) >= stationaryExitErrorY ||
                        std::abs(
                            panPredictiveState.measuredImageVelocity
                        ) >= stationaryExitVelocityX ||
                        std::abs(
                            tiltPredictiveState.measuredImageVelocity
                        ) >= stationaryExitVelocityY;

                    if (stationaryHold)
                    {
                        if (faceClearlyMoved)
                        {
                            stationaryHold = false;
                            stationaryHoldCounter = 0;

                            // Start the prediction model fresh from the current
                            // measured location after leaving hold.
                            resetAxisPredictiveState(
                                panPredictiveState,
                                errorX,
                                panMPC,
                                nominalControlDt
                            );

                            resetAxisPredictiveState(
                                tiltPredictiveState,
                                errorY,
                                tiltMPC,
                                nominalControlDt
                            );
                        }
                    }
                    else
                    {
                        if (
                            faceNearCenter &&
                            faceNearlyStill
                        )
                        {
                            stationaryHoldCounter++;
                        }
                        else
                        {
                            stationaryHoldCounter = 0;
                        }

                        if (
                            stationaryHoldCounter >=
                            stationaryHoldFrames
                        )
                        {
                            stationaryHold = true;

                            panTrackingActive = false;
                            tiltTrackingActive = false;

                            panVelocity = 0.0f;
                            tiltVelocity = 0.0f;

                            panPredictiveState.modelVelocity = 0.0f;
                            tiltPredictiveState.modelVelocity = 0.0f;

                            panPredictiveState.targetImageVelocity = 0.0f;
                            tiltPredictiveState.targetImageVelocity = 0.0f;

                            panPredictiveState.measuredImageVelocity = 0.0f;
                            tiltPredictiveState.measuredImageVelocity = 0.0f;

                            panPredictiveState.previousCommand = 0.0f;
                            tiltPredictiveState.previousCommand = 0.0f;

                            // Match the internal targets to the last physical
                            // servo command so nothing continues drifting while
                            // the hold is active.
                            panCommand = panServoCommand;
                            tiltCommand = tiltServoCommand;
                        }
                    }

                    constexpr float displayPredictionSeconds = 0.18f;

                    float predictedErrorX =
                        errorX +
                        panPredictiveState.targetImageVelocity *
                        displayPredictionSeconds;

                    float predictedErrorY =
                        errorY +
                        tiltPredictiveState.targetImageVelocity *
                        displayPredictionSeconds;

                    constexpr float panStartVelocity = 110.0f;
                    constexpr float panStopVelocity = 28.0f;

                    constexpr float tiltStartVelocity = 90.0f;
                    constexpr float tiltStopVelocity = 24.0f;

                    if (!stationaryHold)
                    {
                        if (panTrackingActive)
                        {
                            if (
                                std::abs(errorX) <= panStopError &&
                                std::abs(
                                    panPredictiveState.targetImageVelocity
                                ) <= panStopVelocity
                            )
                            {
                                panTrackingActive = false;
                            }
                        }
                        else
                        {
                            if (
                                std::abs(predictedErrorX) >= panStartError ||
                                std::abs(
                                    panPredictiveState.targetImageVelocity
                                ) >= panStartVelocity
                            )
                            {
                                panTrackingActive = true;
                            }
                        }

                        if (tiltTrackingActive)
                        {
                            if (
                                std::abs(errorY) <= tiltStopError &&
                                std::abs(
                                    tiltPredictiveState.targetImageVelocity
                                ) <= tiltStopVelocity
                            )
                            {
                                tiltTrackingActive = false;
                            }
                        }
                        else
                        {
                            if (
                                std::abs(predictedErrorY) >= tiltStartError ||
                                std::abs(
                                    tiltPredictiveState.targetImageVelocity
                                ) >= tiltStartVelocity
                            )
                            {
                                tiltTrackingActive = true;
                            }
                        }

                    }
                    else
                    {
                        panTrackingActive = false;
                        tiltTrackingActive = false;
                    }

                    float desiredPanVelocity = 0.0f;
                    float desiredTiltVelocity = 0.0f;

                    if (panTrackingActive)
                    {
                        desiredPanVelocity =
                            solveAxisMPC(
                                errorX,
                                panPredictiveState,
                                panMPC,
                                nominalControlDt
                            );
                    }

                    if (tiltTrackingActive)
                    {
                        desiredTiltVelocity =
                            solveAxisMPC(
                                errorY,
                                tiltPredictiveState,
                                tiltMPC,
                                nominalControlDt
                            );
                    }

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

                    if (stationaryHold)
                    {
                        desiredPanVelocity = 0.0f;
                        desiredTiltVelocity = 0.0f;

                        panVelocity = 0.0f;
                        tiltVelocity = 0.0f;
                    }

                    float maximumVelocityChange =
                        maximumAcceleration *
                        dt;

                    panVelocity +=
                        std::clamp(
                            desiredPanVelocity -
                                panVelocity,
                            -maximumVelocityChange,
                            maximumVelocityChange
                        );

                    tiltVelocity +=
                        std::clamp(
                            desiredTiltVelocity -
                                tiltVelocity,
                            -maximumVelocityChange,
                            maximumVelocityChange
                        );

                    if (stationaryHold)
                    {
                        panVelocity = 0.0f;
                        tiltVelocity = 0.0f;
                    }

                    if (!panTrackingActive)
                    {
                        panVelocity *= 0.45f;

                        if (
                            std::abs(panVelocity) <
                            0.25f
                        )
                        {
                            panVelocity = 0.0f;
                        }
                    }

                    if (!tiltTrackingActive)
                    {
                        tiltVelocity *= 0.40f;

                        if (
                            std::abs(tiltVelocity) <
                            0.25f
                        )
                        {
                            tiltVelocity = 0.0f;
                        }
                    }

                    queueAxisCommand(
                        panPredictiveState,
                        panVelocity
                    );

                    queueAxisCommand(
                        tiltPredictiveState,
                        tiltVelocity
                    );

                    panCommand +=
                        panVelocity *
                        dt;

                    tiltCommand +=
                        tiltVelocity *
                        dt;

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

                    if (panCommand != unclampedPan)
                        panVelocity = 0.0f;

                    if (tiltCommand != unclampedTilt)
                        tiltVelocity = 0.0f;

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

                    int predictedX =
                        std::clamp(
                            static_cast<int>(
                                std::round(
                                    static_cast<float>(
                                        frameCenterX
                                    ) +
                                    predictedErrorX
                                )
                            ),
                            0,
                            frame.cols - 1
                        );

                    int predictedY =
                        std::clamp(
                            static_cast<int>(
                                std::round(
                                    static_cast<float>(
                                        frameCenterY
                                    ) +
                                    predictedErrorY
                                )
                            ),
                            0,
                            frame.rows - 1
                        );

                    cv::circle(
                        frame,
                        cv::Point(
                            predictedX,
                            predictedY
                        ),
                        7,
                        cv::Scalar(255, 0, 255),
                        2
                    );

                    cv::putText(
                        frame,
                        "Prediction",
                        cv::Point(
                            std::min(
                                predictedX + 10,
                                frame.cols - 120
                            ),
                            std::max(
                                predictedY - 10,
                                20
                            )
                        ),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.45,
                        cv::Scalar(255, 0, 255),
                        1
                    );

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
                        << " ErrX="
                        << errorX
                        << " ErrY="
                        << errorY
                        << " TargetVx="
                        << panPredictiveState.targetImageVelocity
                        << " TargetVy="
                        << tiltPredictiveState.targetImageVelocity
                        << " PanModelV="
                        << panPredictiveState.modelVelocity
                        << " TiltModelV="
                        << tiltPredictiveState.modelVelocity
                        << " MPCPan="
                        << desiredPanVelocity
                        << " MPCTilt="
                        << desiredTiltVelocity
                        << " Hold="
                        << (
                            stationaryHold
                            ? "YES"
                            : "NO"
                        )
                        << '\n';
                }
            }
        }
        else
        {
            lostFrames++;

            panTrackingActive = false;
            tiltTrackingActive = false;

            stationaryHold = false;
            stationaryHoldCounter = 0;

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

                panPredictiveState.initialized = false;
                tiltPredictiveState.initialized = false;

                panPredictiveState.delayQueue.clear();
                tiltPredictiveState.delayQueue.clear();
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
            : "AUTO MPC | M manual";

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

            stationaryHold = false;
            stationaryHoldCounter = 0;

            panPredictiveState.initialized = false;
            tiltPredictiveState.initialized = false;

            panPredictiveState.delayQueue.clear();
            tiltPredictiveState.delayQueue.clear();

            previousControlTime =
                std::chrono::steady_clock::now();

            std::cout
                << (
                    manualMode
                    ? "MANUAL MODE\n"
                    : "AUTO MPC MODE\n"
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

            panPredictiveState.initialized = false;
            tiltPredictiveState.initialized = false;

            panPredictiveState.delayQueue.clear();
            tiltPredictiveState.delayQueue.clear();

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