#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>

#include "serial/serial.h"


int main()
{
    // =========================================================
    // WEBCAM
    // =========================================================

    cv::VideoCapture cap(0, cv::CAP_DSHOW);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open webcam.\n";
        return 1;
    }

    // Optional webcam resolution.
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);


    // =========================================================
    // YOLO MODEL
    // =========================================================

    cv::dnn::Net net =
        cv::dnn::readNetFromONNX("Jerison_face.onnx");

    if (net.empty()) {
        std::cerr << "Error: Could not load YOLO model.\n";
        return 1;
    }

    // Prefer CUDA if your OpenCV build supports it.
    // If your OpenCV was not compiled with CUDA DNN,
    // leave these commented out.
    //
    // net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    // net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    std::cout << "YOLO face model loaded successfully!\n";


    // =========================================================
    // ARDUINO
    // =========================================================

    serial::Serial arduino;

    try {
        arduino.setPort("COM5");
        arduino.setBaudrate(115200);

        serial::Timeout timeout =
            serial::Timeout::simpleTimeout(1000);

        arduino.setTimeout(timeout);
        arduino.open();
    }
    catch (const std::exception& error) {
        std::cerr
            << "Arduino connection error: "
            << error.what()
            << '\n';

        return 1;
    }

    if (!arduino.isOpen()) {
        std::cerr << "Arduino not connected!\n";
        return 1;
    }

    std::cout << "Arduino connected!\n";

    // Arduino Uno often resets when serial connection opens.
    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );


    // =========================================================
    // SETTINGS
    // =========================================================

    constexpr int inputWidth = 640;
    constexpr int inputHeight = 640;

    constexpr float confidenceThreshold = 0.50f;
    constexpr float nmsThreshold = 0.45f;

    constexpr int deadZone = 30;

    float panKp = 0.03f;
    float tiltKp = 0.03f;

    int panAngle = 90;
    int tiltAngle = 90;

    int previousPanAngle = -1;
    int previousTiltAngle = -1;

    // Revision M: bound cable travel and command speed. Calibrate actual angles.
    auto lastServoCommand = std::chrono::steady_clock::now()
        - std::chrono::milliseconds(33);

    bool printedOutputShape = false;

    cv::Mat frame;


    // =========================================================
    // MAIN LOOP
    // =========================================================

    while (true) {

        if (!cap.read(frame) || frame.empty()) {
            std::cerr
                << "Error: Failed to read webcam frame.\n";
            break;
        }


        // =====================================================
        // LETTERBOX IMAGE TO 640 x 640
        // =====================================================

        float scale = std::min(
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
            inputWidth - resizedWidth - padX;

        int padBottom =
            inputHeight - resizedHeight - padY;

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


        // =====================================================
        // CREATE YOLO INPUT
        // =====================================================

        cv::Mat blob;

        cv::dnn::blobFromImage(
            letterboxed,
            blob,
            1.0 / 255.0,
            cv::Size(inputWidth, inputHeight),
            cv::Scalar(),
            true,       // BGR -> RGB
            false
        );

        net.setInput(blob);


        // =====================================================
        // YOLO FORWARD PASS
        // =====================================================

        std::vector<cv::Mat> outputs;

        net.forward(
            outputs,
            net.getUnconnectedOutLayersNames()
        );

        if (outputs.empty()) {
            std::cerr
                << "Error: YOLO returned no output.\n";
            break;
        }

        cv::Mat output = outputs[0];


        // =====================================================
        // PRINT OUTPUT SHAPE ONCE
        // =====================================================

        if (!printedOutputShape) {

            std::cout << "ONNX output dimensions: ";

            for (int i = 0; i < output.dims; i++) {
                std::cout << output.size[i];

                if (i < output.dims - 1) {
                    std::cout << " x ";
                }
            }

            std::cout << '\n';

            printedOutputShape = true;
        }


        // =====================================================
        // PARSE CUSTOM YOLO OUTPUT
        //
        // Expected:
        //
        // 1 x 5 x 8400
        //
        // 0 = center X
        // 1 = center Y
        // 2 = width
        // 3 = height
        // 4 = jerison_face confidence
        // =====================================================

        if (output.dims != 3) {
            std::cerr
                << "Unexpected YOLO output dimensions.\n";
            break;
        }

        int dimensions;
        int detections;

        cv::Mat detectionData;

        // Typical YOLOv8 output:
        // [1, 5, 8400]
        if (output.size[1] < output.size[2]) {

            dimensions = output.size[1];
            detections = output.size[2];

            cv::Mat raw(
                dimensions,
                detections,
                CV_32F,
                output.ptr<float>()
            );

            cv::transpose(
                raw,
                detectionData
            );
        }

        // Handle alternate:
        // [1, 8400, 5]
        else {

            detections = output.size[1];
            dimensions = output.size[2];

            detectionData = cv::Mat(
                detections,
                dimensions,
                CV_32F,
                output.ptr<float>()
            );
        }


        if (dimensions != 5) {

            std::cerr
                << "Unexpected number of YOLO values per detection: "
                << dimensions
                << "\nExpected 5 for a one-class YOLOv8 model.\n";

            break;
        }


        // =====================================================
        // EXTRACT BOXES
        // =====================================================

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;

        for (int i = 0; i < detections; i++) {

            const float* data =
                detectionData.ptr<float>(i);

            float centerX = data[0];
            float centerY = data[1];
            float width = data[2];
            float height = data[3];

            // Only one class:
            // jerison_face
            float confidence = data[4];

            if (confidence < confidenceThreshold) {
                continue;
            }


            // =================================================
            // YOLO coordinates are relative to LETTERBOX image.
            //
            // Remove padding, then scale back to webcam image.
            // =================================================

            float leftLetterbox =
                centerX - width / 2.0f;

            float topLetterbox =
                centerY - height / 2.0f;

            float originalLeft =
                (leftLetterbox - padX) / scale;

            float originalTop =
                (topLetterbox - padY) / scale;

            float originalWidth =
                width / scale;

            float originalHeight =
                height / scale;


            int left =
                static_cast<int>(
                    std::round(originalLeft)
                );

            int top =
                static_cast<int>(
                    std::round(originalTop)
                );

            int boxWidth =
                static_cast<int>(
                    std::round(originalWidth)
                );

            int boxHeight =
                static_cast<int>(
                    std::round(originalHeight)
                );


            cv::Rect box(
                left,
                top,
                boxWidth,
                boxHeight
            );


            // Keep box inside webcam image.
            box &= cv::Rect(
                0,
                0,
                frame.cols,
                frame.rows
            );


            if (
                box.width <= 0 ||
                box.height <= 0
            ) {
                continue;
            }


            boxes.push_back(box);
            scores.push_back(confidence);
        }


        // =====================================================
        // NON-MAXIMUM SUPPRESSION
        // =====================================================

        std::vector<int> indices;

        cv::dnn::NMSBoxes(
            boxes,
            scores,
            confidenceThreshold,
            nmsThreshold,
            indices
        );


        // =====================================================
        // FRAME CENTER
        // =====================================================

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
            5,
            cv::Scalar(255, 0, 0),
            -1
        );


        // =====================================================
        // CHOOSE TARGET
        //
        // For now: largest detected face.
        //
        // IMPORTANT:
        // Your current model is NOT yet a reliable identity
        // recognizer. Another person's face may be detected too.
        // =====================================================

        int targetIndex = -1;
        int largestArea = 0;

        for (int index : indices) {

            int area =
                boxes[index].area();

            if (area > largestArea) {

                largestArea = area;
                targetIndex = index;
            }
        }


        // =====================================================
        // TRACK TARGET FACE
        // =====================================================

        if (targetIndex != -1) {

            cv::Rect box =
                boxes[targetIndex];

            float targetConfidence =
                scores[targetIndex];


            // Draw face box
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
                    std::max(20, box.y - 10)
                ),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0, 255, 0),
                2
            );


            // =================================================
            // FACE CENTER
            // =================================================

            int faceCenterX =
                box.x + box.width / 2;

            int faceCenterY =
                box.y + box.height / 2;


            int errorX =
                faceCenterX - frameCenterX;

            int errorY =
                faceCenterY - frameCenterY;


            cv::circle(
                frame,
                cv::Point(
                    faceCenterX,
                    faceCenterY
                ),
                5,
                cv::Scalar(0, 0, 255),
                -1
            );


            // Draw line between target and center.
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


            // =================================================
            // PAN CONTROL
            // =================================================

            if (std::abs(errorX) > deadZone) {

                int panChange =
                    static_cast<int>(
                        panKp * errorX
                    );

                panChange =
                    std::clamp(
                        panChange,
                        -3,
                        3
                    );

                panAngle += panChange;
            }


            // =================================================
            // TILT CONTROL
            // =================================================

            if (std::abs(errorY) > deadZone) {

                int tiltChange =
                    static_cast<int>(
                        tiltKp * errorY
                    );

                tiltChange =
                    std::clamp(
                        tiltChange,
                        -3,
                        3
                    );

                /*
                 * If vertical movement goes the wrong direction,
                 * change:
                 *
                 * tiltAngle -= tiltChange;
                 *
                 * to:
                 *
                 * tiltAngle += tiltChange;
                 */

                tiltAngle -= tiltChange;
            }


            // =================================================
            // SERVO LIMITS
            // =================================================

            panAngle =
                std::clamp(
                    panAngle,
                    45,
                    135
                );

            tiltAngle =
                std::clamp(
                    tiltAngle,
                    45,
                    135
                );


            // =================================================
            // SEND TO ARDUINO
            // =================================================

            const auto servoCommandNow = std::chrono::steady_clock::now();
            if ((panAngle != previousPanAngle || tiltAngle != previousTiltAngle)
                && servoCommandNow - lastServoCommand >= std::chrono::milliseconds(33)) {
                const int lastPan = previousPanAngle < 0 ? 90 : previousPanAngle;
                const int lastTilt = previousTiltAngle < 0 ? 90 : previousTiltAngle;
                panAngle = std::clamp(panAngle, lastPan - 1, lastPan + 1);
                tiltAngle = std::clamp(tiltAngle, lastTilt - 1, lastTilt + 1);
                lastServoCommand = servoCommandNow;

                std::string command =
                    std::to_string(panAngle)
                    + ","
                    + std::to_string(tiltAngle)
                    + "\n";

                arduino.write(command);

                previousPanAngle =
                    panAngle;

                previousTiltAngle =
                    tiltAngle;


                std::cout
                    << "Sent: "
                    << command
                    << "errorX: "
                    << errorX
                    << " errorY: "
                    << errorY
                    << " confidence: "
                    << targetConfidence
                    << '\n';
            }
        }


        // =====================================================
        // DISPLAY INFO
        // =====================================================

        cv::putText(
            frame,
            "Pan: " + std::to_string(panAngle),
            cv::Point(20, 35),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2
        );

        cv::putText(
            frame,
            "Tilt: " + std::to_string(tiltAngle),
            cv::Point(20, 65),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2
        );

        cv::putText(
            frame,
            "Faces: " + std::to_string(indices.size()),
            cv::Point(20, 95),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 0),
            2
        );


        cv::imshow(
            "Custom YOLO Face Pan-Tilt Tracker",
            frame
        );


        int key =
            cv::waitKey(1) & 0xFF;

        if (
            key == 'q' ||
            key == 27
        ) {
            break;
        }
    }


    // =========================================================
    // CLEANUP
    // =========================================================

    if (arduino.isOpen()) {
        arduino.close();
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
