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
    // CAMERA
    // =========================================================

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

    // Try to remove digital zoom
    cap.set(cv::CAP_PROP_ZOOM, 0);

    std::cout << "Camera opened successfully!\n";
    std::cout << "Camera index: " << cameraIndex << "\n";


    // =========================================================
    // YOLO MODEL
    // =========================================================

    cv::dnn::Net net =
        cv::dnn::readNetFromONNX(
            "Jerison_face.onnx"
        );

    if (net.empty())
    {
        std::cerr << "ERROR: Could not load YOLO model.\n";
        return 1;
    }

    std::cout << "YOLO face model loaded successfully!\n";


    // =========================================================
    // ARDUINO
    // =========================================================

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
        std::cerr << "ERROR: Arduino not connected!\n";
        return 1;
    }

    std::cout << "Arduino connected on COM7!\n";

    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );


    // =========================================================
    // YOLO SETTINGS
    // =========================================================

    constexpr int inputWidth = 640;
    constexpr int inputHeight = 640;

    constexpr float confidenceThreshold = 0.40f;

    constexpr float nmsThreshold = 0.45f;


    // =========================================================
    // AUTO TRACKING SETTINGS
    // =========================================================

    constexpr int deadZone = 50;

    float panKp = 0.015f;
    float tiltKp = 0.015f;

    constexpr int maximumServoStep = 2;


    // =========================================================
    // FACE SMOOTHING
    // =========================================================

    constexpr float smoothingAlpha = 0.15f;

    float smoothedFaceX = 0.0f;
    float smoothedFaceY = 0.0f;

    bool firstDetection = true;

    int lostFrames = 0;

    constexpr int resetSmoothingAfterLostFrames = 10;


    // =========================================================
    // SERVO UPDATE RATE
    // =========================================================

    constexpr int servoUpdateIntervalMs = 50;

    auto lastServoUpdate =
        std::chrono::steady_clock::now();


    // =========================================================
    // SERVO VARIABLES
    // =========================================================

    int panAngle = 90;
    int tiltAngle = 90;

    int previousPanAngle = -1;
    int previousTiltAngle = -1;


    // =========================================================
    // MANUAL MODE
    // =========================================================

    bool manualMode = false;

    constexpr int manualServoStep = 2;


    // =========================================================
    // OTHER VARIABLES
    // =========================================================

    bool printedOutputShape = false;

    cv::Mat frame;


    // =========================================================
    // MAIN LOOP
    // =========================================================

    while (true)
    {
        // =====================================================
        // READ CAMERA
        // =====================================================

        if (!cap.read(frame) || frame.empty())
        {
            std::cerr
                << "ERROR: Failed to read webcam frame.\n";

            break;
        }


        // =====================================================
        // LETTERBOX IMAGE TO 640 x 640
        // =====================================================

        float scale =
            std::min(
                static_cast<float>(inputWidth) /
                    static_cast<float>(frame.cols),

                static_cast<float>(inputHeight) /
                    static_cast<float>(frame.rows)
            );


        int resizedWidth =
            static_cast<int>(
                std::round(
                    frame.cols * scale
                )
            );


        int resizedHeight =
            static_cast<int>(
                std::round(
                    frame.rows * scale
                )
            );


        cv::Mat resized;

        cv::resize(
            frame,
            resized,

            cv::Size(
                resizedWidth,
                resizedHeight
            )
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

            cv::Scalar(
                114,
                114,
                114
            )
        );


        // =====================================================
        // YOLO INPUT
        // =====================================================

        cv::Mat blob;

        cv::dnn::blobFromImage(
            letterboxed,
            blob,

            1.0 / 255.0,

            cv::Size(
                inputWidth,
                inputHeight
            ),

            cv::Scalar(),

            true,
            false
        );

        net.setInput(blob);


        // =====================================================
        // RUN YOLO
        // =====================================================

        std::vector<cv::Mat> outputs;

        net.forward(
            outputs,
            net.getUnconnectedOutLayersNames()
        );


        if (outputs.empty())
        {
            std::cerr
                << "ERROR: YOLO returned no output.\n";

            break;
        }


        cv::Mat output =
            outputs[0];


        // =====================================================
        // PRINT OUTPUT SHAPE ONCE
        // =====================================================

        if (!printedOutputShape)
        {
            std::cout
                << "ONNX output dimensions: ";

            for (
                int i = 0;
                i < output.dims;
                i++
            )
            {
                std::cout
                    << output.size[i];

                if (i < output.dims - 1)
                {
                    std::cout
                        << " x ";
                }
            }

            std::cout
                << '\n';

            printedOutputShape = true;
        }


        // =====================================================
        // PARSE YOLO OUTPUT
        // =====================================================

        if (output.dims != 3)
        {
            std::cerr
                << "ERROR: Unexpected YOLO dimensions.\n";

            break;
        }


        int dimensions;
        int detections;

        cv::Mat detectionData;


        if (output.size[1] < output.size[2])
        {
            dimensions =
                output.size[1];

            detections =
                output.size[2];


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

        else
        {
            detections =
                output.size[1];

            dimensions =
                output.size[2];


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
                << "ERROR: Expected 5 YOLO values per detection.\n";

            break;
        }


        // =====================================================
        // EXTRACT DETECTIONS
        // =====================================================

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;


        for (
            int i = 0;
            i < detections;
            i++
        )
        {
            const float* data =
                detectionData.ptr<float>(i);


            float centerX =
                data[0];

            float centerY =
                data[1];

            float width =
                data[2];

            float height =
                data[3];

            float confidence =
                data[4];


            if (
                confidence <
                confidenceThreshold
            )
            {
                continue;
            }


            float leftLetterbox =
                centerX -
                width / 2.0f;

            float topLetterbox =
                centerY -
                height / 2.0f;


            float originalLeft =
                (leftLetterbox - padX)
                / scale;

            float originalTop =
                (topLetterbox - padY)
                / scale;

            float originalWidth =
                width / scale;

            float originalHeight =
                height / scale;


            int left =
                static_cast<int>(
                    std::round(
                        originalLeft
                    )
                );


            int top =
                static_cast<int>(
                    std::round(
                        originalTop
                    )
                );


            int boxWidth =
                static_cast<int>(
                    std::round(
                        originalWidth
                    )
                );


            int boxHeight =
                static_cast<int>(
                    std::round(
                        originalHeight
                    )
                );


            cv::Rect box(
                left,
                top,
                boxWidth,
                boxHeight
            );


            box &=
                cv::Rect(
                    0,
                    0,
                    frame.cols,
                    frame.rows
                );


            if (
                box.width <= 0 ||
                box.height <= 0
            )
            {
                continue;
            }


            boxes.push_back(box);

            scores.push_back(confidence);
        }


        // =====================================================
        // NON MAXIMUM SUPPRESSION
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
        // CAMERA CENTER
        // =====================================================

        int frameCenterX =
            frame.cols / 2;

        int frameCenterY =
            frame.rows / 2;


        // Blue point = center of image
        cv::circle(
            frame,

            cv::Point(
                frameCenterX,
                frameCenterY
            ),

            6,

            cv::Scalar(
                255,
                0,
                0
            ),

            -1
        );


        // =====================================================
        // DEAD ZONE
        // =====================================================

        cv::rectangle(
            frame,

            cv::Point(
                frameCenterX - deadZone,
                frameCenterY - deadZone
            ),

            cv::Point(
                frameCenterX + deadZone,
                frameCenterY + deadZone
            ),

            cv::Scalar(
                255,
                255,
                0
            ),

            2
        );


        // =====================================================
        // SELECT TARGET
        // =====================================================

        int targetIndex = -1;
        int largestArea = 0;


        for (int index : indices)
        {
            int area =
                boxes[index].area();


            if (area > largestArea)
            {
                largestArea =
                    area;

                targetIndex =
                    index;
            }
        }


        // =====================================================
        // TARGET FOUND
        // =====================================================

        if (targetIndex != -1)
        {
            lostFrames = 0;


            cv::Rect box =
                boxes[targetIndex];


            float targetConfidence =
                scores[targetIndex];


            // =================================================
            // FACE BOX
            // =================================================

            cv::rectangle(
                frame,
                box,

                cv::Scalar(
                    0,
                    255,
                    0
                ),

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

                cv::Scalar(
                    0,
                    255,
                    0
                ),

                2
            );


            // =================================================
            // RAW FACE CENTER
            // =================================================

            int rawFaceCenterX =
                box.x +
                box.width / 2;

            int rawFaceCenterY =
                box.y +
                box.height / 2;


            // Red dot = raw YOLO position
            cv::circle(
                frame,

                cv::Point(
                    rawFaceCenterX,
                    rawFaceCenterY
                ),

                4,

                cv::Scalar(
                    0,
                    0,
                    255
                ),

                -1
            );


            // =================================================
            // SMOOTH FACE POSITION
            // =================================================

            if (firstDetection)
            {
                smoothedFaceX =
                    static_cast<float>(
                        rawFaceCenterX
                    );

                smoothedFaceY =
                    static_cast<float>(
                        rawFaceCenterY
                    );

                firstDetection =
                    false;
            }

            else
            {
                smoothedFaceX =
                    smoothingAlpha *
                    rawFaceCenterX

                    +

                    (1.0f - smoothingAlpha) *
                    smoothedFaceX;


                smoothedFaceY =
                    smoothingAlpha *
                    rawFaceCenterY

                    +

                    (1.0f - smoothingAlpha) *
                    smoothedFaceY;
            }


            int faceCenterX =
                static_cast<int>(
                    std::round(
                        smoothedFaceX
                    )
                );


            int faceCenterY =
                static_cast<int>(
                    std::round(
                        smoothedFaceY
                    )
                );


            // Yellow dot = smoothed tracking position
            cv::circle(
                frame,

                cv::Point(
                    faceCenterX,
                    faceCenterY
                ),

                6,

                cv::Scalar(
                    0,
                    255,
                    255
                ),

                -1
            );


            // =================================================
            // ERROR
            // =================================================

            int errorX =
                faceCenterX -
                frameCenterX;

            int errorY =
                faceCenterY -
                frameCenterY;


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

                cv::Scalar(
                    255,
                    255,
                    0
                ),

                2
            );


            // =================================================
            // AUTOMATIC TRACKING
            // =================================================

            if (!manualMode)
            {
                auto now =
                    std::chrono::steady_clock::now();


                auto elapsedMilliseconds =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds
                    >(
                        now -
                        lastServoUpdate
                    ).count();


                if (
                    elapsedMilliseconds >=
                    servoUpdateIntervalMs
                )
                {
                    // =========================================
                    // PAN
                    // =========================================

                    if (
                        std::abs(errorX) >
                        deadZone
                    )
                    {
                        int panChange =
                            static_cast<int>(
                                std::round(
                                    panKp *
                                    errorX
                                )
                            );


                        panChange =
                            std::clamp(
                                panChange,
                                -maximumServoStep,
                                maximumServoStep
                            );


                        // IMPORTANT
                        //
                        // INVERTED because your webcam
                        // is physically mounted reversed.

                        panAngle -=
                            panChange;
                    }


                    // =========================================
                    // TILT
                    // =========================================

                    if (
                        std::abs(errorY) >
                        deadZone
                    )
                    {
                        int tiltChange =
                            static_cast<int>(
                                std::round(
                                    tiltKp *
                                    errorY
                                )
                            );


                        tiltChange =
                            std::clamp(
                                tiltChange,
                                -maximumServoStep,
                                maximumServoStep
                            );


                        tiltAngle -=
                            tiltChange;
                    }


                    // =========================================
                    // SERVO LIMITS
                    // =========================================

                    panAngle =
                        std::clamp(
                            panAngle,
                            0,
                            180
                        );


                    tiltAngle =
                        std::clamp(
                            tiltAngle,
                            0,
                            180
                        );


                    // =========================================
                    // SEND AUTO COMMAND
                    // =========================================

                    if (
                        panAngle !=
                            previousPanAngle ||

                        tiltAngle !=
                            previousTiltAngle
                    )
                    {
                        std::string command =
                            std::to_string(
                                panAngle
                            )

                            + ","

                            + std::to_string(
                                tiltAngle
                            )

                            + "\n";


                        arduino.write(command);


                        previousPanAngle =
                            panAngle;

                        previousTiltAngle =
                            tiltAngle;


                        std::cout
                            << "AUTO -> Pan: "
                            << panAngle

                            << " Tilt: "
                            << tiltAngle

                            << " ErrorX: "
                            << errorX

                            << " ErrorY: "
                            << errorY

                            << " Confidence: "
                            << targetConfidence

                            << '\n';
                    }


                    lastServoUpdate =
                        now;
                }
            }
        }


        // =====================================================
        // NO FACE FOUND
        // =====================================================

        else
        {
            lostFrames++;


            if (
                lostFrames >=
                resetSmoothingAfterLostFrames
            )
            {
                firstDetection =
                    true;
            }
        }


        // =====================================================
        // DISPLAY PAN
        // =====================================================

        cv::putText(
            frame,

            "Pan: " +
            std::to_string(
                panAngle
            ),

            cv::Point(
                20,
                35
            ),

            cv::FONT_HERSHEY_SIMPLEX,

            0.7,

            cv::Scalar(
                0,
                255,
                0
            ),

            2
        );


        // =====================================================
        // DISPLAY TILT
        // =====================================================

        cv::putText(
            frame,

            "Tilt: " +
            std::to_string(
                tiltAngle
            ),

            cv::Point(
                20,
                65
            ),

            cv::FONT_HERSHEY_SIMPLEX,

            0.7,

            cv::Scalar(
                0,
                255,
                0
            ),

            2
        );


        // =====================================================
        // DISPLAY FACE COUNT
        // =====================================================

        cv::putText(
            frame,

            "Faces: " +
            std::to_string(
                indices.size()
            ),

            cv::Point(
                20,
                95
            ),

            cv::FONT_HERSHEY_SIMPLEX,

            0.7,

            cv::Scalar(
                0,
                255,
                0
            ),

            2
        );


        // =====================================================
        // DISPLAY MODE
        // =====================================================

        std::string modeText;


        if (manualMode)
        {
            modeText =
                "MANUAL | W/S Tilt | A/D Pan | R Center | M Auto";
        }

        else
        {
            modeText =
                "AUTO TRACKING | M = Manual";
        }


        cv::putText(
            frame,
            modeText,

            cv::Point(
                20,
                125
            ),

            cv::FONT_HERSHEY_SIMPLEX,

            0.55,

            cv::Scalar(
                0,
                255,
                255
            ),

            2
        );


        // =====================================================
        // SHOW CAMERA
        // =====================================================

        cv::imshow(
            "Custom YOLO Face Pan-Tilt Tracker",
            frame
        );


        // =====================================================
        // KEYBOARD
        // =====================================================

        int key =
            cv::waitKey(1) &
            0xFF;


        // =====================================================
        // M = AUTO / MANUAL
        // =====================================================

        if (
            key == 'm' ||
            key == 'M'
        )
        {
            manualMode =
                !manualMode;


            if (manualMode)
            {
                std::cout
                    << "\n============================\n";

                std::cout
                    << "MANUAL MODE ENABLED\n";

                std::cout
                    << "W = Tilt Up\n";

                std::cout
                    << "S = Tilt Down\n";

                std::cout
                    << "A = Pan Left\n";

                std::cout
                    << "D = Pan Right\n";

                std::cout
                    << "R = Center Camera\n";

                std::cout
                    << "M = Automatic Mode\n";

                std::cout
                    << "============================\n\n";
            }

            else
            {
                std::cout
                    << "\nAUTO TRACKING ENABLED\n";

                firstDetection =
                    true;
            }
        }


        // =====================================================
        // MANUAL CONTROL
        // =====================================================

        if (manualMode)
        {
            bool servoChanged =
                false;


            // =================================================
            // W = TILT UP
            // =================================================

            if (
                key == 'w' ||
                key == 'W'
            )
            {
                tiltAngle +=
                    manualServoStep;

                servoChanged =
                    true;
            }


            // =================================================
            // S = TILT DOWN
            // =================================================

            if (
                key == 's' ||
                key == 'S'
            )
            {
                tiltAngle -=
                    manualServoStep;

                servoChanged =
                    true;
            }


            // =================================================
            // A = PAN LEFT
            //
            // INVERTED because your webcam is mounted backwards
            // =================================================

            if (
                key == 'a' ||
                key == 'A'
            )
            {
                panAngle +=
                    manualServoStep;

                servoChanged =
                    true;
            }


            // =================================================
            // D = PAN RIGHT
            //
            // INVERTED because your webcam is mounted backwards
            // =================================================

            if (
                key == 'd' ||
                key == 'D'
            )
            {
                panAngle -=
                    manualServoStep;

                servoChanged =
                    true;
            }


            // =================================================
            // R = CENTER
            // =================================================

            if (
                key == 'r' ||
                key == 'R'
            )
            {
                panAngle = 90;
                tiltAngle = 90;

                servoChanged =
                    true;
            }


            // =================================================
            // SERVO LIMITS
            // =================================================

            panAngle =
                std::clamp(
                    panAngle,
                    0,
                    180
                );


            tiltAngle =
                std::clamp(
                    tiltAngle,
                    0,
                    180
                );


            // =================================================
            // SEND MANUAL COMMAND
            // =================================================

            if (servoChanged)
            {
                std::string command =
                    std::to_string(
                        panAngle
                    )

                    + ","

                    + std::to_string(
                        tiltAngle
                    )

                    + "\n";


                arduino.write(command);


                previousPanAngle =
                    panAngle;

                previousTiltAngle =
                    tiltAngle;


                std::cout
                    << "MANUAL -> Pan: "
                    << panAngle

                    << " Tilt: "
                    << tiltAngle

                    << '\n';
            }
        }


        // =====================================================
        // QUIT
        // =====================================================

        if (
            key == 'q' ||
            key == 'Q' ||
            key == 27
        )
        {
            break;
        }
    }


    // =========================================================
    // CLEANUP
    // =========================================================

    if (arduino.isOpen())
    {
        arduino.close();
    }


    cap.release();

    cv::destroyAllWindows();

    return 0;
}