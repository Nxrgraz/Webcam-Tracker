#include <Servo.h>

Servo panServo;
Servo tiltServo;

constexpr int PAN_PIN = 9;
constexpr int TILT_PIN = 10;

constexpr float PAN_MIN_DEG = 15.0f;
constexpr float PAN_MAX_DEG = 165.0f;
constexpr float TILT_MIN_DEG = 60.0f;
constexpr float TILT_MAX_DEG = 120.0f;

// Arduino Servo library's normal write(0..180) range.
// Using microseconds lets us keep sub-degree commands from the PC.
constexpr int SERVO_MIN_US = 544;
constexpr int SERVO_MAX_US = 2400;

float panAngle = 90.0f;
float tiltAngle = 90.0f;

int angleToMicroseconds(float angle)
{
    angle = constrain(angle, 0.0f, 180.0f);

    float fraction = angle / 180.0f;

    return static_cast<int>(
        round(
            SERVO_MIN_US +
            fraction *
            (SERVO_MAX_US - SERVO_MIN_US)
        )
    );
}

void writeServos()
{
    panServo.writeMicroseconds(
        angleToMicroseconds(panAngle)
    );

    tiltServo.writeMicroseconds(
        angleToMicroseconds(tiltAngle)
    );
}

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(20);

    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);

    writeServos();

    Serial.println("Ready");
}

void loop()
{
    if (Serial.available() <= 0)
        return;

    String command =
        Serial.readStringUntil('\n');

    command.trim();

    int commaPosition =
        command.indexOf(',');

    if (commaPosition < 0)
        return;

    float requestedPan =
        command
            .substring(0, commaPosition)
            .toFloat();

    float requestedTilt =
        command
            .substring(commaPosition + 1)
            .toFloat();

    panAngle =
        constrain(
            requestedPan,
            PAN_MIN_DEG,
            PAN_MAX_DEG
        );

    tiltAngle =
        constrain(
            requestedTilt,
            TILT_MIN_DEG,
            TILT_MAX_DEG
        );

    writeServos();
}
