#include <Servo.h>

Servo panServo;
Servo tiltServo;

int panAngle = 90;
int tiltAngle = 90;

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(50);

    panServo.attach(9);
    tiltServo.attach(10);

    panServo.write(panAngle);
    tiltServo.write(tiltAngle);

    Serial.println("Ready");
}

void loop()
{
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        int commaPosition = command.indexOf(',');

        if (commaPosition == -1) {
            return;
        }

        String panText =
            command.substring(0, commaPosition);

        String tiltText =
            command.substring(commaPosition + 1);

        panAngle = constrain(
            panText.toInt(),
            0,
            180
        );

        tiltAngle = constrain(
            tiltText.toInt(),
            0,
            180
        );

        panServo.write(panAngle);
        tiltServo.write(tiltAngle);

        Serial.print("Pan: ");
        Serial.print(panAngle);
        Serial.print(" Tilt: ");
        Serial.println(tiltAngle);
    }
}