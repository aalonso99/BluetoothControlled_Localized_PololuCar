#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

#define FORWARD_SPEED 150
#define BACKWARD_SPEED 150
#define ROTATION_SPEED 250

const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";

char report[80];
char command;

void displayReport() {
    static uint8_t lastDisplayTime;
    static uint8_t displayErrorLeftCountdown = 0;
    static uint8_t displayErrorRightCountdown = 0;

    if ((uint8_t)(millis() - lastDisplayTime) >= 100) {
        lastDisplayTime = millis();

        int16_t countsLeft = encoders.getCountsLeft();
        int16_t countsRight = encoders.getCountsRight();

        bool errorLeft = encoders.checkErrorLeft();
        bool errorRight = encoders.checkErrorRight();

        // Send the information to the serial monitor also.
        snprintf_P(report, sizeof(report),
            PSTR("%6d %6d %6d %6d"),
            countsLeft, countsRight, errorLeft, errorRight);
        
            
        Serial.println(report);
    }
}

void setup() {

    Serial.begin(9600);
    Serial1.begin(115200);

}

void loop() {
    
    if (Serial1.available() > 0) {

        //// READ COMMAND FROM ESP32
        command = Serial1.read(); 
        Serial.println(command);

        //// RUN COMMAND
        if (command == '0') {
            motors.setSpeeds(0, 0);
        }
        else if (command == '1') {
            motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
        }
        else if (command == '2'){
            motors.setSpeeds(-BACKWARD_SPEED, -BACKWARD_SPEED);
        }
        else if (command == '3'){
            motors.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
        }
        else if (command == '4'){
            motors.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
        } else {
            motors.setSpeeds(0, 0);
        }

    }

}
