#include <BluetoothSerial.h>
#include <RPLidar.h>

#define RXD2 16
#define TXD2 17
// The PWM pin for control the speed of RPLIDAR's motor.
// This pin should connected with the RPLIDAR's MOTOCTRL signal 
#define RPLIDAR_MOTOR 22 
// #define RPLIDAR_MOTOR 21
#define SAMPLE_SIZE 2

#define MAX_REPORT_SIZE 9

BluetoothSerial SerialBT;
RPLidar lidar;

char command;

float sample [SAMPLE_SIZE];
float estimated_distance;
char est_dist_report [MAX_REPORT_SIZE];

void read_lidar(){

    if (IS_OK(lidar.waitPoint())) {
        float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
        float angle    = lidar.getCurrentPoint().angle; //angle value in degrees
        bool  startBit = lidar.getCurrentPoint().startBit; //whether this point belongs to a new scan
        byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement 
        

        if ((angle >355) || (angle <5)){ 
            for (int i=0 ;i<SAMPLE_SIZE;i++)
                sample[i] = sample[i+1];
            sample[SAMPLE_SIZE-1] = distance;
            
            estimated_distance = 0;
            for (int i=0 ;i<SAMPLE_SIZE;i++)
                estimated_distance += sample[i];
            estimated_distance = estimated_distance / SAMPLE_SIZE;
                         
        } 
    } else {
        digitalWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
        // try to detect RPLIDAR... 
        rplidar_response_device_info_t info;
    
        if (IS_OK(lidar.getDeviceInfo(info, 100))) 
        {
            // detected...
            _s8 fail_type = 0;
            lidar.startScan(fail_type);
        
            // start motor rotating at max allowed speed
            digitalWrite(RPLIDAR_MOTOR, 1);
            //delay(1000);
        }
    }

}

void setup() {
    // Connection with Pololu
    Serial.begin(115200);

    // Bluetooth connection
    SerialBT.begin("BT_ESP32");

    // Connection with Lidar
    lidar.begin(Serial2);
    Serial.print("Check lidar is open: ");
    Serial.println( lidar.isOpen() );
    pinMode(RPLIDAR_MOTOR, OUTPUT); 
    digitalWrite(RPLIDAR_MOTOR, 0);

    // Start Lidar
    Serial.print("Check lidar starts scanning: ");
    _s8 fail_type = 0;
    bool correctStart = IS_OK(lidar.startScan(fail_type, true, lidar.RPLIDAR_DEFAULT_TIMEOUT*5));
    Serial.println( correctStart );
    if(correctStart) {
        digitalWrite(RPLIDAR_MOTOR, 1);
    } else {
        Serial.print("Lidar scanning fail type: ");
        Serial.println( fail_type );
    }
    
    delay(1000);
}

void loop(){ 

    read_lidar();

    if (SerialBT.available()){

        command = SerialBT.read();
        if (command=='0' || command=='1' || command=='2' || command=='3' || command=='4') {
            // Send command to Pololu
            Serial.write(command);

            // Send sensor data to controller
            snprintf(est_dist_report, MAX_REPORT_SIZE, "%f", estimated_distance);
            SerialBT.print(est_dist_report);
        }
        
    }
    
}