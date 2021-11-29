// For servos and stepper
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

unsigned long serial_data_received_time;

#define SERVOMIN 75   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 545  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

const int stepsPerRevolution = 2048; // change this to fit the number of steps per revolution
const int rolePerMinute = 10;        // Adjustable range of 28BYJ-48 stepper is 0~17 rpm

int xlocation;
int ylocation;

int angleOfStepper;

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup()
{
    myStepper.setSpeed(rolePerMinute); // set speed of stepper
    pwm.begin();                       // initialize servo drivers
    pwm.setPWMFreq(SERVO_FREQ);        // Analog servos run at ~50 Hz updates

    // initialize the serial port:
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1);

    armInitialize();
}

void loop()
{

    recvWithStartEndMarkers();
    showNewData();

    // // Controlling Stepper:
    //   myStepper.step(stepsPerRevolution/2);
    //   delay(500);

    // Controlling Servos:
    // pwm.setPWM(0, 0, 250);
    // pwm.setPWM(servo_num, 0, value);

    // Data from Camera (x,y position of face)
    // Serial.print("x,y: ");
    // Serial.print(xlocation);
    // Serial.print(", ");
    // Serial.println(ylocation);
    // delay(200);

    if (xlocation > 4 || xlocation < -4)
    {
        moveStepper(xlocation, face_size);
    }

    // setMotionState();
}

int[] heightChange(int height)
{
}

void armInitialize()
{
    xySet(100, 2000);
    baseSet(0);
}

void baseSet(int angle)
{
}

int angleToPWM(servo, angle)
{
}

void moveStepper(xlocation, face_size)
{
}

void recvWithStartEndMarkers()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial2.available() > 0 && newData == false)
    {
        rc = Serial2.read();

        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                {
                    ndx = numChars - 1;
                }
            }
            else
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
}

void showNewData()
{
    if (newData == true)
    {
        serial_data_received_time = millis();
        int rotation, height, face_size;
        int result = sscanf(receivedChars, "%i,%i,%i", &rotation, &height, &face_size);
        xlocation = map(rotation, 0, 320, -32.5, 32.5);
        ylocation = (195 * (height - 120)) / face_size;

        //      Serial.print("x,y: ");
        //      Serial.print(xlocation);
        //      Serial.print(", ");
        //      Serial.println(ylocation);

        newData = false;
    }
}