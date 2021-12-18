// For servos and stepper
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "IRremote.h"
#include <Servo.h>

Servo myservo;
int receiver = 2;

IRrecv irrecv(receiver); // create instance of 'irrecv'
decode_results results;  // create instance of 'decode_results'

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

unsigned long serial_data_received_time;

int redPin = 7;
int greenPin = 12;
int bluePin = 13;

#define SERVOMIN 75   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 545  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

const int stepsPerRevolution = 2048; // change this to fit the number of steps per revolution
const int stepperSpeed = 15;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm

int xlocation;
int ylocation;

int angleOfStepper = 512;

int curX;
int curY;

int rotation, height, face_size;
int before;

int amountX = 5;
int amountY = 5;
int amountZ = 10;

bool open = false;

int pwms[5];
// int pwms[5] = {300, 300, 300, 300, 300};

String x;
String y;

// for the repeat key on IRremote
unsigned long key_value = 0;

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

bool stepperAngleCheck(int steps)
{
    int end = angleOfStepper + steps / 2;
    if (end > -500 && end < 1000)
    {
        Serial.println(angleOfStepper);
        angleOfStepper = end;
        return true;
    }
    return false;
}

void setColor(int red, int green, int blue)
{
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
}

void color(String colord)
{
    Serial.println(colord);
    if (colord == "red")
    {
        setColor(0, 0, 255);
    }
    else if (colord == "blue")
    {
        setColor(255, 0, 0); // red
    }
    else if (colord == "purple")
    {
        setColor(150, 0, 150);
    }
    else if (colord == "off")
    {
        setColor(0, 0, 0);
    }
    // delay(1000);
}

void translateIR()
{
    if (results.value == 0XFFFFFFFF)
        results.value = key_value;
    switch (results.value)
    {
    case 0xFFA25D:
        Serial.println("POWER");
        armInitialize();
        delay(200);
        break;
    case 0xFFE21D:
        Serial.println("FUNC/STOP");
        break;
    case 0xFF629D:
        Serial.println("VOL+");
        moveY(amountY);
        break;
    case 0xFF22DD:
        Serial.println("FAST BACK");
        moveX(-amountX);
        break;
    case 0xFF02FD:
        Serial.println("PAUSE");
        if (open)
        {
            pwms[0] = 305;
        }
        else
        {
            pwms[0] = 450;
        }
        open = !open;
        delay(200);
        break;
    case 0xFFC23D:
        Serial.println("FAST FORWARD");
        moveX(amountX);
        break;
    case 0xFFE01F:
        Serial.println("DOWN");
        if (stepperAngleCheck(-amountZ))
        {
            myStepper.step(-amountZ);
        }
        break;
    case 0xFFA857:
        Serial.println("VOL-");
        moveY(-amountY);
        break;
    case 0xFF906F:
        Serial.println("UP");
        if (stepperAngleCheck(amountZ))
        {
            myStepper.step(amountZ);
        }
        // moveY(amount);
        break;
    case 0xFF9867:
        Serial.println("EQ");
        if (amountX > 2)
        {
            amountY -= 1;
            amountX -= 1;
        }

        Serial.println(amountX);
        break;
    case 0xFFB04F:
        Serial.println("ST/REPT");
        if (amountX < 25)
        {
            amountY += 1;
            amountX += 1;
        }

        Serial.println(amountX);
        break;
    case 0xFF6897:
        Serial.println("0");
        pwms[1] += 5;
        break;
    case 0xFF30CF:
        Serial.println("1");
        pwms[1] -= 5;
        break;
    case 0xFF18E7:
        Serial.println("2");
        break;
    case 0xFF7A85:
        Serial.println("3");
        break;
    case 0xFF10EF:
        Serial.println("4");
        pwms[4] += 5;
        break;
    case 0xFF38C7:
        Serial.println("5");
        pwms[3] += 5;
        break;
    case 0xFF5AA5:
        Serial.println("6");
        pwms[2] += 5;
        break;
    case 0xFF42BD:
        Serial.println("7");
        pwms[4] -= 5;
        break;
    case 0xFF4AB5:
        Serial.println("8");
        pwms[3] -= 5;
        break;
    case 0xFF52AD:
        Serial.println("9");
        pwms[2] -= 5;
        break;
        // case 0xFFFFFFFF: Serial.println(" REPEAT");break;

    default:
        Serial.println(" other button   ");
        results.value = 0;

    } // End Case
    if (results.value != 0)
    {
        key_value = results.value;
    }
    // delay(200); // Do not get immediate repeat
}

void armInitialize()
{
    moveXY(40, 150);
    pwms[2] = 312;
    int numSteps = 512 - angleOfStepper;
    myStepper.step(numSteps);
    angleOfStepper = 512;
}

bool inBounds(int x, int y)
{
    if (y > 240 || y < 0)
    {
        return false;
    }
    if (x > 120 && y > 80)
    {
        return false;
    }
    if (x > 85 && y > 130)
    {
        return false;
    }
    return true;
}

void tilt()
{
    float ymove = ((-120 + ylocation) / 2);
    pwms[1] = ((pwms[1] + ymove) + pwms[1]) / 2;
    Serial.println("ylocation");
    Serial.println(ylocation);
    Serial.println("pwms[1]");
    Serial.println(pwms[1]);
    Serial.println("ymove");
    Serial.println(ymove);
}

void rotate()
{
    Serial.print("xlocation: ");
    Serial.println(xlocation);
    int numSteps = map(xlocation, -32.5, 32.5, -185, 185);
    Serial.print("numSteps: ");
    Serial.println(numSteps);
    if (stepperAngleCheck(numSteps))
    {
        myStepper.step(numSteps * 2);
    }
    // Serial.println(numSteps);
}

// void smove(int s1, int s2, int s3){
//   int pos1 = s1 / 12;
//   int pos2 = s2 / 12;
//   int pos3 = s3 / 12;
//   int pos3b = pos3;
//   int pos2b = pos2;
//   int pos1b = pos1;
//   myservo2.write(pos1b);
//   myservo3.write(pos2b);
//   myservo4.write(pos3b);
//   delay(75); //1
// }

int setAngle(int servo, int angle)
{
    int output;
    int outcome = 0;

    switch (servo)
    {
    case 0:
        output = map(angle, 0, 90, 220, 450);
        pwms[0] = output;
        break;
    case 1:
        Serial.println(angle);
        // output = map(angle, -90, 90, 180, 0);
        output = map(angle, 0, 90, 390, 170);
        if (output > 520)
        {
            color("purple");

            Serial.println("Servo 1 (theta) out of bounds");
            pwms[1] = 520;
        }
        else
        {
            pwms[1] = output;
        }
        break;
    case 2:
        output = map(angle, -90, 90, 105, 515);
        pwms[2] = output;
        break;
    case 3:
        output = map(angle, -90, 0, 460, 255);
        if (output > 146)
        {
            pwms[3] = output;
        }
        else
        {
            color("purple");

            Serial.println("Servo3 (beta) Will break something, setting to 145");
            pwms[3] = 145;
            outcome = 3145;
        }
        break;
    case 4:
        Serial.print("alpha: ");
        Serial.println(angle);
        if (angle >= -88 && angle <= 90)
        {
            int output = map(angle, -88, 90, 540, 137);
            pwms[4] = output;
        }
        else if (angle < -88)
        {
            color("purple");

            Serial.println("Servp4 (alpha) Will break something, setting to 550");
            pwms[4] = 550;
            outcome = 4550;
        }
        else if (angle > 90)
        {
            color("purple");
            Serial.println("Servo4 (alpha) Will break something, setting to 137");
            pwms[4] = 137;
            outcome = 4137;
        }
        // Serial.print("outcome: ");
        // Serial.println(outcome);
        break;
    default:
        break;
    }
    return outcome;
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

void printDouble(double val, unsigned int precision)
{
    // prints val with number of decimal places determine by precision
    // NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
    // example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

    Serial.print(int(val)); //prints the int part
    Serial.print(".");      // print the decimal point
    unsigned int frac;
    if (val >= 0)
        frac = (val - int(val)) * precision;
    else
        frac = (int(val) - val) * precision;
    Serial.println(frac, DEC);
}

// void wrongAngle (int outa, int outb, int outt) {
//   int fails[] = [3145, 4550, 4137];
//   // for (int i; i < sizeof(fails); i++){
//   //   int val = fails[i];
//   //   if (outa = )
//   // }
//   if (outa == 4550){
//     // value of servo 4 is 550
//   } else if (outa == 4137){
//     //val of 4 is 137
//   } if (outb == 3145){
//     //val of 3 is 145
//   }
//   }
// }

void moveY(int y)
{
    Serial.print("(");
    Serial.print(curX);
    Serial.print(",");
    Serial.print(curY);
    Serial.println(")");
    moveXY(curX, curY + y);
}

void moveX(int x)
{
    Serial.print("(");
    Serial.print(curX);
    Serial.print(",");
    Serial.print(curY);
    Serial.println(")");
    moveXY(curX + x, curY);
}

void moveXY(double x, double y)
{

    // Serial.println("----- Inside moveXY -----");
    // Serial.print("(");
    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.println(")");

    if (inBounds(x, y))
    {
        if (sqrt(x * x + y * y) < 286.3 || y > 0)
        {

            double beta = acos((42724.89 - x * x - y * y) / 40392) * (180.0 / 3.14159) - 90.0;
            double alpha = (acos((x * x + y * y - 13924.89) / (240 * sqrt(x * x + y * y))) + atan(y / x)) * (180.0 / 3.14159);
            double theta = 90 - beta - alpha;
            Serial.print("alpha: ");
            // printDouble(alpha, 100);
            // Serial.print("beta: ");
            // printDouble(beta, 100);
            // Serial.print("theta: ");
            // printDouble(theta, 100);

            if (alpha >= 0 && alpha <= 180)
            {
                alpha = -alpha + 90;
                theta = -theta;
                beta = -beta;

                Serial.print("alpha: ");
                printDouble(alpha, 100);
                Serial.print("beta: ");
                printDouble(beta, 100);
                Serial.print("theta: ");
                printDouble(theta, 100);

                int outa = setAngle(4, alpha);
                // Serial.println(outa);
                if (outa == 0)
                {
                    int outb = setAngle(3, beta);
                    // Serial.println(outb);
                    if (outb == 0)
                    {
                        int outt = setAngle(1, theta);
                        // Serial.println(outt);
                        curX = x;
                        curY = y;
                        Serial.print("(");
                        Serial.print(curX);
                        Serial.print(",");
                        Serial.print(curY);
                        Serial.println(")");
                    }
                }
            }
            else
            {
                Serial.println("out of 3");
            }
        }
        else
        {
            color("red");
            Serial.println("out of 2");
        }
    }
    else
    {
        color("red");
        Serial.println("out of 1");
    }
}

void showNewData()
{
    if (newData == true)
    {
        serial_data_received_time = millis();

        int result = sscanf(receivedChars, "%i,%i,%i", &rotation, &height, &face_size);

        xlocation = map(rotation, -320, 320, -32.5, 32.5);
        // ylocation = (195 * (height - 120)) / face_size;
        ylocation = height;

        newData = false;
    }
}

void setup()
{

    // initialize the serial port:
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1);

    myStepper.setSpeed(stepperSpeed); // set speed of stepper
    pwm.begin();                      // initialize servo drivers
    pwm.setPWMFreq(SERVO_FREQ);       // Analog servos run at ~50 Hz updates

    Serial.println();
    Serial.println("------ Start of Program ------");

    irrecv.enableIRIn(); // Start the receiver

    // myservo.attach(9);

    pinMode(redPin, OUTPUT);
    // pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    armInitialize();
}

void loop()
{

    recvWithStartEndMarkers();
    showNewData();

    if (rotation + height + face_size == before)
    {
        xlocation = 0;
        ylocation = 0;
        // Serial.println("Same face");
        // color("off");
    }
    else
    {
        Serial.println("Found face!");
        color("blue");
        tilt();
        rotate();
    }

    // // Read value from joystick
    // horizontal = analogRead(A4);  // orange
    // vertical = analogRead(A5);    // yellow

    // Serial.print("horizontal values: ");
    // Serial.println(horizontal);
    // Serial.print("vertical values: ");
    // Serial.println(vertical);

    // // move the stepper
    // if (horizontal > 500) {
    //   myStepper.step(10);
    // }

    // if (horizontal < 300) {
    //   myStepper.step(-10);
    // }
    // Serial.print("horizontal values: ");

    // have we received an IR signal?
    if (irrecv.decode(&results))
    {
        color("purple");
        translateIR();
        irrecv.resume(); // receive the next value
    }

    // Serial.println("Enter X: ");
    // while (Serial.available() == 0)   {}

    // x = Serial.readString();
    // Serial.println("Y: ");
    // while (Serial.available() == 0)   {}

    // y = Serial.readString();
    // Serial.print("XY: ");
    // Serial.print(x);
    // Serial.print(" ,");
    // Serial.println(y);

    //    moveXY(x.toInt(), y.toInt());

    // Serial.println("Setting PWMs");
    for (int i = 4; i >= 0; i--)
    {
        // Serial.println(i);
        pwm.setPWM(i, 0, pwms[i]);
    }

    // if the data is exactly the same, do nothing
    before = rotation + height + face_size;
}
