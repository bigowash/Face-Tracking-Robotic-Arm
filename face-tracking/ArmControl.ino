// For servos and stepper
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN 75   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 545  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

const int stepsPerRevolution = 2048; // change this to fit the number of steps per revolution
const int rolePerMinute = 10;        // Adjustable range of 28BYJ-48 stepper is 0~17 rpm

int RGBvalues[3];
// analogWrite(RED, RGBvalues[0]);
// analogWrite(GREEN, RGBvalues[1]);
// analogWrite(BLUE, RGBvalues[2]);

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

// initialize RBG pins
#define BLUE 5
#define GREEN 6
#define RED 7

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// analog pin used to connect the potentiometer
int potpin1 = A0;
int potpin2 = A1;
int potpin3 = A2;
int potpin4 = A3;

// variable to read the value from the analog pin
int val1;
int val2;
int val3;
int stepper;
int ptepper;

// Tolerances
int stepTol = 7;

void setup()
{
    myStepper.setSpeed(rolePerMinute); // set speed of stepper
    pwm.begin();                       // initialize servo drivers
    pwm.setPWMFreq(SERVO_FREQ);        // Analog servos run at ~50 Hz updates

    // initialize the serial port:
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1);

    // initialize RGB
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, LOW);

    // Check stepper
    ptepper = map(analogRead(potpin4), 0, 1023, -180, 180);
}

void loop()
{
    // reads the value of the potentiometer (value between 0 and 1023)
    val1 = analogRead(potpin1);
    val2 = analogRead(potpin2);
    val3 = analogRead(potpin3);
    stepper = analogRead(potpin4);

    // scale it to use it with the servo (value between 0 and 180)
    val1 = map(val1, 1023, 0, 0, 180);
    val2 = map(val2, 0, 1023, 0, 180);
    val3 = map(val3, 1023, 0, 0, 180);
    stepper = map(stepper, 0, 1023, -180, 180);

    runServo();

    if (stepper - ptepper > stepTol || ptepper - stepper > stepTol)
    {
        runStepper();
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

void moveXY(int x, int y))
{
    double beta = acos((42724.89 - x * x - y * y) / 40392) * (180.0 / 3.14159) - 90.0;
    double alpha = (acos((x * x + y * y - 13924.89) / (240 * sqrt(x * x + y * y))) + atan(y / x)) * (180.0 / 3.14159);
    double theta = 90 - beta - alpha;

    Serial.print("beta: ");
    printDouble(beta, 100);
    Serial.print("alpha: ");
    printDouble(alpha, 100);
    Serial.print("theta: ");
    printDouble(theta, 100);

    pwm.setPWM(4, 0, angleToPWM(4, alpha));
    pwm.setPWM(3, 0, angleToPWM(3, beta));
    pwm.setPWM(1, 0, angleToPWM(1, theta));
}
}

void runServo()
{
    pwm.setPWM(0, 0, val1);
    pwm.setPWM(1, 0, val2);
    pwm.setPWM(2, 0, val3);
    // pwm.setPWM(3, 0, val4);
}

int angleToPWM(int servo, int angle)
{
    switch (servo)
    {
    case 1:
        return map(angle, -90, 90, 540, 145);
        break;
    case 3:
        return map(angle, -90, 90, 540, 135);
        break;
    case 4:
        return map(angle, -90, 90, 530, 150);
        break;
    case 2:
        return map(angle, -90, 90, 520, 100);
        break;
    default:
        break;
    }
}

void runStepper()
{
    int steps = map(ptepper - stepper, -360, 360, -2048, 2048);

    myStepper.step(-steps);
    p = val;
}