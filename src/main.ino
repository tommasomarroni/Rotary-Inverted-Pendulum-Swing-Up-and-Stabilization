#include "KalmanFilter.h"

#define LQR 1
#define SMC 2

// Pins
const int encoderPinA = 2;
const int encoderPinB = 3;
const int buttonPin = 4;
const int motorPinM = 5;
const int motorPinE = 6;
const int potentiometerPin = A0;

// Constants
const double dt = 1; // In milliseconds
const int refCenter = 430;

// Flags
bool loopStarted = false;
int controlType = LQR;

// Timing and Cycles
double startTime = 0, clockStart = 0, clockEnd = 0, timeElapsed = 0;
double cycles = 0;

// Motor Encoder
double encoderCounter = 0;

// Pendulum Parameters
const double mp = 0.158;
const double lp = 0.135;
const double Jp = (1.0/12.0)*mp*(sq(0.15) + sq(0.035));
const double omega_0 = sqrt((mp*9.81*(lp/2.0))/Jp);

// LQR Control
const double c1 = -1.4142, c2 = -3.0459, c3 = -65.7953, c4 = -5.9920;
//const double c1 = -0.0000, c2 = -1.2505, c3 = -43.3071, c4 = -6.2106;
double u_lqr = 0;

// SMC Control
const double epsilon = 2;
double sigma = 0;
double u_smc = 0;

// Swing-Up Control
const double k = 200;
const double E0 = -0.05;
double E = 0;
double u_swing = 0;

// Measurements
double theta = 0;
double theta_d = 0;
double phi = 0;
double phi_d = 0;
double theta_unfold = 0;

double theta_old = 0;
double theta_unfold_old = 0;
double phi_old = 0;

// Kalman Filters
KalmanFilter KF_theta_d = KalmanFilter(2, 2, 0.01);
KalmanFilter KF_phi_d = KalmanFilter(2, 2, 0.01);

// START: Utils -----------------------------------------------------

void start() {
    while(1) {
        if (digitalRead(buttonPin) == HIGH) {
            Serial.print("Start!\n");
            return;
        }
    }
}

void halt() {
    while(digitalRead(buttonPin) == HIGH);

    Serial.print("Halt! [Average cycle time: ");
    Serial.print((millis() - startTime)/cycles);
    Serial.print(" ms]\n");

    analogWrite(motorPinE, 0);
    loopStarted = false;

    while(1) {
        if (digitalRead(buttonPin) == HIGH) {
            Serial.print("Restart!\n");
            startTime = millis();
            encoderCounter = 0;
            cycles = 0;
            return;
        }
    }
}

double normalizePendPos(int pendPos) {
    return ((pendPos - refCenter)*(180.0/420.0))*2.0*PI/360.0;
}

double normalizeBasePos(int encCount) {
    return ((360.0/539.0)*encCount)*2.0*PI/360.0;
}

void motorCommand(double PWMcommand) {
    PWMcommand = round(255*PWMcommand/12);

    if (PWMcommand > 255) PWMcommand = 255;
    if (PWMcommand < -255) PWMcommand = -255;

    if (PWMcommand >= 0) digitalWrite(motorPinM, LOW);
    else digitalWrite(motorPinM, HIGH);

    analogWrite(motorPinE, abs(PWMcommand));
}

double normalizeAngle(double angle)
{
    angle = fmod(angle + PI, 2 * PI);
    return angle >= 0 ? (angle - PI) : (angle + PI);
}

double sat(double value, double min, double max) {
    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}

double sign(double value) {
    if (value > 0) return 1;
    else if (value == 0) return 0;
    else return -1;
}

void pulseEncoderA() {
    if (digitalRead(encoderPinB) == LOW) encoderCounter++;
    else encoderCounter--;
}

void pulseEncoderB() {
    if (digitalRead(encoderPinA) == LOW) encoderCounter--;
    else encoderCounter++;
}

// END: Utils -------------------------------------------------------

void sensing() {
    theta = - normalizeAngle(normalizeBasePos(encoderCounter));
    theta_unfold = - normalizeBasePos(encoderCounter);
    phi = normalizePendPos(analogRead(potentiometerPin));

    theta_d = (theta_unfold - theta_unfold_old)/(dt/1000.0);
    phi_d = (phi - phi_old)/(dt/1000.0);

    theta_d = KF_theta_d.updateEstimate(theta_d);
    phi_d = KF_phi_d.updateEstimate(phi_d);

    theta_old = theta;
    theta_unfold_old = theta_unfold;
    phi_old = phi;
}

void lqr_control() {
    u_lqr =  - (c1*theta + c2*theta_d + c3*phi + c4*phi_d);
    motorCommand(sat(u_lqr, -12, 12));
}

void smc_control() {
    sigma = theta_d + 2*phi_d + 0.5*theta + 30*phi;
    u_smc = 12*(sigma/(abs(sigma) + epsilon));
    motorCommand(u_smc);
}

void swing_up() {
    E = mp*9.81*(lp/2)*(0.5*sq(phi_d/omega_0) + cos(phi) - 1);
    u_swing = sat(k*(E-E0), -12, 12)*sign(phi_d*cos(phi));
    motorCommand(u_swing);
}

void setup() {
    Serial.begin(9600);
    pinMode(buttonPin, INPUT);
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(motorPinM, OUTPUT);
    pinMode(motorPinE, OUTPUT);

    int eraser = 7, prescaler = 4;
    TCCR4B &= ~eraser;
    TCCR4B |= prescaler;

    start();
    startTime = millis();

    attachInterrupt(digitalPinToInterrupt(encoderPinA), pulseEncoderA, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), pulseEncoderB, RISING);
}

void loop() {
    clockStart = millis();

    if (digitalRead(buttonPin) == HIGH && loopStarted) halt();
    if (digitalRead(buttonPin) == LOW) loopStarted = true;

    // START --------------------------------------------------------

    sensing();

    if (abs(phi) < PI/7 && controlType == LQR) lqr_control();
    else if (abs(phi) < PI/7 && controlType == SMC) smc_control();
    else swing_up();

    // END ----------------------------------------------------------

    // Clock stuff: do NOT touch
    clockEnd = millis();
    timeElapsed = clockEnd - clockStart;
    if (dt > timeElapsed) delay(dt - timeElapsed);
    cycles++;
}
