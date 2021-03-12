#include <Arduino.h>
#include <vector> 
#include <MPU6050_6Axis_MotionApps20.h>
#include <BluetoothSerial.h>
#include <EEPROM.h>

using namespace std;

//com pins
#define SDA 21
#define SCL 22

//EEPROM constants
#define EEPROM_SIZE 80
#define EEPROM_ADR_INIT 0
#define EEPROM_ADR_ANGLE_kP 10
#define EEPROM_ADR_ANGLE_kI 20
#define EEPROM_ADR_ANGLE_kD 30
#define EEPROM_ADR_VELOCITY_kP 40
#define EEPROM_ADR_VELOCITY_kI 50
#define EEPROM_ADR_VELOCITY_kD 60
#define EEPROM_ADR_OFFSET 70

//motor pins
#define ENA 5
#define M0 15
#define M1 2
#define M2 4
#define DIR1 14
#define STEP1 12
#define DIR2 25
#define STEP2 26
#define POT 2

//derivative calculation
#define DT 5

//motion control
#define MICRO_UPPER_THRESH 30000
#define MICRO_LOWER_THRESH 3000
#define MAX_ACCEL 3000

//finite state machine - operating mode
#define TESTING 0
#define ANGLE_CONTROL 1
#define SPEED_CONTROL 2
uint8_t mode = SPEED_CONTROL;

//task control
TaskHandle_t mainTaskHandle;
TaskHandle_t comTaskHandle;
SemaphoreHandle_t paramSem;
SemaphoreHandle_t angleSem;

//motor control vars and timers
bool standing = false;
int velocity = 0;
int maxFrequency = 80000;
int maxTimerCount0;
int maxTimerCount1;
const int preScaler = 80;
volatile uint8_t pulseState0 = 0;
volatile uint8_t pulseState1 = 0;
hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

//accelerometer vars
const int accelPollRate = 100;
double angleOffset = -1.74;
double angle = 0.0;
float angleRad = 0.0;
unsigned long currentTime;
unsigned long lastPollTime;
bool dataReady;
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//DMP management
MPU6050 mpu;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

//testing:
unsigned long prevPollCount;
unsigned long currPollCount;

//bluetooth vars
unsigned long currentTXTime;
unsigned long lastTXTime;
String rcvMsg = "";
BluetoothSerial SerialBT;

//PID parameters
//should params be volatile???
double angle_kP;
double angle_kI;
double angle_kD;
const uint16_t angleBufferSize = 500;
double angleSum = 0;
vector<double> angleBuffer;

double velocity_kP;
double velocity_kI;
double velocity_kD;
const uint16_t velocityBufferSize = 500;
double velocitySum = 0;
vector<double> velocityBuffer;

//motion control vars
double targetAngle = 0.0;
double maxTargetAngle = 1.0 * PI/180; //1 degree
int targetVelocity = 0;
uint8_t microsteppingFactor = 32;
unsigned int steppedFrequency0 = 0;
unsigned int steppedFrequency1 = 0;
double steeringFactor = 0.0;

//motor 1 pulse
void IRAM_ATTR onTimer0() {
    portENTER_CRITICAL_ISR(&timerMux0);
    pulseState0 = 1 - pulseState0;
    portEXIT_CRITICAL_ISR(&timerMux0);
    if (pulseState0) GPIO.out_w1ts = ((uint32_t)1 << STEP1);
    else GPIO.out_w1tc = ((uint32_t)1 << STEP1);
}

//motor 2 pulse
void IRAM_ATTR onTimer1() {
    portENTER_CRITICAL_ISR(&timerMux1);
    pulseState1 = 1 - pulseState1;
    portEXIT_CRITICAL_ISR(&timerMux1);
    if (pulseState1) GPIO.out_w1ts = ((uint32_t)1 << STEP2);
    else GPIO.out_w1tc = ((uint32_t)1 << STEP2);
}

void setFrequency(uint8_t motor, int frequency) {
    int timerCount;
    if (!frequency) timerCount = 100000000;
    else timerCount = 1000000/frequency;

    if (!motor) timerAlarmWrite(timer0, timerCount, true);
    else timerAlarmWrite(timer1, timerCount, true);
}

void setMicrostepping(uint8_t factor) {
    switch (factor) {
        case 32:
            digitalWrite(M0, HIGH);
            digitalWrite(M1, HIGH);
            digitalWrite(M2, HIGH);
        break;
        case 16:
            digitalWrite(M0, LOW);
            digitalWrite(M1, LOW);
            digitalWrite(M2, HIGH);
        break;
        case 8:
            digitalWrite(M0, HIGH);
            digitalWrite(M1, HIGH);
            digitalWrite(M2, LOW);
        break;
        case 4:
            digitalWrite(M0, LOW);
            digitalWrite(M1, HIGH);
            digitalWrite(M2, LOW);
        break;
        case 2:
            digitalWrite(M0, HIGH);
            digitalWrite(M1, LOW);
            digitalWrite(M2, LOW);
        break;
        case 1:
            digitalWrite(M0, LOW);
            digitalWrite(M1, LOW);
            digitalWrite(M2, LOW);
        break;

    }
}

void printBuffer(vector<double> vec) {
    Serial.print("[");
    for (int i=1; i<vec.size(); i++) {
        Serial.print(vec.at(i));
        Serial.print(", ");
    }
    Serial.println("]");
}

float getAngle() {
    fifoCount = mpu.getFIFOCount();

    // check for overflow
    if (fifoCount == 1024) {
        // reset FIFO to continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return(0.0);
    }
    else {
        //wait for FIFO to fill
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        return(ypr[2] + angleOffset);
    }
        
}

void comTask(void * pvParameters) {

    while (true) {
        //check for bluetooth commands being sent
        //PROBLEM: MOVE THIS TO OTHER CORE SO ROBOT CONTINUES TO WORK WHILE RECEIVING COMMANDS
        while (SerialBT.available()) {
            Serial.println("Bluetooth available");
            rcvMsg = SerialBT.readStringUntil('|');
            //Serial.println(rcvMsg);

            if (mode == ANGLE_CONTROL) {

                switch (rcvMsg[0]) {
                    case 's':
                        switch (rcvMsg[1]) {
                            case '0':
                                mode = TESTING;
                            break;
                            case '2':
                                mode = SPEED_CONTROL;
                            break;

                        }
                    break;
                    case 'p':
                        angle_kP = rcvMsg.substring(1,rcvMsg.length()-1).toDouble();
                        EEPROM.writeDouble(EEPROM_ADR_ANGLE_kP, angle_kP);
                        Serial.println("Updated angle_kP to " + (String)angle_kP);
                        SerialBT.println("pid" + (String)angle_kP + "," + (String)angle_kI + "," + (String)angle_kD);
                    break;
                    case 'i':
                        angle_kI = rcvMsg.substring(1,rcvMsg.length()-1).toDouble();
                        EEPROM.writeDouble(EEPROM_ADR_ANGLE_kI, angle_kI);
                        Serial.println("Updated angle_kI to " + (String)angle_kI);
                        SerialBT.println("pid" + (String)angle_kP + "," + (String)angle_kI + "," + (String)angle_kD);
                    break;
                    case 'd':
                        angle_kD = rcvMsg.substring(1,rcvMsg.length()-1).toDouble();
                        EEPROM.writeDouble(EEPROM_ADR_ANGLE_kD, angle_kD);
                        Serial.println("Updated angle_kD to " + (String)angle_kD);
                        SerialBT.println("pid" + (String)angle_kP + "," + (String)angle_kI + "," + (String)angle_kD);
                    break;
                    case 'a':
                        angleOffset = rcvMsg.substring(1,rcvMsg.length()-1).toDouble() / 1000;
                        EEPROM.writeDouble(EEPROM_ADR_OFFSET, angleOffset);
                        SerialBT.println("Updated angle offset to " + (String)angleOffset);
                    break;
                break;
                }

            }
            
            else if (mode == SPEED_CONTROL) {

                String cmdType = rcvMsg.substring(0, 2);
                
                //angle kP
                if (cmdType == "ap") {
                    xSemaphoreTake(paramSem, portMAX_DELAY);
                    angle_kP = rcvMsg.substring(2,rcvMsg.length()).toDouble();
                    xSemaphoreGive(paramSem);
                    EEPROM.writeDouble(EEPROM_ADR_ANGLE_kP, angle_kP);
                    EEPROM.commit();
                    Serial.println("Updated angle_kP to " + (String)angle_kP);
                    SerialBT.println(" angle pid " + (String)angle_kP + ", " + 
                        (String)angle_kI + ", " + (String)angle_kD);
                }
                //angle kI
                else if (cmdType == "ai") {
                    xSemaphoreTake(paramSem, portMAX_DELAY);
                    angle_kI = rcvMsg.substring(2,rcvMsg.length()).toDouble();
                    xSemaphoreGive(paramSem);
                    EEPROM.writeDouble(EEPROM_ADR_ANGLE_kI, angle_kI);
                    EEPROM.commit();
                    Serial.println("Updated angle_kI to " + (String)angle_kI);
                    SerialBT.println("angle pid " + (String)angle_kP + ", " + 
                        (String)angle_kI + ", " + (String)angle_kD);
                }
                //angle kD
                else if (cmdType == "ad") {
                    xSemaphoreTake(paramSem, portMAX_DELAY);
                    angle_kD = rcvMsg.substring(2,rcvMsg.length()).toDouble();
                    xSemaphoreGive(paramSem);
                    EEPROM.writeDouble(EEPROM_ADR_ANGLE_kD, angle_kD);
                    EEPROM.commit();
                    Serial.println("Updated angle_kD to " + (String)angle_kD);
                    SerialBT.println("angle pid" + (String)angle_kP + ", " + 
                        (String)angle_kI + ", " + (String)angle_kD);
                }
                //velocity kP
                else if (cmdType == "vp") {
                    xSemaphoreTake(paramSem, portMAX_DELAY);
                    velocity_kP = rcvMsg.substring(2,rcvMsg.length()).toDouble();
                    xSemaphoreGive(paramSem);
                    EEPROM.writeDouble(EEPROM_ADR_VELOCITY_kP, velocity_kP);
                    EEPROM.commit();
                    Serial.println("Updated velocity_kP to " + (String)velocity_kP);
                    SerialBT.println("velocity pid " + (String)velocity_kP + ", " + 
                        (String)velocity_kI + ", " + (String)velocity_kD);
                }
                //velocity kI
                else if (cmdType == "vi") {
                    xSemaphoreTake(paramSem, portMAX_DELAY);
                    velocity_kI = rcvMsg.substring(2,rcvMsg.length()).toDouble();
                    xSemaphoreGive(paramSem);
                    EEPROM.writeDouble(EEPROM_ADR_VELOCITY_kI, velocity_kI);
                    EEPROM.commit();
                    Serial.println("Updated velocity_kI to " + (String)velocity_kI);
                    SerialBT.println("velocity pid " + (String)velocity_kP + ", " + 
                        (String)velocity_kI + ", " + (String)velocity_kD);
                }
                //velocity kD
                else if (cmdType == "vd") {
                    xSemaphoreTake(paramSem, portMAX_DELAY);
                    velocity_kD = rcvMsg.substring(2,rcvMsg.length()).toDouble();
                    xSemaphoreGive(paramSem);
                    EEPROM.writeDouble(EEPROM_ADR_VELOCITY_kD, velocity_kD);
                    EEPROM.commit();
                    Serial.println("Updated velocity_kD to " + (String)velocity_kD);
                    SerialBT.println("velocity pid " + (String)velocity_kP + ", " + 
                        (String)velocity_kI + ", " + (String)velocity_kD);
                }
                //angle offset
                else if (cmdType == "ao") {
                    xSemaphoreTake(paramSem, portMAX_DELAY);
                    angleOffset = - (rcvMsg.substring(2,rcvMsg.length()).toDouble()) / 10000.0;
                    xSemaphoreGive(paramSem);
                    EEPROM.writeDouble(EEPROM_ADR_OFFSET, angleOffset);
                    EEPROM.commit();
                    SerialBT.println("Updated angle offset to " + (String)angleOffset);
                }

            }

        }

        //transmit angle over bluetooth
        currentTXTime = millis();
        if (dataReady && currentTXTime - lastTXTime >= 50) {
            lastTXTime = currentTXTime;
            dataReady = false;
            xSemaphoreTake(angleSem, portMAX_DELAY);
            SerialBT.println("a" + (String)angle);
            xSemaphoreGive(angleSem);
            SerialBT.flush();
        }
        delay(20);
    }

}

void mainTask(void * pvParameters) {

    while (true) {
        angleRad = getAngle();

        //handle angle error buffer
        xSemaphoreTake(angleSem, portMAX_DELAY);
        angle = angleRad * 180.0/PI;
        xSemaphoreGive(angleSem);
        double angleError = angleRad - targetAngle;
        angleBuffer.push_back(angleError);
        uint16_t currentAngleBufferSize = angleBuffer.size();
        if (currentAngleBufferSize >= angleBufferSize) angleBuffer.erase(angleBuffer.begin());
        if (currentAngleBufferSize >= 2) angleSum += (angleBuffer[-1] - angleBuffer[0]);

        //handle velocity error buffer;
        double velocityError = (double)(velocity - targetVelocity);
        velocityBuffer.push_back(velocityError / 10000.0); //scale down to avoid large numbers
        uint16_t currentVelocityBufferSize = velocityBuffer.size();
        if (currentVelocityBufferSize >= velocityBufferSize) velocityBuffer.erase(velocityBuffer.begin());
        if (currentVelocityBufferSize >= 2) velocitySum += (velocityBuffer[-1] - velocityBuffer[0]);
        

        bool debugging = false;
        if (debugging) {
            Serial.println("Angle buffer:");
            printBuffer(angleBuffer);
            Serial.println("Velocity buffer:");
            printBuffer(velocityBuffer);
        }

        //enough data to begin correcting
        if (currentAngleBufferSize >= DT+1 && currentVelocityBufferSize >= DT+1) {

            //PID calculation for outer velocity control loop
            xSemaphoreTake(paramSem, portMAX_DELAY);
            double velocity_P = - 0.00000001 * velocity_kP * velocityError;
            double velocity_I = - 0.00000001 * velocity_kI * velocitySum;
            double velocity_D = - 0.00000001 * velocity_kD * (velocityBuffer[currentVelocityBufferSize-1]
                                                - velocityBuffer[currentVelocityBufferSize-DT-1]);
            xSemaphoreGive(paramSem);
            //correction amount for the target angle
            double targetAngleAdjust = velocity_P + velocity_I + velocity_D;
            targetAngle += targetAngleAdjust;

            //limit the target angle to avoid big sways
            if (targetAngle > maxTargetAngle) targetAngle = maxTargetAngle;
            else if (targetAngle < - maxTargetAngle) targetAngle = - maxTargetAngle;

            //PID calculation for inner angle control loop
            xSemaphoreTake(paramSem, portMAX_DELAY);
            double angle_P = - angle_kP * angleError;
            double angle_I = - angle_kI * (0.5 / accelPollRate * angleSum);
            double angle_D = - angle_kD * ((angleBuffer[currentAngleBufferSize-1]
                                            - angleBuffer[currentAngleBufferSize-DT-1]) * accelPollRate);
            xSemaphoreGive(paramSem);
            //correction amount for velocity
            int velocityAdjust = (int) (angle_P + angle_I + angle_D);
            
            //limit acceleration for obvious reasons
            if (velocityAdjust > MAX_ACCEL) velocityAdjust = MAX_ACCEL;
            else if (velocityAdjust < - MAX_ACCEL) velocityAdjust = - MAX_ACCEL;

            //add the limited velocity correction amount to current velocity
            velocity += velocityAdjust;

            
            //debugging print statements
            bool printAnglePID = true;
            if (printAnglePID) {
                Serial.println( "angle: " + (String)angle + " target angle: " + (String)targetAngle + 
                    " P: " + (String)angle_P + " I: " + (String)angle_I + " D: " + 
                    (String)angle_D + " adj: " + (String)velocityAdjust + 
                    " frequency: " + (String)velocity + " stepped0: " + (String)steppedFrequency0);
            }
            
            bool printVelocityPID = false;
            if (printVelocityPID) {
                Serial.println( "velocity: " + (String)velocity + " target velocity: " + (String)targetVelocity + 
                    " P: " + (String)velocity_kP + " I: " + (String)velocity_kI + " D: " + 
                    (String)velocity_kD + " target angle: " + (String)(targetAngle * 180.0/PI) + 
                    " angle: " + (String)angle);
            }
            
            
            

        }

        //incorporate steering factor to produce left and right frequencies
        int steering = (int) (velocity * steeringFactor);
        int frequency0 = velocity + steering;
        int frequency1 = velocity - steering;

        //limit frequencies to maximum value
        if (frequency0 > maxFrequency) frequency0 = maxFrequency;
        if (frequency0 < - maxFrequency) frequency0 = - maxFrequency;
        if (frequency1 > maxFrequency) frequency1 = maxFrequency;
        if (frequency1 < - maxFrequency) frequency1 = - maxFrequency;

        //automatically control microstepping
        //POTENTIAL PROBLEM: velocity lags behind by one iteration
        if (steppedFrequency0 > MICRO_UPPER_THRESH && microsteppingFactor != 1) {
            microsteppingFactor = microsteppingFactor / 2;
            setMicrostepping(microsteppingFactor);
        }
        if (steppedFrequency0 < MICRO_LOWER_THRESH && microsteppingFactor != 32) {
            microsteppingFactor = microsteppingFactor * 2;
            setMicrostepping(microsteppingFactor);
        }
        //modified output frequencies considering microstepping adjustments
        steppedFrequency0 = abs(frequency0 / 32 * microsteppingFactor);
        steppedFrequency1 = abs(frequency1 / 32 * microsteppingFactor);
        
        //change direction pins based on frequency signs
        if (frequency0 < 0) digitalWrite(DIR1, LOW);
        else digitalWrite(DIR1, HIGH);
        if (frequency1 < 0) digitalWrite(DIR2, HIGH);
        else digitalWrite(DIR2, LOW);
        

        //detect if upright, disable everything if not
        if (abs(angle) < 45.0) digitalWrite(ENA, LOW);
        else {
            digitalWrite(ENA, HIGH);
            velocity = 0;
            targetVelocity = 0;
            targetAngle = 0.0;
            steppedFrequency0 = 0;
            steppedFrequency1 = 0;
            frequency0 = 0;
            frequency1 = 0;
            angleBuffer.clear();
            velocityBuffer.clear();
        }

        //set up timers for new frequencies
        setFrequency(0, steppedFrequency0);
        setFrequency(1, steppedFrequency1);

        delay(2);
    }

}

void setup() {

    //set output pins
    pinMode(ENA, OUTPUT);
    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(STEP1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(STEP2, OUTPUT);

    //set initial output pin states
    digitalWrite(ENA, HIGH);
    digitalWrite(M0, HIGH);
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    digitalWrite(DIR1, LOW);
    digitalWrite(STEP1, LOW);
    digitalWrite(DIR2, HIGH);
    digitalWrite(STEP2, LOW);

    //begin serial and bluetooth serial connections
    Serial.begin(115200);
    SerialBT.begin("ESP32");

    //pulse timers
    timer0 = timerBegin(0, preScaler, true);
    timer1 = timerBegin(1, preScaler, true);
    timerAttachInterrupt(timer0, &onTimer0, true);
    timerAttachInterrupt(timer1, &onTimer1, true);
    setFrequency(0, 0);
    setFrequency(1, 0);
    timerAlarmEnable(timer0);
    timerAlarmEnable(timer1);

    //Start up accelerometer and switch off sleep mode
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    devStatus = mpu.dmpInitialize();


    //set gyro offsets
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    //check if DMP initialized properly
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        //Serial.println("DMP initialized");
    }
    else {
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    EEPROM.begin(EEPROM_SIZE);

    //initialize EEPROM if it's the first time
    if (EEPROM.read(EEPROM_ADR_INIT) != 123) {
        EEPROM.write(EEPROM_ADR_INIT, 123);
        for (uint16_t i=1; i<EEPROM_SIZE; i++) {
            EEPROM.write(i, 0);
        }
    }
    EEPROM.commit();
    
    angle_kP = EEPROM.readDouble(EEPROM_ADR_ANGLE_kP);
    angle_kI = EEPROM.readDouble(EEPROM_ADR_ANGLE_kI);
    angle_kD = EEPROM.readDouble(EEPROM_ADR_ANGLE_kD);
    velocity_kP = EEPROM.readDouble(EEPROM_ADR_VELOCITY_kP);
    velocity_kI = EEPROM.readDouble(EEPROM_ADR_VELOCITY_kI);
    velocity_kD = EEPROM.readDouble(EEPROM_ADR_VELOCITY_kD);
    angleOffset = EEPROM.readDouble(EEPROM_ADR_OFFSET);
    Serial.println("Initialized angle_kP=" + (String)angle_kP + ", angle_kI="
    + (String)angle_kI + ", angle_kD=" + (String)angle_kD + ", velocity_kP=" + (String)velocity_kP
    + ", velocity_kI=" + (String)velocity_kI + ", velocity_kD=" + (String)velocity_kD
    + ", angleOffset=" + (String)angleOffset);

    //task management and initialization
    paramSem = xSemaphoreCreateMutex();
    angleSem = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        mainTask,   /* Task function. */
        "Main Task",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        2,           /* priority of the task */
        &mainTaskHandle,      /* Task handle to keep track of created task */
        1);          /* pin task to core 1 */

    xTaskCreatePinnedToCore(
        comTask,   /* Task function. */
        "Communication Task",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        1,           /* priority of the task */
        &comTaskHandle,      /* Task handle to keep track of created task */
        0);          /* pin task to core 0 */

        Serial.println("core tasks launched");
}

void loop() {
    //vTaskDelay(3);
}