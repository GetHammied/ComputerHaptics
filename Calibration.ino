/*
Note:
  - Hapkits can't sustain permanent use. The motor will overheat.
  - White boards' timers delay and millis is 64x faster to achieve fast PWM.
*/

enum Board {WHITE, GREEN};
enum Motor {TOP, BOTTOM};
enum Pin {PWM, DIR};
int pinMapping[2][2][2] = {
  5, 8, 6, 7,
  3, 12, 11, 13
};

// Calibration 
int minValue = 49;
int maxValue = 976;
int sensorRange = 928;

// Pins 
int *pins = pinMapping[WHITE][TOP];
int pwmPin = pins[PWM];
int dirPin = pins[DIR];
int sensorPin = A2;

// Position tracking
long lastSensorValue;
#define SENSOR_DISTANCE_COUNT 32
long sensorDistances[SENSOR_DISTANCE_COUNT];
int sensorDistanceIndex = 0;
float ticksPerDegree = 71.0;
long positionInTicks = 0;
float positionInDegree = 0.0f;
float positionInMeter = 0.0f;
float radius = 0.075f; //measured 75 mm
// Kinematics
unsigned long velocityLastMillis;
float velocity = 0.0f;
#define VELOCITY_1POLE_CUTOFF 0.05f

void setup() 
{
    // Set in/out-put pins
    pinMode(pwmPin, OUTPUT);
    setPwmFrequency(pwmPin, 1);
    pinMode(dirPin, OUTPUT);
    pinMode(sensorPin, INPUT);

    // Start serial communication
    Serial.begin(500000); // or 57600 

    // Initialize position variables
    velocityLastMillis = millis();
    lastSensorValue = analogRead(sensorPin);

    // Initialize motor
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
}

// ======================================== //

int getSensorDistance (int x1, int x0)
{
    int d = x1 - x0;
    if (abs (d) <= sensorRange/2) return d;
    if (x1 < x0) return d + sensorRange;
    return d - sensorRange;
}

void updatePosition()
{
    // Read raw position from MR sensor
    long sensorValue = analogRead (sensorPin);
    // Log & calculate difference between subsequent raw MR sensor readings
    sensorDistances[sensorDistanceIndex++] = getSensorDistance (sensorValue, lastSensorValue);
    
    if (sensorDistanceIndex == SENSOR_DISTANCE_COUNT)
    {
        float lastPositionInMeter = positionInMeter;
        long sensorDistanceAccumulator = 0;
        for (int i = 0; i < SENSOR_DISTANCE_COUNT; ++i)
            sensorDistanceAccumulator += sensorDistances[i];

        double pi = 3.141592653589f;
        
        // Calculate position in meters
        positionInTicks = sensorDistanceAccumulator;  // This assumes 1 tick per unit
        positionInDegree = (float)positionInTicks / ticksPerDegree;
        positionInMeter = (float)positionInDegree * (2.0 * pi * 0.075) / 360.0;

        // Update values
        lastSensorValue = sensorValue;
        sensorDistanceIndex = 0;

        // Calculate velocity
        unsigned long dt = millis() - velocityLastMillis;
        velocity = (float)(positionInTicks - lastPositionInMeter) / (float)dt;
        velocityLastMillis = millis();
    }
}

// ======================================== //

// Main loop
unsigned long printCount = 0;
void loop()
{
    // interrupt
    if (Serial.available())
    {
        analogWrite (pwmPin, 0);
        while (true)
        {
            Serial.println (23);
            delay (2000);
        }
    }
    updatePosition();

    // Set force in Newtons
    double force = 0.0;
    if (millis() > printCount * 100)
    {
        Serial.print("Position (meters): ");
        Serial.println(positionInMeter, 4); // Assuming 4 decimal places for precision
        Serial.print("Angle (degrees): ");
        Serial.println(positionInDegree, 2); // Assuming 2 decimal places for precision
        ++printCount;
    }
    // Compute & apply force
    motorControl (force);
}

// ======================================== //

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      // changing TCCR0B effects millis() and delay()
      // https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm
      // https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
  Serial.println(mode, HEX);
}

void motorControl (double force)
{
  double rp = 0.004191;
  double rs = 0.073152;
  double rh = 0.065659;

  //TODO: Compute the require motor pulley torque (Tp) to generate given force
  double Tp = force * radius;;
  // Determine correct direction for motor torque
  // You may need to reverse the digitalWrite functions according to your motor connections
  if (force < 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // ToDo: Compute the duty cycle (double duty) required to generate Tp (torque at the motor pulley)
  double duty = Tp / (rp * rs *rh);
  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  unsigned int output = (unsigned int)(duty * 255);  // convert duty cycle to output signal
  analogWrite(pwmPin, output); // output the signal
}
