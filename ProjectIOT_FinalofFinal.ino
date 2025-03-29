const int LEFT_SENSOR_PIN = A0;
const int CENTER_SENSOR_PIN = A1;
const int RIGHT_SENSOR_PIN = A2;

const int MOTOR_RIGHT_PWM1 = 3;
const int MOTOR_RIGHT_PWM2 = 9;
const int MOTOR_LEFT_PWM1 = 10;
const int MOTOR_LEFT_PWM2 = 11;

const int TRIG_PIN = 5;
const int ECHO_PIN = 6;

const int SPEED = 180;
const int MAX_SPEED = 255;
const int MIN_DISTANCE = 15;

int junctionCount = 0;
int obstacleCount = 0;
int lapCount = 0;
bool hasJunction = false;
bool isStoppedPermanently = false;

char bluetoothCommand;
bool autoMode = false;

unsigned long lastBluetoothTime = 0;
const long bluetoothTimeout = 2000;
unsigned long lastDetectionTime = 0;

static unsigned long lostLineTime = 0;
static bool wasFollowingLine = false;
static bool wasGoingStraight = false;
static unsigned long actionStartTime = 0;
static int actionState = 0;
static int lastAction = 0;

unsigned long previousMillis = 0;
const long interval = 20;

float Kp = 70;
float Ki = 0.05;
float Kd = 20;
float error = 0, previousError = 0, integral = 0;

const int SENSOR_THRESHOLD = 500;

void moveBackward(bool maxSpeed = false) {
  int speed = maxSpeed ? MAX_SPEED : SPEED;
  analogWrite(MOTOR_RIGHT_PWM1, speed);
  analogWrite(MOTOR_RIGHT_PWM2, 0);
  analogWrite(MOTOR_LEFT_PWM1, 0);
  analogWrite(MOTOR_LEFT_PWM2, speed);
}

void turnRight(bool maxSpeed = false) {
  int speed = maxSpeed ? MAX_SPEED : SPEED;
  analogWrite(MOTOR_RIGHT_PWM1, 0);
  analogWrite(MOTOR_RIGHT_PWM2, speed);
  analogWrite(MOTOR_LEFT_PWM1, 0);
  analogWrite(MOTOR_LEFT_PWM2, speed);
}

void turnLeft(bool maxSpeed = false) {
  int speed = maxSpeed ? MAX_SPEED : SPEED;
  analogWrite(MOTOR_RIGHT_PWM1, speed);
  analogWrite(MOTOR_RIGHT_PWM2, 0);
  analogWrite(MOTOR_LEFT_PWM1, speed);
  analogWrite(MOTOR_LEFT_PWM2, 0);
}

void moveForward(bool maxSpeed = false) {
  int speed = maxSpeed ? MAX_SPEED : SPEED;
  analogWrite(MOTOR_RIGHT_PWM1, 0);
  analogWrite(MOTOR_RIGHT_PWM2, speed);
  analogWrite(MOTOR_LEFT_PWM1, speed);
  analogWrite(MOTOR_LEFT_PWM2, 0);
}

void stopRobot() {
  analogWrite(MOTOR_RIGHT_PWM1, 0);
  analogWrite(MOTOR_RIGHT_PWM2, 0);
  analogWrite(MOTOR_LEFT_PWM1, 0);
  analogWrite(MOTOR_LEFT_PWM2, 0);
}

void backwardRight(bool maxSpeed = false) {
  int speed = maxSpeed ? MAX_SPEED : SPEED;
  analogWrite(MOTOR_RIGHT_PWM1, speed);
  analogWrite(MOTOR_RIGHT_PWM2, 0);
  analogWrite(MOTOR_LEFT_PWM1, 0);
  analogWrite(MOTOR_LEFT_PWM2, speed / 2);
}

void forwardLeft(bool maxSpeed = false) {
  int speed = maxSpeed ? MAX_SPEED : SPEED;
  analogWrite(MOTOR_RIGHT_PWM1, 0);
  analogWrite(MOTOR_RIGHT_PWM2, speed / 2);
  analogWrite(MOTOR_LEFT_PWM1, speed);
  analogWrite(MOTOR_LEFT_PWM2, 0);
}

void forwardRight(bool maxSpeed = false) {
  int speed = maxSpeed ? MAX_SPEED : SPEED;
  analogWrite(MOTOR_RIGHT_PWM1, 0);
  analogWrite(MOTOR_RIGHT_PWM2, speed);
  analogWrite(MOTOR_LEFT_PWM1, speed / 2);
  analogWrite(MOTOR_LEFT_PWM2, 0);
}

void backwardLeft(bool maxSpeed = false) {
  int speed = maxSpeed ? MAX_SPEED : SPEED;
  analogWrite(MOTOR_RIGHT_PWM1, speed / 2);
  analogWrite(MOTOR_RIGHT_PWM2, 0);
  analogWrite(MOTOR_LEFT_PWM1, 0);
  analogWrite(MOTOR_LEFT_PWM2, speed);
}

void checkLineSensors(float &left, float &center, float &right) {
  left = analogRead(LEFT_SENSOR_PIN);
  center = analogRead(CENTER_SENSOR_PIN);
  right = analogRead(RIGHT_SENSOR_PIN);
}

float checkUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = (duration == 0) ? 1000 : (duration * 0.034 / 2);
  return distance;
}

void reset() {
  junctionCount = 0;
  obstacleCount = 0;
  hasJunction = false;
  isStoppedPermanently = false;
  autoMode = true;
  lastBluetoothTime = 0;
  lastDetectionTime = 0;
  lostLineTime = 0;
  wasFollowingLine = false;
  wasGoingStraight = false;
  actionStartTime = 0;
  actionState = 0;
  lastAction = 0;
  previousMillis = 0;
  error = 0;
  integral = 0;
  previousError = 0;
  lapCount++;
  stopRobot();
}

void avoidObstacle() {
  stopRobot();
  delay(100);
  turnRight();
  delay(200);
  float centerValue;
  do {
    centerValue = analogRead(CENTER_SENSOR_PIN);
    delay(10);
  } while (centerValue < SENSOR_THRESHOLD);

  stopRobot();
  delay(200);
  obstacleCount++;
}

void bluetoothControl() {
  if (Serial.available() > 0) {
    isStoppedPermanently = false;
    junctionCount = 0;
    obstacleCount = 0;
    char inputvalue = Serial.read();
    bluetoothCommand = inputvalue;

    if (inputvalue == 'F') moveForward(true);
    else if (inputvalue == 'B') moveBackward(true);
    else if (inputvalue == 'R') turnRight(true);
    else if (inputvalue == 'L') turnLeft(true);
    else if (inputvalue == 'I') forwardRight(true);
    else if (inputvalue == 'G') forwardLeft(true);
    else if (inputvalue == 'J') backwardRight(true);
    else if (inputvalue == 'H') backwardLeft(true);
    else stopRobot();

    autoMode = false;
    lastBluetoothTime = millis();
  } else {
    if (millis() - lastBluetoothTime >= bluetoothTimeout) {
      autoMode = true;
    }
  }
}

void handleJunction() {
  float leftValue, centerValue, rightValue;
  checkLineSensors(leftValue, centerValue, rightValue);

  if (leftValue > SENSOR_THRESHOLD && centerValue > SENSOR_THRESHOLD && rightValue > SENSOR_THRESHOLD) {
    stopRobot();
    moveForward();
    delay(100);

    if (junctionCount == 1) {
      if (obstacleCount == 1) {
        turnLeft();
        delay(300);
        do {
          centerValue = analogRead(CENTER_SENSOR_PIN);
          delay(10);
        } while (centerValue <= SENSOR_THRESHOLD);
      } else if (obstacleCount == 0) {
        stopRobot();
        delay(100);
        isStoppedPermanently = true;
        return;
      }
    } else if (junctionCount == 2) {
      if (obstacleCount == 2) {
        turnLeft();
        delay(300);
        do {
          centerValue = analogRead(CENTER_SENSOR_PIN);
          delay(10);
        } while (centerValue <= SENSOR_THRESHOLD);
      } else if (obstacleCount == 1) {
        stopRobot();
        delay(100);
        isStoppedPermanently = true;
        return;
      }
    } else if (junctionCount == 3) {
      if (obstacleCount == 2) {
        turnLeft();
        delay(300);
        do {
          centerValue = analogRead(CENTER_SENSOR_PIN);
          delay(10);
        } while (centerValue <= SENSOR_THRESHOLD);
      }
    } else if (junctionCount == 4) {
      if (obstacleCount == 3) {
        turnRight();
        delay(300);
        do {
          centerValue = analogRead(CENTER_SENSOR_PIN);
          delay(10);
        } while (centerValue <= SENSOR_THRESHOLD);
        reset();
        return;
      } else if (obstacleCount == 2) {
        stopRobot();
        delay(100);
        isStoppedPermanently = true;
        return;
      }
    }

    junctionCount++;
    hasJunction = true;
  }
}

void applyPID() {
  float leftValue, centerValue, rightValue;
  checkLineSensors(leftValue, centerValue, rightValue);

  static int lastAction = 0;
  static unsigned long lostLineTime = 0;

  bool sharpTurn = (centerValue < SENSOR_THRESHOLD && (leftValue > SENSOR_THRESHOLD || rightValue > SENSOR_THRESHOLD));

  int dynamicSpeed = sharpTurn ? SPEED / 2 : SPEED;
  float tempKp = sharpTurn ? 100 : 70;
  float tempKd = sharpTurn ? 30 : 20;

  if (leftValue < SENSOR_THRESHOLD && centerValue < SENSOR_THRESHOLD && rightValue < SENSOR_THRESHOLD) {
    if (lostLineTime == 0) lostLineTime = millis();

    if (millis() - lostLineTime < 500) {
      if (lastAction == -1) turnLeft();
      else if (lastAction == 1) turnRight();
      else moveForward();
    } else if (millis() - lostLineTime < 1500) {
      stopRobot();
      delay(200);
      turnLeft();
      delay(300);
      float centerValue;
      do {
        centerValue = analogRead(CENTER_SENSOR_PIN);
      } while (centerValue < SENSOR_THRESHOLD);

    } else {
      stopRobot();
    }
    error = 0;
    integral = 0;
    return;
  } else {
    lostLineTime = 0;
  }

  if (leftValue > SENSOR_THRESHOLD && centerValue > SENSOR_THRESHOLD && rightValue > SENSOR_THRESHOLD) {
    handleJunction();
    return;
  }

  error = (rightValue - leftValue) / (leftValue + 2 * centerValue + rightValue + 0.01);

  integral += error;
  float derivative = error - previousError;
  float output = tempKp * error + Ki * integral + tempKd * derivative;
  previousError = error;

  int leftSpeed = constrain(dynamicSpeed - output, 0, 255);
  int rightSpeed = constrain(dynamicSpeed + output, 0, 255);

  analogWrite(MOTOR_LEFT_PWM1, leftSpeed);
  analogWrite(MOTOR_LEFT_PWM2, 0);
  analogWrite(MOTOR_RIGHT_PWM1, 0);
  analogWrite(MOTOR_RIGHT_PWM2, rightSpeed);

  if (output > 20) lastAction = 1;
  else if (output < -20) lastAction = -1;
  else lastAction = 0;
}

void setup() {
  pinMode(MOTOR_RIGHT_PWM1, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM2, OUTPUT);
  pinMode(MOTOR_LEFT_PWM1, OUTPUT);
  pinMode(MOTOR_LEFT_PWM2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  junctionCount = 0;
  obstacleCount = 0;
  isStoppedPermanently = false;
}

void loop() {
  if (isStoppedPermanently) {
    stopRobot();
  }

  bluetoothControl();

  if (autoMode && !isStoppedPermanently) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      float distance = checkUltrasonic();
      if (distance < MIN_DISTANCE && distance > 0) {
        avoidObstacle();
      } else {
        applyPID();
      }
    }
  }
}