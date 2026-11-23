// ================================================================
//  ПИД-РЕГУЛЯТОР ДВИЖЕНИЯ ПО ЛИНИИ (автономный режим)
//  Платформа : ESP32 (38 пин)
//  Датчики   : 2 × TCRT5000 (GPIO 34 и GPIO 39)
//  Моторы    : драйвер L298N / TB6612 (пины 21, 19, 23, 22)
// ================================================================

// ================================================================
//  РАЗДЕЛ 1 — ПИНЫ МОТОРОВ
// ================================================================
const int pinDriveL_FW = 21;  // Левый мотор: движение ВПЕРЁД (ШИМ)
const int pinDriveL_BW = 19;  // Левый мотор: движение НАЗАД  (ШИМ)
const int pinDriveR_FW = 23;  // Правый мотор: движение ВПЕРЁД (ШИМ)
const int pinDriveR_BW = 22;  // Правый мотор: движение НАЗАД  (ШИМ)

// ================================================================
//  РАЗДЕЛ 2 — КАЛИБРОВОЧНЫЕ ЗНАЧЕНИЯ
// ================================================================
const int PIN_SENSOR_LEFT  = 34;  // Левый датчик TCRT5000
const int PIN_SENSOR_RIGHT = 36;  // Правый датчик TCRT5000

// ВСТАВЬТЕ ВАШИ ЗНАЧЕНИЯ КАЛИБРОВКИ СЮДА
const int CAL_MIN_LEFT  = 9;     // Минимум левого (белая поверхность)
const int CAL_MAX_LEFT  = 4095;  // Максимум левого (чёрная линия)
const int CAL_MIN_RIGHT = 10;    // Минимум правого (белая поверхность)
const int CAL_MAX_RIGHT = 4095;  // Максимум правого (чёрная линия)

// ================================================================
//  РАЗДЕЛ 3 — ПАРАМЕТРЫ ДВИЖЕНИЯ
// ================================================================
const int BASE_SPEED = 140;
const int MAX_CORRECTION = 140;

// ================================================================
//  РАЗДЕЛ 4 — КОЭФФИЦИЕНТЫ ПИД-РЕГУЛЯТОРА
// ================================================================
float Kp = 0.35;   // Пропорциональный
float Ki = 0.001;  // Интегральный
float Kd = 1.2;    // Дифференциальный

// ================================================================
//  РАЗДЕЛ 5 — ВНУТРЕННИЕ ПЕРЕМЕННЫЕ ПИД
// ================================================================
float pid_integral   = 0.0;
float pid_prev_error = 0.0;
const float INTEGRAL_LIMIT = 500.0;

// ================================================================
//  РАЗДЕЛ 7 — НОРМАЛИЗАЦИЯ ДАТЧИКА
// ================================================================
int normalizeLeft(int raw) {
  raw = constrain(raw, CAL_MIN_LEFT, CAL_MAX_LEFT);
  return (raw - CAL_MIN_LEFT) * 1000 / (CAL_MAX_LEFT - CAL_MIN_LEFT);
}

int normalizeRight(int raw) {
  raw = constrain(raw, CAL_MIN_RIGHT, CAL_MAX_RIGHT);
  return (raw - CAL_MIN_RIGHT) * 1000 / (CAL_MAX_RIGHT - CAL_MIN_RIGHT);
}

// ================================================================
//  РАЗДЕЛ 8 — УПРАВЛЕНИЕ МОТОРАМИ
// ================================================================
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(pinDriveL_FW, speed);
    digitalWrite(pinDriveL_BW, LOW);
  } else if (speed < 0) {
    digitalWrite(pinDriveL_FW, LOW);
    analogWrite(pinDriveL_BW, -speed);
  } else {
    digitalWrite(pinDriveL_FW, LOW);
    digitalWrite(pinDriveL_BW, LOW);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(pinDriveR_FW, speed);
    digitalWrite(pinDriveR_BW, LOW);
  } else if (speed < 0) {
    digitalWrite(pinDriveR_FW, LOW);
    analogWrite(pinDriveR_BW, -speed);
  } else {
    digitalWrite(pinDriveR_FW, LOW);
    digitalWrite(pinDriveR_BW, LOW);
  }
}

void setMotorsSpeed(int speedL, int speedR) {
  setLeftMotor(speedL);
  setRightMotor(speedR);
}

void stopMotors() {
  digitalWrite(pinDriveL_FW, LOW); digitalWrite(pinDriveL_BW, LOW);
  digitalWrite(pinDriveR_FW, LOW); digitalWrite(pinDriveR_BW, LOW);
}

// ================================================================
//  РАЗДЕЛ 9 — ПИД-ВЫЧИСЛЕНИЕ
// ================================================================
void pidUpdate(int normL, int normR) {
  float error = (float)(normL - normR);

  pid_integral += error;
  pid_integral = constrain(pid_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = error - pid_prev_error;
  pid_prev_error = error;

  float correction = Kp * error
                   + Ki * pid_integral
                   + Kd * derivative;

  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int speedL = BASE_SPEED - (int)correction;
  int speedR = BASE_SPEED + (int)correction;

  setMotorsSpeed(speedL, speedR);

  Serial.print("L="); Serial.print(normL);
  Serial.print(" R="); Serial.print(normR);
  Serial.print(" err="); Serial.print(error, 1);
  Serial.print(" P="); Serial.print(Kp * error, 1);
  Serial.print(" I="); Serial.print(Ki * pid_integral, 1);
  Serial.print(" D="); Serial.print(Kd * derivative, 1);
  Serial.print(" corr="); Serial.print(correction, 1);
  Serial.print(" L:"); Serial.print(speedL);
  Serial.print(" R:"); Serial.println(speedR);
}

// ================================================================
//  РАЗДЕЛ 10 — ОБНАРУЖЕНИЕ ПЕРЕКРЁСТКА
// ================================================================
const int CROSS_THRESHOLD = 800;
const unsigned long CROSS_DEBOUNCE_MS = 800;

// 0 — прямо, 1 — влево, 2 — вправо, 3 — стоп
const int CROSS_ACTION = 0;
const int CROSS_TURN_MS = 400;

int crossroadCount = 0;
unsigned long lastCrossTime = 0;

bool detectCrossroad(int normL, int normR) {
  bool bothOnLine = (normL > CROSS_THRESHOLD) && (normR > CROSS_THRESHOLD);
  if (!bothOnLine) return false;

  unsigned long now = millis();
  if (now - lastCrossTime < CROSS_DEBOUNCE_MS) {
    return false;
  }

  lastCrossTime = now;
  crossroadCount++;
  return true;
}

void handleCrossroad() {
  Serial.print(">>> ПЕРЕКРЁСТОК #");
  Serial.print(crossroadCount);

  switch (CROSS_ACTION) {
    case 0:
      Serial.println(" -> прямо");
      setMotorsSpeed(BASE_SPEED, BASE_SPEED);
      delay(CROSS_TURN_MS);
      break;
    case 1:
      Serial.println(" -> поворот влево");
      setMotorsSpeed(0, BASE_SPEED);
      delay(CROSS_TURN_MS);
      break;
    case 2:
      Serial.println(" -> поворот вправо");
      setMotorsSpeed(BASE_SPEED, 0);
      delay(CROSS_TURN_MS);
      break;
    case 3:
      Serial.println(" -> СТОП (конец маршрута)");
      stopMotors();
      while (true) {}
      break;
    default:
      Serial.println(" -> неизвестное действие, едем прямо");
      setMotorsSpeed(BASE_SPEED, BASE_SPEED);
      delay(CROSS_TURN_MS);
      break;
  }

  pid_integral   = 0.0;
  pid_prev_error = 0.0;
  Serial.println("    ПИД сброшен.");
}

// ================================================================
//  РАЗДЕЛ 11 — SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=== ПИД Следование по линии ===");
  Serial.print("Kp="); Serial.print(Kp);
  Serial.print("  Ki="); Serial.print(Ki);
  Serial.print("  Kd="); Serial.println(Kd);

  pinMode(pinDriveL_FW, OUTPUT);
  pinMode(pinDriveL_BW, OUTPUT);
  pinMode(pinDriveR_FW, OUTPUT);
  pinMode(pinDriveR_BW, OUTPUT);

  stopMotors();

  Serial.println("--- Калибровочные значения ---");
  Serial.print("Левый:  min="); Serial.print(CAL_MIN_LEFT);
  Serial.print("  max="); Serial.println(CAL_MAX_LEFT);
  Serial.print("Правый: min="); Serial.print(CAL_MIN_RIGHT);
  Serial.print("  max="); Serial.println(CAL_MAX_RIGHT);

  if (CAL_MIN_LEFT == 0 && CAL_MAX_LEFT == 4095) {
    Serial.println("!! ВНИМАНИЕ: калибровочные значения не заданы !!");
  }

  Serial.println("\nСтарт через 3 секунды...");
  delay(1000); Serial.println("3...");
  delay(1000); Serial.println("2...");
  delay(1000); Serial.println("1... ПОЕХАЛИ!");

  pid_integral   = 0.0;
  pid_prev_error = 0.0;
}

// ================================================================
//  РАЗДЕЛ 12 — LOOP
// ================================================================
int debugCounter = 0;

void loop() {
  int rawL = analogRead(PIN_SENSOR_LEFT);
  int rawR = analogRead(PIN_SENSOR_RIGHT);

  int normL = normalizeLeft(rawL);
  int normR = normalizeRight(rawR);

  if (detectCrossroad(normL, normR)) {
    handleCrossroad();
    return;
  }

  debugCounter++;
  if (debugCounter >= 5) {
    debugCounter = 0;
    pidUpdate(normL, normR);
  } else {
    float error      = (float)(normL - normR);
    pid_integral    += error;
    pid_integral     = constrain(pid_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float derivative = error - pid_prev_error;
    pid_prev_error   = error;
    float correction = Kp * error + Ki * pid_integral + Kd * derivative;
    correction       = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
    setMotorsSpeed(BASE_SPEED - (int)correction, BASE_SPEED + (int)correction);
  }

  delay(5);
}
