const int SERVO_PIN[3] = {25, 26, 27};
const int TRIG[3] = {13, 18, 32};
const int ECHO[3] = {14, 19, 33};

// target
float targetTinggi = 25;
float targetPitch = 0;
float targetRoll = 0;

// posisi sensor dari cg
const float X_DEPAN = 120, X_BELAKANG = -210, Y_SAMPING = 80;

// variabel foilnya
const float FOIL_DASAR = 0;
const float FOIL_MIN = -4, FOIL_MAX = 10;  // batas
const int   ARAH_SERVO[3] = {1, -1, 1};

struct PID {
  float kp, ki, kd;
  float integral;
  float errorLama;
};

PID pidTinggi = {0.12, 0.10, 0.01, 0, 0};
PID pidPitch  = {0.60, 0.40, 0.00, 0, 0};
PID pidRoll   = {0.50, 0.50, 0.00, 0, 0};

float hitungPID(PID &pid, float error, float dt) {
  pid.integral += pid.ki * error * dt;
  pid.integral = constrain(pid.integral, -4, 4);
  float turunan = (error - pid.errorLama) / dt;
  pid.errorLama = error;
  float output = pid.kp * error + pid.integral + pid.kd * turunan;
  return constrain(output, -6, 6);
}

float servoSekarang[3] = {90, 90, 90};   //ini di set 90 sebagai default di 3 servonya

void servoMulai(int i) {
  ledcAttach(SERVO_PIN[i], 50, 14);
}

void tulisFoil(int i, float sudutFoil) {
  sudutFoil = constrain(sudutFoil, FOIL_MIN, FOIL_MAX);
  float sudutServo = 90 + ARAH_SERVO[i] * 3 * sudutFoil;  
  servoSekarang[i] = sudutServo;
  float pulsaUs = 544 + sudutServo * (2400 - 544) / 180;
  int duty = pulsaUs * 16383 / 20000; // 14 bit dengan periode 20 ms
  ledcWrite(SERVO_PIN[i], duty);
}

// sensor
float bacaJarak(int i) {
  digitalWrite(TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG[i], LOW);
  long durasi = pulseIn(ECHO[i], HIGH, 10000);  
  if (durasi == 0) return -1; 
  return durasi / 58.0;
}

float jarak[3] = {25, 25, 25};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    servoMulai(i);
    tulisFoil(i, FOIL_DASAR);
  }
}

void loop() {
  const float dt = 0.04;
  static unsigned long waktuLama = 0;
  if (millis() - waktuLama < 40) return;
  waktuLama = millis();

  // read ketiga sensor
  for (int i = 0; i < 3; i++) {
    float d = bacaJarak(i);
    if (d > 2 && d < 150) jarak[i] = d;
  }

  // hasil jarak -> tinggi pitch roll
  float tanRoll  = (jarak[0] - jarak[1]) / (2 * Y_SAMPING);
  float tanPitch = ((jarak[0] + jarak[1]) / 2 - jarak[2]) / (X_DEPAN - X_BELAKANG);
  float tinggi = jarak[2] - X_BELAKANG * tanPitch;
  float pitch  = atan(tanPitch) * RAD_TO_DEG;
  float roll   = atan(tanRoll) * RAD_TO_DEG;

  // kalkulasi pid
  float uTinggi = hitungPID(pidTinggi, targetTinggi - tinggi, dt);
  float uPitch  = hitungPID(pidPitch,  targetPitch - pitch,   dt);
  float uRoll   = hitungPID(pidRoll,   targetRoll - roll,     dt);

  float foil[3];
  foil[0] = FOIL_DASAR + uTinggi + uPitch + uRoll;
  foil[1] = FOIL_DASAR + uTinggi + uPitch - uRoll;
  foil[2] = FOIL_DASAR + uTinggi - 1.75 * uPitch;
  for (int i = 0; i < 3; i++) {
    foil[i] = constrain(foil[i], FOIL_MIN, FOIL_MAX);
    tulisFoil(i, foil[i]);
  }

  // di tampil serial monitor
  static unsigned long waktuPrint = 0;
  if (millis() - waktuPrint >= 500) {
    waktuPrint = millis();
    Serial.println("------------------------------------------------------------");
    Serial.printf("Sensor (cm) : sL = %5.1f   sR = %5.1f   sB = %5.1f\n",
                  jarak[0], jarak[1], jarak[2]);
    Serial.printf("Kapal       : tinggi = %5.1f cm   pitch = %5.2f deg   roll = %5.2f deg\n",
                  tinggi, pitch, roll);
    Serial.printf("Sudut foil  : FL = %5.2f   FR = %5.2f   FB = %5.2f  deg\n",
                  foil[0], foil[1], foil[2]);
    Serial.printf("Putar servo : FL = %5.1f   FR = %5.1f   FB = %5.1f  deg\n",
                  servoSekarang[0], servoSekarang[1], servoSekarang[2]);
  }
}
