#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Define ESP32 Cores
#define CORE_0 0
#define CORE_1 1

// Pin LoRa
#define ss 5
#define rst 14
#define dio0 2
#define BAND 433E6

// Pin Buzzer
#define BUZZER_PIN 25

// Sensor DHT11
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Pin Driver Motor L298N
#define IN1 12  // Input 1 Motor A
#define IN2 22  // Input 2 Motor A 
#define IN3 16  // Input 1 Motor B
#define IN4 15  // Input 2 Motor B

// Pin Sensor Ultrasonik HC-SR04
#define TRIG_PIN 32
#define ECHO_PIN_FRONT 33
#define ECHO_PIN_BACK 27
#define ECHO_PIN_RIGHT 35
#define ECHO_PIN_LEFT 13

// Definisi Sensor MQ-135
#define board ("ESP-32")
#define Voltage_Resolution 3.3
#define pin 34
#define type "MQ-135"
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6
#define WARMUP_TIME 20000 // 20 detik waktu pemanasan

// Definisi pin GPS untuk ESP32
static const int RXPin = 17;
static const int TXPin = 21;
static const uint32_t GPSBaud = 9600;

// Inisialisasi objek GPS
TinyGPSPlus gps;
HardwareSerial GPS(2);

// Deklarasi Sensor MQ-135
MQUnifiedsensor MQ135(board, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

// Mutex untuk LoRa
SemaphoreHandle_t loraMutex = NULL;

// Fungsi untuk membunyikan buzzer
void beepBuzzer(int duration, int frequency = 1000) {
  tone(BUZZER_PIN, frequency);
  delay(duration);
  noTone(BUZZER_PIN);
}

// Fungsi untuk LoRa initialization success sound
void loraInitSound() {
  for (int i = 0; i < 4; i++) {
    beepBuzzer(100, 900); 
    delay(40);            
    beepBuzzer(100, 700); 
    delay(40);
  }

  beepBuzzer(300, 1200);
  delay(100);
  beepBuzzer(150, 1000);
}

// Fungsi untuk MQ135 calibration completion sound
void mq135CalibrationSound() {
  beepBuzzer(150, 1000);
  delay(50);
  beepBuzzer(150, 1500);
  delay(50);
  beepBuzzer(150, 2000);
  delay(50);
  beepBuzzer(300, 2500);
}

void mq135ErrorSound() {
  for(int i = 0; i < 3; i++) {
    beepBuzzer(200, 2000);
    delay(100);
    beepBuzzer(200, 1000);
    delay(100);
  }
  beepBuzzer(500, 500); // Long low tone at the end
}

// Fungsi untuk inisialisasi LoRa
bool startLoRA() {
  LoRa.setPins(ss, rst, dio0);
  for (int i = 0; i < 5; i++) {
    if (LoRa.begin(BAND)) {
      Serial.println("LoRa Initialization OK!");
      loraInitSound();
      return true;
    }
    Serial.print(".");
    delay(500);
  }
  Serial.println("LoRa Initialization FAILED!");
  return false;
}

// Inisialisasi motor driver
void setupMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

// Inisialisasi sensor ultrasonik
void setupUltrasonicSensors() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(ECHO_PIN_BACK, INPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
}

// Fungsi untuk membaca jarak dari sensor ultrasonik
float getDistance(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(echoPin, HIGH);
  
  if (duration == 0) {
    return NAN;
  }

  float distance = duration * 0.034 / 2;
  
  if (distance > 400.0) {
    return NAN;
  }

  return distance;
}

// Fungsi untuk mengkoreksi nilai berdasarkan suhu dan kelembaban
float correctedPPM(float ppm, float temperature, float humidity) {
  if (temperature == 0 || humidity == 0) return ppm;
  
  return ppm * (1.0 + 0.02 * (temperature - 20.0)) * (1.0 + 0.02 * (humidity - 65.0));
}

// Fungsi kalibrasi MQ-135
void calibrateMQ135(MQUnifiedsensor &mq135, DHT &dht) {
  Serial.println("Memulai kalibrasi MQ-135...");
  Serial.println("Pastikan sensor berada di udara bersih!");
  
  // Pemanasan sensor
  Serial.println("Pemanasan sensor selama 20 detik...");
  unsigned long startTime = millis();
  while (millis() - startTime < WARMUP_TIME) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nPemanasan selesai!");

  // Baca suhu dan kelembaban
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  Serial.println("Suhu: " + String(temperature) + "°C");
  Serial.println("Kelembaban: " + String(humidity) + "%");

  // Kalibrasi dengan pengambilan sampel
  Serial.println("Mengambil sampel untuk kalibrasi...");
  float calcR0 = 0;
  int validSamples = 0;
  
  for (int i = 1; i <= 10; i++) {
    mq135.update();
    float R0 = mq135.calibrate(RatioMQ135CleanAir);
    
    if (!isinf(R0) && R0 != 0) {
      calcR0 += R0;
      validSamples++;
      Serial.println("Sampel " + String(i) + ": R0 = " + String(R0));
    } else {
      Serial.println("Sampel " + String(i) + ": Invalid!");
    }
    
    delay(1000);
  }

  if (validSamples > 0) {
    float finalR0 = calcR0 / validSamples;
    mq135.setR0(finalR0);
    Serial.println("Kalibrasi selesai!");
    Serial.println("R0 final: " + String(finalR0));
    mq135CalibrationSound();  // Added completion sound
  } else {
    Serial.println("Kalibrasi gagal! Periksa koneksi sensor.");
    mq135ErrorSound();
  }
}

// Fungsi untuk membaca gas dengan kompensasi suhu dan kelembaban
void readGasSensor(float &carbonDioxide, float &ammonia, String &airQuality) {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Deteksi CO2
  MQ135.setA(110.47); MQ135.setB(-2.862);
  MQ135.update();
  float rawCO2 = MQ135.readSensor();
  carbonDioxide = correctedPPM(rawCO2, temperature, humidity);

  // Deteksi NH3 (Amonia)
  MQ135.setA(116.602); MQ135.setB(-2.769);
  MQ135.update();
  float rawNH3 = MQ135.readSensor();
  ammonia = correctedPPM(rawNH3, temperature, humidity);
}

// Fungsi GPS
void processGPS() {
  while (GPS.available() > 0) {
    gps.encode(GPS.read());
  }
}

// Fungsi untuk menjalankan perintah motor
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Fungsi untuk mengeksekusi perintah
void executeCommand(String command) {
  command.toUpperCase();
  if (command == "W") {
    Serial.println("Rover bergerak maju");
    moveForward();
  } else if (command == "S") {
    Serial.println("Rover bergerak mundur");
    moveBackward();
  } else if (command == "D") {
    Serial.println("Rover belok kanan");
    turnRight();
  } else if (command == "A") {
    Serial.println("Rover belok kiri");
    turnLeft(); 
  } else if (command == "X") {
    Serial.println("Rover berhenti");
    stopMotors();
  } else {
    Serial.println("Perintah tidak dikenali");
  }
}

// Task untuk menangani penerimaan perintah
void commandTask(void * parameter) {
  while(1) {
    if (xSemaphoreTake(loraMutex, portMAX_DELAY)) {
      int packetSize = LoRa.parsePacket();
      if (packetSize) {
        String command = "";
        while (LoRa.available()) {
          command += (char)LoRa.read();
        }
        Serial.print("Message Received: ");
        Serial.println(command);
        executeCommand(command);
      }
      xSemaphoreGive(loraMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task untuk menangani pengiriman data sensor
void sensorTask(void * parameter) {
  while(1) {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    float ultrasonic_front = getDistance(ECHO_PIN_FRONT);
    float ultrasonic_back = getDistance(ECHO_PIN_BACK);
    float ultrasonic_right = getDistance(ECHO_PIN_RIGHT);
    float ultrasonic_left = getDistance(ECHO_PIN_LEFT);

    float carbonDioxide, ammonia;
    String airQuality;
    readGasSensor(carbonDioxide, ammonia, airQuality);

    String sensorData = "temperature:" + String(temperature, 1) + "C "
                       "humidity:" + String(humidity, 1) + "% "
                       "carbonDioxide:" + String(carbonDioxide, 1) + " "
                       "ammonia:" + String(ammonia, 1) + " "
                       "ultrasonic_front:" + String(ultrasonic_front, 1) + "cm "
                       "ultrasonic_back:" + String(ultrasonic_back, 1) + "cm "
                       "ultrasonic_right:" + String(ultrasonic_right, 1) + "cm "
                       "ultrasonic_left:" + String(ultrasonic_left, 1) + "cm ";
    
    if (gps.location.isValid()) {
      sensorData += "latitude:" + String(gps.location.lat(), 6) + " "
                    "longitude:" + String(gps.location.lng(), 6) + " "
                    "altitude:" + String(gps.altitude.meters(), 1) + "m "
                    "speed:" + String(gps.speed.kmph(), 1) + "km/h";
    } else {
      sensorData += "GPS:NoFix";
    }

    if (xSemaphoreTake(loraMutex, portMAX_DELAY)) {
      LoRa.beginPacket();
      LoRa.print(sensorData);
      LoRa.endPacket();
      xSemaphoreGive(loraMutex);
      
      Serial.print("Send sensor data: ");
      Serial.println(sensorData);
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Task untuk memproses GPS
void gpsTask(void * parameter) {
  while(1) {
    processGPS();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  
  pinMode(BUZZER_PIN, OUTPUT);
  
  dht.begin();
  setupMotors();
  setupUltrasonicSensors();

  if (!startLoRA()) {
    Serial.println("LoRa failed to start!");
    return;
  }

  // Inisialisasi GPS
  Serial.println("GPS Initialization...");
  GPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  unsigned long startTime = millis();
  bool gpsDetected = false;

  while ((millis() - startTime) < 5000) {
    if (GPS.available()) {
      gpsDetected = true;
      break;
    }
    delay(100);
  }
  
  if (gpsDetected) {
    Serial.println("GPS Initialization OK!");
  } else {
    Serial.println("Warning: GPS Not Detected. Check the Connection!");
  }

  // Inisialisasi dan kalibrasi sensor MQ-135
  MQ135.init();
  calibrateMQ135(MQ135, dht);

  // Buat mutex untuk LoRa
  loraMutex = xSemaphoreCreateMutex();
  if (loraMutex == NULL) {
    Serial.println("Mutex creation failed!");
    return;
  }

  // Buat task untuk masing-masing fungsi
  xTaskCreatePinnedToCore(
    commandTask,    // Fungsi task
    "CommandTask",  // Nama task
    10000,         // Stack size
    NULL,          // Parameter
    1,             // Prioritas
    NULL,          // Handle task
    CORE_0         // Core yang digunakan
  );

   xTaskCreatePinnedToCore(
    sensorTask,     
    "SensorTask",   
    10000,         
    NULL,          
    1,             
    NULL,          
    CORE_1         
  );

  xTaskCreatePinnedToCore(
    gpsTask,        
    "GPSTask",      
    10000,         
    NULL,          
    1,             
    NULL,          
    CORE_1         
  );

  Serial.println("Setup complete");
}

void loop() {
  vTaskDelete(NULL);
}
