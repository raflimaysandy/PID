#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Motor control pins
const int in1 = 26;
const int in2 = 27;
const int pwmPin = 33;

// PWM config
const int pwmChannel = 0;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// IR sensor
const int sensorPin = 25;

// PID constants
float Kp = 0.0015525, Ki = 0.04166, Kd = 1.4464e-05;
float setpointRPM = 0, targetSetpointRPM = 0;
float inputRPM = 0, outputPWM = 0;
float error, previousError = 0, integral = 0;
float previousValidRPM = 0;
float rampRate = 50;

volatile int pulseCount = 0;
unsigned long lastRPMTime = 0, lastMQTTTime = 0;

// WiFi & MQTT
const char* ssid = "POCO M4 Pro";
const char* password = "terlalusulit";
const char* mqtt_server = "7a3adf123d584eb0b1c891a0cb842c35.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "isp32";
const char* mqtt_password = "ISPsogem22";

const char* level_topic = "motor/pid_level";       
const char* rpm_topic = "motor/rpm";
const char* pwm_topic = "motor/pwm";
const char* direction_topic = "motor/direction";   

WiFiClientSecure espClient;
PubSubClient client(espClient);

int motorDirection = 0;  // 1 = maju, 2 = mundur, 0 = stop

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

float getBasePWM(float rpmTarget) {
  float pwm = (rpmTarget - 334.85) / 6.9794;
  return constrain(pwm, 0, 255);
}

void IRAM_ATTR handlePulse() {
  pulseCount++;
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      client.subscribe(level_topic);
      client.subscribe(direction_topic);
    } else {
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];

  if (String(topic) == level_topic) {
    if (message == "0") targetSetpointRPM = 0;
    else if (message == "1") targetSetpointRPM = 900;
    else if (message == "2") targetSetpointRPM = 1500;
    else if (message == "3") targetSetpointRPM = 2000;
  } else if (String(topic) == direction_topic) {
    ledcWrite(pwmChannel, 0); // matikan PWM dulu
    delay(10);

    if (message == "1") motorDirection = 1;
    else if (message == "2") motorDirection = 2;
    else motorDirection = 0;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), handlePulse, FALLING);

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pwmPin, pwmChannel);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect();

  lastRPMTime = millis();
  lastMQTTTime = millis();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long currentTime = millis();

  if (currentTime - lastRPMTime >= 100) {
    // ramping setpoint
    if (setpointRPM < targetSetpointRPM) {
      setpointRPM += rampRate;
      if (setpointRPM > targetSetpointRPM) setpointRPM = targetSetpointRPM;
    } else if (setpointRPM > targetSetpointRPM) {
      setpointRPM -= rampRate;
      if (setpointRPM < targetSetpointRPM) setpointRPM = targetSetpointRPM;
    }

    noInterrupts();
    int pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float rpm = (pulses / 20.0) * (60.0 / 0.1);

    if (abs(rpm - previousValidRPM) > 500) {
      rpm = previousValidRPM;
    } else {
      previousValidRPM = rpm;
    }

    inputRPM = rpm;

    error = setpointRPM - inputRPM;
    integral += error * 0.1;
    float derivative = (error - previousError) / 0.1;
    float pidOutput = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    float basePWM = getBasePWM(setpointRPM);
    outputPWM = basePWM + pidOutput;
    outputPWM = constrain(outputPWM, 0, 255);

     // Jika target RPM adalah 0, perlahan turunkan outputPWM ke 0
    static float previousPWM = 0;
    float smoothing = 0.2;

    if (targetSetpointRPM == 0) {
      outputPWM = previousPWM * (1.0 - smoothing);  // pelan-pelan turun ke 0
    } else {
      outputPWM = previousPWM + (outputPWM - previousPWM) * smoothing;
    }
    
    previousPWM = outputPWM;

    // set arah motor dulu
    ledcWrite(pwmChannel, 0);
    delay(10);

    if (motorDirection == 1) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else if (motorDirection == 2) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }

    // aktifkan PWM
    ledcWrite(pwmChannel, (int)outputPWM);

    // Tampilkan data ke Serial Plotter
    Serial.print(inputRPM);
    Serial.print("\t");  // Bisa juga pakai ","
    Serial.println(outputPWM);

    lastRPMTime = currentTime;
  }

  if (currentTime - lastMQTTTime >= 1000) {
    char rpmStr[16], pwmStr[16];
    dtostrf(inputRPM, 6, 1, rpmStr);
    dtostrf(outputPWM, 6, 1, pwmStr);
    client.publish(rpm_topic, rpmStr);
    client.publish(pwm_topic, pwmStr);
    lastMQTTTime = currentTime;
  }
}