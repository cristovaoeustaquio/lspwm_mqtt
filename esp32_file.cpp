#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define RXD2 16
#define TXD2 17
#define BAUDRATE 115200
#define DSP_PIN 15
#define SET_PIN 13
#define INTERVAL 1000

const char *ssid = "";       // SSID / nome da rede WiFi
const char *password = ""; // Senha da rede WiFi

const char *mqttServer = "test.mosquitto.org"; // Endereço IP do Mosquitto Broker
const int mqttPort = 1883;                     // Porta do Mosquitto Broker

const char *topic1 = "lspwm/voltage";
const char *topic2 = "lspwm/freq";
const char *topic3 = "lspwm/modulation";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

uint32_t time_1 = 0;
uint8_t data[3];
volatile uint16_t voltage = 0;
volatile uint16_t frequency = 0;
volatile uint16_t modulation = 0;

void setupWiFi()
{
  delay(10);
  Serial.println();
  Serial.print("Conectando-se à rede Wi-Fi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.print("Connecting to broker MQTT: ");
    Serial.println(mqttServer);

    if (mqttClient.connect("lspwmClient"))
    {
      Serial.println("Broker MQTT connected!");
    }
    else
    {
      Serial.print("Failed broker connection. State: ");
      Serial.print(mqttClient.state());
      Serial.println("Trying to reconnect...");
      delay(1000);
    }
  }
}

void set_values(uint16_t *data_from_dsp)
{
  voltage = data[0];
  frequency = data[1];
  modulation = data[2];
}

void setup()
{
  Serial.begin(BAUDRATE);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  setupWiFi();
  mqttClient.setServer(mqttServer, mqttPort);
  pinMode(DSP_PIN, OUTPUT); // sets the digital pin 15 as output
  pinMode(SET_PIN, INPUT_PULLUP);
  digitalWrite(DSP_PIN, LOW);
}
String recebido = "";
uint16_t c;
void loop()
{
  if (millis() >= time_1 + INTERVAL)
  {
    time_1 += INTERVAL;
    if (digitalRead(SET_PIN))
    {
      digitalWrite(DSP_PIN, HIGH);
      if (Serial1.available() > 0)
      {
        Serial1.readBytes(data, 3);
        Serial.println(data[0]);
      }
      if (mqttClient.connected())
      {
        mqttClient.publish(topic1, String((data[0]-100)*15).c_str(), 1);
        mqttClient.publish(topic2, String(data[1]).c_str(), 1);
        mqttClient.publish(topic3, String((float)data[2]/10).c_str(), 1);
        mqttClient.loop();
      }
      else
        reconnectMQTT();
      digitalWrite(DSP_PIN, LOW);
    }
  }
  
}
