#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// Capteur de distance et de son
const int trig_pin = 5;
const int echo_pin = 18;
const int sound_sensor_pin = 34;
const int led_pin = 23;

// Vitesse du son
#define SOUND_SPEED 340
#define TRIG_PULSE_DURATION_US 10
#define SOUND_THRESHOLD 200

// Objets
WebSocketsClient webSocket;
StaticJsonDocument<200> jsonDoc;

// Paramètres WebSocket
const char* websocket_host = "192.168.10.250:"; // Remplacez par l'IP de votre Mac
const int websocket_port = 8080; // Port de votre serveur WebSocket
const char* websocket_path = "/say"; // route de connexion pour les ws


long ultrason_duration;
float distance_cm;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000; // Envoyer des données toutes les secondes

// Wifi
const char* ssid = "Cudy-EFFC";
const char* password = "33954721";



void setup() {
  Serial.begin(115200);

  // Configuration WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(100);
  }

  Serial.println("\nConnecté au réseau WiFi");
  Serial.print("IP locale ESP32: ");
  Serial.println(WiFi.localIP());

  // Configuration des pins
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(sound_sensor_pin, INPUT);
  pinMode(led_pin, OUTPUT);

  // Configuration WebSocket
  webSocket.begin(websocket_host, websocket_port, websocket_path);
  Serial.println("websocket :");
  Serial.print(websocket_host);
  Serial.print(websocket_port);
  Serial.print(websocket_path);
  webSocket.onEvent(webSocketEvent);
}

// Callback WebSocket
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED:
      Serial.println("WebSocket connecté");
      // send message to server when Connected
			webSocket.sendTXT("Connected");
      break;
    case WStype_DISCONNECTED:
      Serial.println("WebSocket déconnecté");
      break;
    case WStype_TEXT:
      Serial.print("Message reçu: ");
      Serial.println((char*)payload);
      break;
  }
}

void loop() {
  webSocket.loop(); // Maintenir la connexion WebSocket

  // Mesure de distance
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(TRIG_PULSE_DURATION_US);
  digitalWrite(trig_pin, LOW);

  ultrason_duration = pulseIn(echo_pin, HIGH);
  distance_cm = ultrason_duration * SOUND_SPEED / 2 * 0.0001;

  // Détection de souffle
  int sound_level = analogRead(sound_sensor_pin);

  // Envoyer les données périodiquement
  if (millis() - lastSendTime > sendInterval) {
    // Préparer le JSON
    jsonDoc.clear();
    jsonDoc["distance"] = distance_cm;
    jsonDoc["sound_level"] = sound_level;

    // Convertir en chaîne
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Envoyer via WebSocket
    webSocket.sendTXT(jsonString);

    // Déboguer
    Serial.print("Envoyé: ");
    Serial.println(jsonString);

    // Mise à jour du temps
    lastSendTime = millis();
  }

  // Logique de détection
  if (distance_cm < 30) {
    Serial.println("Objet detecté");
  }

  if (sound_level > SOUND_THRESHOLD) {
    Serial.println("Souffle détecté !");
    digitalWrite(led_pin, HIGH);
    delay(1000);
    digitalWrite(led_pin, LOW);
  }

  delay(100); // Petit délai pour stabilité
}