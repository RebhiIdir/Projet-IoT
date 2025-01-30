#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <Adafruit_NeoPixel.h>  // Bibliothèque pour contrôler le LED Ring

// Définition des pins
const int DHT_PIN = 27;
const int PIR_PIN = 8;  
const int LDR_PIN = 35;
const int RED_PIN = 17;
const int GREEN_PIN = 19;
const int BLUE_PIN = 18;
const int RING_PIN = 9; // Pin pour le LED Ring

DHTesp dhtSensor;

// Informations de connexion Wi-Fi et MQTT
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "purplecuckoo-sxx6ui.a03.euc1.aws.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "idir.rebhi";
const char* mqtt_password = "CORE64@bob@";

WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;

// Paramètres du capteur LDR
const float RL10 = 50;

// Initialisation du LED Ring
Adafruit_NeoPixel ring = Adafruit_NeoPixel(16, RING_PIN, NEO_GRB + NEO_KHZ800);

// Fonction de connexion Wi-Fi
void setup_wifi() {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.println(WiFi.localIP());
}

// Callback MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    String message = String((char*)payload);
    Serial.printf("Message received on topic %s: %s\n", topic, message.c_str());

    // Contrôle des LED RGB via MQTT
    if (strcmp(topic, "led/r") == 0) {
        int redValue = message.toInt();
        analogWrite(RED_PIN, redValue);
    } else if (strcmp(topic, "led/g") == 0) {
        int greenValue = message.toInt();
        analogWrite(GREEN_PIN, greenValue);
    } else if (strcmp(topic, "led/b") == 0) {
        int blueValue = message.toInt();
        analogWrite(BLUE_PIN, blueValue);
    }

    // Mise à jour de l'état de la LED RGB
    if (strcmp(topic, "led/r") == 0 || strcmp(topic, "led/g") == 0 || strcmp(topic, "led/b") == 0) {
        int currentRed = analogRead(RED_PIN);
        int currentGreen = analogRead(GREEN_PIN);
        int currentBlue = analogRead(BLUE_PIN);
        String payload = "R:" + String(currentRed) + ",G:" + String(currentGreen) + ",B:" + String(currentBlue);
        client.publish("status/LED", payload.c_str());
    }

    // Contrôle du LED Ring via MQTT
    if (strcmp(topic, "LEDRING") == 0) {
        if (message == "ON") {
            for (int i = 0; i < ring.numPixels(); i++) {
                ring.setPixelColor(i, ring.Color(255, 255, 255)); // Allume tous les pixels en blanc
            }
            ring.show();
            client.publish("status/LED_RING", "ON");
        } else if (message == "OFF") {
            for (int i = 0; i < ring.numPixels(); i++) {
                ring.setPixelColor(i, ring.Color(0, 0, 0)); // Éteint tous les pixels
            }
            ring.show();
            client.publish("status/LED_RING", "OFF");
        }
    }
}

// Reconnexion MQTT
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32Client-" + String(random(0xffff), HEX);
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("Connected");
            // Les topics aux quels l'ESP32 s'est abonnée
            client.subscribe("led/r");
            client.subscribe("led/g");
            client.subscribe("led/b");
            client.subscribe("LEDRING");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

float getLux(int ldrValue) {
    if (ldrValue == 0) {
        return 0.0;
    }

    float voltage = ldrValue / 4095.0 * 3.3;
    if (voltage <= 0 || voltage >= 3.3) {
        return 0.0;
    }

    float resistance = 2000 * voltage / (3.3 - voltage);
    if (resistance <= 0) {
        return 0.0;
    }

    // Approximation de la luminosité (LUX)
    float lux = RL10 * 1e3 / resistance;
    return lux;
}

void setup() {
    Serial.begin(115200);
    setup_wifi();
    espClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
    pinMode(PIR_PIN, INPUT);
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    ring.begin();  // Initialisation du LED Ring
    ring.show();   // Assure que les LEDs sont éteintes au début
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    
    unsigned long now = millis();
    if (now - lastMsg > 2000) { // Envoi des données toutes les 2 secondes
        lastMsg = now;

        // Lecture de la température et de l'humidité
        TempAndHumidity data = dhtSensor.getTempAndHumidity();
        if (!isnan(data.temperature) && !isnan(data.humidity)) {
            client.publish("sensor/temperature", String(data.temperature, 2).c_str());
            client.publish("sensor/humidity", String(data.humidity, 1).c_str());
            client.publish("status/dht22", "OK"); // Capteur DHT fonctionnel
        } else {
            Serial.println("Erreur de lecture du capteur DHT!");
            client.publish("status/dht22", "ERROR"); // Erreur du capteur DHT
        }

        // Lecture du PIR (mouvement détecté)
        int motion = digitalRead(PIR_PIN);
        client.publish("sensor/motion", motion ? "1" : "0");
        client.publish("status/PIR", "OK"); // PIR fonctionnel

        // Contrôle du LED Ring en fonction du capteur PIR
        if (motion == HIGH) {
            for (int i = 0; i < ring.numPixels(); i++) {
                ring.setPixelColor(i, ring.Color(255, 255, 255)); // Allume tous les pixels en blanc
            }
            ring.show();
            client.publish("status/LED_RING", "ON");
        } else {
            for (int i = 0; i < ring.numPixels(); i++) {
                ring.setPixelColor(i, ring.Color(0, 0, 0)); // Éteint tous les pixels
            }
            ring.show();
            client.publish("status/LED_RING", "OFF");
        }

        // Lecture de la luminosité (LDR)
        int ldrValue = analogRead(LDR_PIN);

        if (ldrValue >= 0 && ldrValue <= 4095) {
            float lux = getLux(ldrValue);
            client.publish("sensor/light", String(lux, 2).c_str());
            client.publish("status/LDR", "OK"); // LDR fonctionnel
        } else {
            client.publish("sensor/light", "nan");  // Publier "nan" si la lecture échoue
            client.publish("status/LDR", "ERROR"); // Erreur du capteur LDR
        }

        // Vérification des LED RGB
        if (analogRead(RED_PIN) > 0 || analogRead(GREEN_PIN) > 0 || analogRead(BLUE_PIN) > 0) {
            client.publish("status/LED", "ON");
        } else {
            client.publish("status/LED", "OFF");
        }
    }
}
