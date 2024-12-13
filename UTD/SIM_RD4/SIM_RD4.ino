// Station de Pompage SIM_RD4 avec calcul de débit
#include <WiFi.h>
#include <SPIFFS.h>
#include "Update.h"
#include <WiFiClientSecure.h>
#include <ThingSpeak.h>

// Informations WiFi
#define ssid "Airbox"
#define password "Sc@st111"

// Détails du serveur et fichiers
#define HOST "raw.githubusercontent.com"
#define VERSION_PATH "/Elyes-blh/Scast/refs/heads/main/UTD/SIM_RD4/version.txt"
#define FIRMWARE_PATH "/Elyes-blh/Scast/refs/heads/main/UTD/SIM_RD4/SpV6.bin"
#define PORT 443
#define FILE_NAME "firmware.bin"

// Version actuelle
String currentVersion = "1.0.0";
unsigned long previousMillis = 0;
unsigned long previousMillis_P = 1000;
const unsigned long updateInterval = 600000;

// ThingSpeak
WiFiClient client;
unsigned long myChannelNumber = 2784680;
const char *myWriteAPIKey = "13O5R00KE5YJBSIT";
#define API_ReadKey "YH7PU8PNPEW2ZOYY"
const unsigned long PublishInterval = 60000;

// Pins et paramètres capteurs
const int Pin_P1 = 25;
const int Pin_P2 = 26;
const int Pin_P3 = 27;
const int Pin_P4 = 14;
const int Pin_D = 35;
const int pin_ADC = 36;

const float tension_min = 0.17;
const float tension_max = 3.28;
const int niveau_max = 650;
const float surface_bache = 7.776; // Surface en m²

float sommeNiveaux = 0;
int nombreEchantillons = 0;
float niveauMoyen = 0;
float dernierNiveau = 0;
unsigned long dernierTemps = 0;

void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  connectToWiFi();

  if (checkForNewVersion()) {
    getFileFromServer();
    performOTAUpdateFromSPIFFS();
  } else {
    Serial.println("Firmware à jour, aucune mise à jour nécessaire.");
  }

  pinMode(Pin_P1, INPUT_PULLDOWN);
  pinMode(Pin_P2, INPUT_PULLDOWN);
  pinMode(Pin_P3, INPUT_PULLDOWN);
  pinMode(Pin_P4, INPUT_PULLDOWN);
  pinMode(Pin_D, INPUT_PULLDOWN);
  pinMode(pin_ADC, INPUT_PULLDOWN);
  ThingSpeak.begin(client);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;
    Serial.println("Vérification périodique des mises à jour...");
    if (checkForNewVersion()) {
      getFileFromServer();
      performOTAUpdateFromSPIFFS();
    } else {
      Serial.println("Aucune mise à jour disponible.");
    }
  }

  int etat_P1 = digitalRead(Pin_P1);
  int etat_P2 = digitalRead(Pin_P2);
  int etat_P3 = digitalRead(Pin_P3);
  int etat_P4 = digitalRead(Pin_P4);
  int etat_deversement = digitalRead(Pin_D);

  float tension_ADC = analogRead(pin_ADC) * (3.3 / 4095.0);
  int niveau = calculerNiveau(tension_ADC);

  sommeNiveaux += niveau;
  nombreEchantillons++;

  if (currentMillis - previousMillis_P >= PublishInterval) {
    previousMillis_P = currentMillis;

    if (nombreEchantillons > 0) {
      niveauMoyen = sommeNiveaux / nombreEchantillons;
      sommeNiveaux = 0;
      nombreEchantillons = 0;

      float debit = calculerDebit(niveauMoyen, dernierNiveau, currentMillis - dernierTemps);
      dernierNiveau = niveauMoyen;
      dernierTemps = currentMillis;

      publierSurThingSpeak(niveauMoyen, debit, etat_P1, etat_P2, etat_P3, etat_P4, etat_deversement);
    }
  }

  delay(500);
}

void publierSurThingSpeak(int niveau, float debit, int P1, int P2, int P3, int P4, int deversement) {
  ThingSpeak.setField(1, niveau);
  ThingSpeak.setField(2, P1);
  ThingSpeak.setField(3, P2);
  ThingSpeak.setField(4, P3);
  ThingSpeak.setField(5, debit);
  ThingSpeak.setField(6, deversement);

  int httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (httpCode == 200) {
    Serial.println("Données publiées avec succès !");
  } else {
    Serial.println("Erreur lors de la publication : " + String(httpCode));
  }
}

int calculerNiveau(float tension) {
  if (tension <= tension_min) return 0;
  if (tension >= tension_max) return niveau_max;
  return (int)((tension - tension_min) * niveau_max / (tension_max - tension_min));
}

float calculerDebit(float niveauActuel, float niveauPrecedent, unsigned long deltaTempsMs) {
  if (deltaTempsMs == 0) return 0;
  float variationNiveau = niveauActuel - niveauPrecedent;
  float variationVolume = variationNiveau * surface_bache / 100.0; // En m³
  float tempsHeures = deltaTempsMs / 3600000.0; // Conversion ms -> heures
  return variationVolume / tempsHeures; // Débit en m³/h
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connexion au WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connecté");
}



bool checkForNewVersion() {
  WiFiClientSecure client;
  client.setInsecure();

  if (client.connect(HOST, PORT)) {
    Serial.println("Connecté au serveur pour vérifier la version");

    client.print("GET " + String(VERSION_PATH) + " HTTP/1.1\r\n");
    client.print("Host: " + String(HOST) + "\r\n");
    client.println("Connection: close\r\n");
    client.println();

    String headers = "";
    bool endOfHeaders = false;
    String httpResponseCode = "error";

    // Lecture des headers
    while (client.connected() && !endOfHeaders) {
      if (client.available()) {
        char c = client.read();
        headers += c;
        if (headers.startsWith("HTTP/1.1")) {
          httpResponseCode = headers.substring(9, 12);
        }
        if (headers.endsWith("\r\n\r\n")) {
          endOfHeaders = true;
        }
      }
    }

    if (httpResponseCode != "200") {
      Serial.println("Erreur lors de la récupération de la version: " + httpResponseCode);
      return false;
    }

    // Lecture de la version
    String latestVersion = "";
    while (client.available()) {
      char c = client.read();
      latestVersion += c;
    }
    latestVersion.trim();

    Serial.println("Version actuelle: " + currentVersion);
    Serial.println("Dernière version disponible: " + latestVersion);

    return currentVersion != latestVersion;
  } else {
    Serial.println("Échec de la connexion pour vérifier la version.");
    return false;
  }
}

void getFileFromServer() {
  WiFiClientSecure client;
  client.setInsecure();

  if (client.connect(HOST, PORT)) {
    Serial.println("Connecté au serveur pour télécharger le firmware");

    client.print("GET " + String(FIRMWARE_PATH) + " HTTP/1.1\r\n");
    client.print("Host: " + String(HOST) + "\r\n");
    client.println("Connection: close\r\n");
    client.println();

    File file = SPIFFS.open("/" + String(FILE_NAME), FILE_WRITE);
    if (!file) {
      Serial.println("Échec de l'ouverture du fichier pour écriture");
      return;
    }

    bool endOfHeaders = false;
    String headers = "";
    const size_t bufferSize = 1024;
    uint8_t buffer[bufferSize];

    // Lecture des headers
    while (client.connected() && !endOfHeaders) {
      if (client.available()) {
        char c = client.read();
        headers += c;
        if (headers.endsWith("\r\n\r\n")) {
          endOfHeaders = true;
        }
      }
    }

    // Lecture et écriture des données
    while (client.connected()) {
      if (client.available()) {
        size_t bytesRead = client.readBytes(buffer, bufferSize);
        file.write(buffer, bytesRead);
      }
    }
    file.close();
    client.stop();
    Serial.println("Fichier firmware téléchargé avec succès");
  } else {
    Serial.println("Échec de la connexion pour télécharger le firmware");
  }
}

void performOTAUpdateFromSPIFFS() {
  File file = SPIFFS.open("/" + String(FILE_NAME), FILE_READ);
  if (!file) {
    Serial.println("Échec de l'ouverture du fichier firmware");
    return;
  }

  Serial.println("Début de la mise à jour OTA...");
  size_t fileSize = file.size();
  Serial.println("Taille du fichier : " + String(fileSize) + " octets");

  if (!Update.begin(fileSize, U_FLASH)) {
    Serial.println("Échec de la préparation de la mise à jour");
    return;
  }

  Update.writeStream(file);

  if (Update.end()) {
    Serial.println("Mise à jour réussie");
    if (Update.isFinished()) {
      Serial.println("Redémarrage en cours...");
      delay(2000);
      ESP.restart();
    }
  } else {
    Serial.println("Erreur pendant la mise à jour : " + String(Update.getError()));
  }

  file.close();
}

