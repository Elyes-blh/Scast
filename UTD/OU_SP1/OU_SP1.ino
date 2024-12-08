
// Station de Pompage  OU_SP1
//26/11/2024 ota ok + niveau , états pompes et débordement
#include <WiFi.h>
#include <SPIFFS.h>
#include "Update.h"
#include <WiFiClientSecure.h>
#include <ThingSpeak.h>


// Définir les informations d'identification WiFi
#define ssid "Airbox"
#define password "Sc@st111"

// Définir les détails du serveur et le chemin des fichiers
#define HOST "raw.githubusercontent.com"
#define VERSION_PATH "/Elyes-blh/Scast/refs/heads/main/UTD/OU_SP1/version.txt"
#define FIRMWARE_PATH "/Elyes-blh/Scast/refs/heads/main/UTD/OU_SP1/SpV6.bin"
#define PORT 443
#define FILE_NAME "firmware.bin"

// Version actuelle du firmware
 String currentVersion = "1.0.0";
 unsigned long previousMillis = 0; // Dernière vérification
 unsigned long previousMillis_P = 1000; // Dernière vérification Publication
 const unsigned long updateInterval = 600000; // Fréquence en millisecondes (1 heure = 3600000 ms)
 //pour publication sur ThingSpeak
 WiFiClient client; // Nécessaire pour ThingSpeak
 unsigned long myChannelNumber = 2779217; // Votre ID de canal
 const char *myWriteAPIKey = "BTHJDCR1R3GGZMJB"; // Clé API d'écriture
 #define API_ReadKey "Z0QGMT4B3H2FPWKI"

 const unsigned long PublishInterval=60000; //Fréquence de publication sur channel 20000= 20s
 
// Pins des pompes et du capteur
    const int Pin_P1 = 25;  // Pompe 1
    const int Pin_P2 = 26;  // Pompe 2
    const int Pin_P3 = 27;  // Pompe 3
    const int Pin_P4 = 14;  // Pompe 4
    const int Pin_D = 35;   // Détecteur de déversement
    const int pin_ADC = 34; // Broche ADC pour le niveau

    // Paramètres de calcul du niveau
    const float tension_min = 0.0;  // Tension correspondant à 0 cm
    const float tension_max = 3.28;  // Tension correspondant à 700 cm
    const int niveau_max = 800;     // Niveau maximal en cm




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

// Configurer les broches en entrée
  pinMode(Pin_P1, INPUT_PULLDOWN);
  pinMode(Pin_P2, INPUT_PULLDOWN);
  pinMode(Pin_P3, INPUT_PULLDOWN);
  pinMode(Pin_P4, INPUT_PULLDOWN);
  pinMode(Pin_D, INPUT_PULLDOWN);
  pinMode(pin_ADC, INPUT);
  ThingSpeak.begin(client); // Initialisation de ThingSpeak
}

void loop() {
  unsigned long currentMillis = millis();

  // Vérifie si le temps écoulé dépasse l'intervalle spécifié
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
     // Lecture des états**
    int etat_P1 = digitalRead(Pin_P1);
    int etat_P2 = digitalRead(Pin_P2);
    int etat_P3 = digitalRead(Pin_P3);
    int etat_P4 = digitalRead(Pin_P4);
    int etat_deversement = digitalRead(Pin_D);

    // Lecture et calcul du niveau
    float tension_ADC = analogRead(pin_ADC) * (3.3 / 4095.0); // Conversion ADC -> tension
    int niveau = calculerNiveau(tension_ADC);
    //publication sur channel
     if (currentMillis - previousMillis_P >= PublishInterval) {
     previousMillis_P = currentMillis;
     publierSurThingSpeak(niveau, etat_P1, etat_P2, etat_P3, etat_P4, etat_deversement);
    }
}
void publierSurThingSpeak(int niveau, int P1, int P2, int P3, int P4, int deversement) {
  // Remplir les champs de ThingSpeak
  ThingSpeak.setField(1, niveau);
  ThingSpeak.setField(2, P1);
  ThingSpeak.setField(3, P2);
  ThingSpeak.setField(4, P3);
  ThingSpeak.setField(5, P4);
  ThingSpeak.setField(6, deversement);

  // Envoyer les données
  int httpCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (httpCode == 200) {
    Serial.println("Données publiées avec succès !");
  } else {
    Serial.println("Erreur lors de la publication : " + String(httpCode));
  }
}
int calculerNiveau(float tension) {
  if (tension <= tension_min) return 0; // Niveau minimum
  if (tension >= tension_max) return niveau_max; // Niveau maximum

  // Interpolation linéaire pour calculer le niveau
  return (int)((tension - tension_min) * niveau_max / (tension_max - tension_min));
}

// Calcul du niveau en cm en fonction de la tension mesurée**
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
