// Station de Pompage MED_SRJ2 avec calcul de débit
#include <WiFi.h>
#include <SPIFFS.h>
#include "Update.h"
#include <WiFiClientSecure.h>
#include <ThingSpeak.h>

#include <TimeLib.h>  // Librairie Time pour gérer l'heure
#include <NTPClient.h> // Librairie NTPClient


// Informations WiFi
#define ssid "Airbox"
#define password "Sc@st111"

// Détails du serveur et fichiers
#define HOST "raw.githubusercontent.com"
#define VERSION_PATH "/Elyes-blh/Scast/refs/heads/main/MI_SRJ22/version.txt"
#define FIRMWARE_PATH "/Elyes-blh/Scast/refs/heads/main/MI_SRJ22/SpV6.bin"
#define DEBITS_PATH "/Elyes-blh/Scast/refs/heads/main/MI_SRJ22/debits.txt"
#define PORT 443
#define FILE_NAME "firmware.bin"

// Version actuelle
String currentVersion = "1.0.0";
unsigned long previousMillis = 0;
unsigned long previousMillis_P = 1000;
const unsigned long updateInterval = 600000;

// ThingSpeak
WiFiClient client;
unsigned long myChannelNumber = 2782228;
const char *myWriteAPIKey = "QDWK441D56WWIEX1";
#define API_ReadKey "ZKIVM07PU7P1JRPA"
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
const int niveau_max = 800;
const float surface_bache = 7; // Surface en m²

float sommeNiveaux = 0;
int nombreEchantillons = 0;
float niveauMoyen = 0;
float dernierNiveau = 0;
unsigned long dernierTemps = 0;

// Débits des pompes (par défaut)
float debit_P1 = 10.0;
float debit_P2 = 12.0;
float debit_P3 = 15.0;
float debit_P4 = 18.0;
float debit_P12, debit_P13, debit_P14, debit_P23, debit_P24, debit_P34;
float debit_P123, debit_P234, debit_P134, debit_P124, debit_P1234;

WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 3600, 60000);  // 3600 secondes de décalage pour UTC+1

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

  // Initialisation de l'heure via NTP
  timeClient.begin();
  timeClient.update();  // Obtenir l'heure NTP
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
 timeClient.update();

  // Obtenez l'heure actuelle (en heure locale après ajustement du fuseau horaire)
  int currentHour = hour();  // Utilise TimeLib pour obtenir l'heure actuelle

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



bool fetchDebitsFromGitHub() {
  WiFiClientSecure client;
  client.setInsecure();

  if (client.connect(HOST, PORT)) {
    String url = String(DEBITS_PATH);
    client.println("GET " + url + " HTTP/1.1");
    client.println("Host: " + String(HOST));
    client.println("Connection: close");
    client.println();

    while (client.connected() || client.available()) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        if (line.startsWith("#")) continue; // Ignore les commentaires

        sscanf(line.c_str(), "P1=%f P2=%f P3=%f P4=%f P12=%f P13=%f P14=%f P23=%f P24=%f P34=%f P123=%f P234=%f P134=%f P124=%f P1234=%f",
               &debit_P1, &debit_P2, &debit_P3, &debit_P4, &debit_P12, &debit_P13, &debit_P14, &debit_P23, &debit_P24, &debit_P34,
               &debit_P123, &debit_P234, &debit_P134, &debit_P124, &debit_P1234);
        return true;
      }
    }
  }
  return false;
}

float calculeVolumeRefoulement(int etat_P1, int etat_P2, int etat_P3, int etat_P4) {
  // Combinaisons possibles
  if (etat_P1 && etat_P2 && etat_P3 && etat_P4) return debit_P1234;
  if (etat_P1 && etat_P2 && etat_P3) return debit_P123;
  if (etat_P2 && etat_P3 && etat_P4) return debit_P234;
  if (etat_P1 && etat_P3 && etat_P4) return debit_P134;
  if (etat_P1 && etat_P2 && etat_P4) return debit_P124;
  if (etat_P1 && etat_P2) return debit_P12;
  if (etat_P1 && etat_P3) return debit_P13;
  if (etat_P1 && etat_P4) return debit_P14;
  if (etat_P2 && etat_P3) return debit_P23;
  if (etat_P2 && etat_P4) return debit_P24;
  if (etat_P3 && etat_P4) return debit_P34;

  // Combinaisons simples
  if (etat_P1) return debit_P1;
  if (etat_P2) return debit_P2;
  if (etat_P3) return debit_P3;
  if (etat_P4) return debit_P4;

  return 0.0; // Aucune pompe active
}

void calibrationDebits() {
  if (hour() >= 2 && hour() <= 5) { // Calibration entre 2h et 5h
    float nouveau_debit_P1 = mesurerDebit(Pin_P1);
    float nouveau_debit_P2 = mesurerDebit(Pin_P2);
    float nouveau_debit_P3 = mesurerDebit(Pin_P3);
    float nouveau_debit_P4 = mesurerDebit(Pin_P4);

    // Recalcul des combinaisons
    float nouveau_debit_P12 = nouveau_debit_P1 + nouveau_debit_P2;
    float nouveau_debit_P13 = nouveau_debit_P1 + nouveau_debit_P3;
    float nouveau_debit_P14 = nouveau_debit_P1 + nouveau_debit_P4;
    float nouveau_debit_P23 = nouveau_debit_P2 + nouveau_debit_P3;
    float nouveau_debit_P24 = nouveau_debit_P2 + nouveau_debit_P4;
    float nouveau_debit_P34 = nouveau_debit_P3 + nouveau_debit_P4;

    float nouveau_debit_P123 = nouveau_debit_P1 + nouveau_debit_P2 + nouveau_debit_P3;
    float nouveau_debit_P234 = nouveau_debit_P2 + nouveau_debit_P3 + nouveau_debit_P4;
    float nouveau_debit_P134 = nouveau_debit_P1 + nouveau_debit_P3 + nouveau_debit_P4;
    float nouveau_debit_P124 = nouveau_debit_P1 + nouveau_debit_P2 + nouveau_debit_P4;
    float nouveau_debit_P1234 = nouveau_debit_P1 + nouveau_debit_P2 + nouveau_debit_P3 + nouveau_debit_P4;

    // Mise à jour des valeurs globales
    debit_P1 = nouveau_debit_P1;
    debit_P2 = nouveau_debit_P2;
    debit_P3 = nouveau_debit_P3;
    debit_P4 = nouveau_debit_P4;
    debit_P12 = nouveau_debit_P12;
    debit_P13 = nouveau_debit_P13;
    debit_P14 = nouveau_debit_P14;
    debit_P23 = nouveau_debit_P23;
    debit_P24 = nouveau_debit_P24;
    debit_P34 = nouveau_debit_P34;
    debit_P123 = nouveau_debit_P123;
    debit_P234 = nouveau_debit_P234;
    debit_P134 = nouveau_debit_P134;
    debit_P124 = nouveau_debit_P124;
    debit_P1234 = nouveau_debit_P1234;

    // Écriture des nouvelles valeurs dans le fichier sur GitHub
    uploadDebitsToGitHub();
  }
}

float mesurerDebit(int pinPompe) {
  // Mesure du débit pour une pompe donnée
  if (digitalRead(pinPompe)) {
    float niveauInitial = dernierNiveau;
    delay(5000); // Simuler une mesure sur 5 secondes
    float niveauFinal = dernierNiveau; // Simulation ou mesure réelle
    return (niveauInitial - niveauFinal) * surface_bache / 5.0; // Débit en m³/s
  }
  return 0.0;
}

void uploadDebitsToGitHub() {
  WiFiClientSecure client;
  client.setInsecure();

  if (client.connect(HOST, PORT)) {
    String data = String("P1=") + debit_P1 + " P2=" + debit_P2 + " P3=" + debit_P3 + " P4=" + debit_P4 +
                  " P12=" + debit_P12 + " P13=" + debit_P13 + " P14=" + debit_P14 +
                  " P23=" + debit_P23 + " P24=" + debit_P24 + " P34=" + debit_P34 +
                  " P123=" + debit_P123 + " P234=" + debit_P234 + " P134=" + debit_P134 +
                  " P124=" + debit_P124 + " P1234=" + debit_P1234;

    String url = String("/repos/Elyes-blh/Scast/contents/MI_SRJ22/debits.txt"); // Remplacez par l'URL de l'API
    client.println("PUT " + url + " HTTP/1.1");
    client.println("Host: " + String(HOST));
    client.println("User-Agent: ESP32");
    client.println("Content-Type: application/json");
    client.println("Authorization: Bearer YOUR_GITHUB_TOKEN"); // Remplacez par votre token
    client.println("Content-Length: " + String(data.length()));
    client.println();
    client.println(data);

    while (client.connected() || client.available()) {
      if (client.available()) {
        Serial.println(client.readString());
      }
    }
  } else {
    Serial.println("Échec de connexion au serveur GitHub.");
  }
}

