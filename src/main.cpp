#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_system.h>

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <cstring>
#include <FS.h>
#include <SD.h>
#include <ctime>

// Pines del T-Call SIM7000G
constexpr int LED_PIN     = 13;  // LED azul de la T-Call
constexpr int PWRKEY      = 4;   // Pin para encender el módem
constexpr int MODEM_TX    = 27;  // ESP32 TX hacia RX del SIM7000G
constexpr int MODEM_RX    = 26;  // ESP32 RX desde TX del SIM7000G
constexpr int CURRENT_PIN = 36;  // ADC1 para SCT013 100A/1V con bias a 1.65V
constexpr int VOLT_PIN    = 39;  // ADC1 para ZMPT101B (acoplo AC)
constexpr int I2C_SDA     = 21;
constexpr int I2C_SCL     = 22;
constexpr uint8_t LCD_ADDR = 0x27; // cambia a 0x3F si tu backpack usa esa dirección
constexpr int RELAY_PIN   = 25;
constexpr int BUTTON_PIN  = 32; // pulsador en GPIO32
// false: pulsador entre 3.3V y GPIO (activo HIGH) + INPUT_PULLDOWN
// true : pulsador entre GND  y GPIO (activo LOW)  + INPUT_PULLUP
constexpr bool BUTTON_ACTIVE_LOW = false;
constexpr bool RELAY_ACTIVE_LOW = false; // activo en alto

#define SerialMon Serial
#define SerialAT  Serial1

// Configuración de red
const char apn[]   = "web.tigo.com.pa";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT destino
const char mqttHost[] = "test.mosquitto.org";
const uint16_t mqttPort = 1883;
const char mqttUser[] = "";
const char mqttPass[] = "";
const char mqttStateTopic[] = "sim7000g/estado";
const char mqttCmdTopic[] = "sim7000g/cmd";
const char mqttGpsTopic[] = "sim7000g/gps";
const char mqttCurrentTopic[] = "sim7000g/corriente";
const char mqttVoltageTopic[] = "sim7000g/voltaje";
const char mqttAdcVTopic[] = "sim7000g/adc_v";

// Factor de calibración de voltaje: Vrms_linea = vrms_adc * voltageScale
// Ajusta con una referencia conocida
float voltageScale = 224.0f;  // ajusta según tu referencia (Vrms_linea = vrms_adc * factor)
float setpointV = 120.0f;
float rangoVolt = 0.10f; // +-10%
const unsigned long voltRetryMs = 120000; // 2 minutos antes de reintentar

TinyGsm modem(SerialAT);
TinyGsmClient netClient(modem);
PubSubClient mqtt(netClient);

const unsigned long statusIntervalMs = 10000;
unsigned long lastStatusPublish = 0;
uint32_t adcOffset = 2048;       // se calibrará al arranque
float noiseFloorA = 0.05f;       // sin uso si desactivamos recorte
const unsigned long sampleWindowMs = 800; // ventana de muestreo RMS (más corta para no perder el botón)
unsigned long lastGpsPublish = 0;
const unsigned long gpsIntervalMs = 15000; // publica GNSS cada 15s
float lastIrms = 0.0f;
float lastVolts = 0.0f;
float lastVoltAdc = 0.0f;
float lastFaultI = 0.0f;
float lastFaultV = 0.0f;
const bool DEBUG_ADC_CURRENT = true;
const unsigned long ADC_DEBUG_INTERVAL_MS = 2000;
unsigned long lastAdcDebug = 0;

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2); // ajusta dirección si es 0x3F
bool lcdReady = false;
String lastL1;
String lastL2;
unsigned long lastLcdUpdate = 0;
const unsigned long lcdRefreshMs = 700;

// Setpoints y tiempos (valores por defecto)
float setpointA = 10.0f;                // Amperios
float rangoSobre = 0.2f;                // +20%
float rangoBaja = 0.1f;                 // -10%
unsigned long tiempoRecuperacionMs = 15UL * 60UL * 1000UL; // 15 min por defecto

// Estados de protección
enum SysState { STATE_STARTUP, STATE_RUNNING, STATE_RECOVERY, STATE_FAULT, STATE_VOLT_WAIT };
SysState sysState = STATE_STARTUP;
unsigned long workStartMs = 0;
float workedBeforeOffMin = 0.0f;  // tta
unsigned long recStartMs = 0;
bool relayOn = true;
bool haveCurrent = false;
bool haveVoltage = false;
unsigned long voltOutStart = 0;
unsigned long voltWaitStart = 0;
bool voltFaultHigh = false;
float voltFaultVal = 0.0f;
// SD deshabilitada
bool manualOverride = false;
bool manualRelay = false;
bool manualOffState = false;

// Botón / edición
bool rawButtonState = HIGH;
bool lastRawButtonState = HIGH;
bool debouncedState = HIGH;
bool prevDebouncedState = HIGH;
unsigned long lastDebounceMs = 0;
const unsigned long debounceIntervalMs = 45;
unsigned long buttonPressStart = 0;
unsigned long lastIncrementMs = 0;
enum EditState { EDIT_NONE, EDIT_SETPOINT, EDIT_RECUP };
EditState editState = EDIT_NONE;
const unsigned long longPressMs = 1500;   // entrar al menú
const unsigned long savePressMs  = 2000;  // guardar valor

// Mensajes temporales en LCD
char msgLine1[17] = {0};
char msgLine2[17] = {0};
unsigned long messageUntil = 0;

// EEPROM layout
const int EEPROM_ADDR_SETPOINT = 0;               // float
const int EEPROM_ADDR_RECUP_MIN = sizeof(float);  // uint16_t
const int EEPROM_ADDR_SETPOINT_V = EEPROM_ADDR_RECUP_MIN + sizeof(uint16_t); // float
const int EEPROM_ADDR_OVER_PCT   = EEPROM_ADDR_SETPOINT_V + sizeof(float);   // float
const int EEPROM_ADDR_UNDER_PCT  = EEPROM_ADDR_OVER_PCT   + sizeof(float);   // float
const int EEPROM_ADDR_VOLT_TOL   = EEPROM_ADDR_UNDER_PCT  + sizeof(float);   // float

// Adelantos de funciones que se usan antes de definirse
void handleButton();
unsigned long getTimestamp();
void publishStatus();
bool mqttPublish(const char* topic, const char* payload, bool retain);
void publishConfig();

const char* resetReasonStr(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SW:        return "SW_RESET";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_EXT:       return "EXT_RESET";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

void powerOnModem() {
  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, LOW);
  delay(1000);
  digitalWrite(PWRKEY, HIGH);
}

void scanI2C() {
  SerialMon.println("Escaneando I2C...");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      SerialMon.print("I2C encontrado en 0x");
      SerialMon.println(addr, HEX);
      count++;
      delay(2);
    }
  }
  if (count == 0) {
    SerialMon.println("No se detectaron dispositivos I2C");
  }
}

bool connectGprs() {
  modem.sendAT("+CMNB=3");  // GSM only
  modem.waitResponse(3000);
  modem.sendAT("+CNMP=13"); // GSM only
  modem.waitResponse(3000);
  modem.sendAT("+CGDCONT=1,\"IP\",\"", apn, "\"");
  modem.waitResponse(3000);

  SerialMon.println("Esperando registro de red...");
  if (!modem.waitForNetwork(60000)) {
    SerialMon.println("No hay registro de red");
    return false;
  }
  SerialMon.println("Registrado en red");
  SerialMon.print("Operador: ");
  SerialMon.println(modem.getOperator());
  SerialMon.print("RSSI: ");
  SerialMon.println(modem.getSignalQuality());

  SerialMon.println("Conectando GPRS...");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("Fallo gprsConnect");
    return false;
  }
  if (!modem.isGprsConnected()) {
    SerialMon.println("GPRS no quedó conectado");
    return false;
  }
  SerialMon.print("GPRS conectado. IP local: ");
  SerialMon.println(modem.getLocalIP());
  return true;
}

void doMqtt() {
  mqtt.setServer(mqttHost, mqttPort);
  mqtt.setKeepAlive(30);
  mqtt.setBufferSize(512);
  mqtt.setCallback([](char* topic, uint8_t* payload, unsigned int len) {
    String t = String(topic);
    String p;
    for (unsigned int i = 0; i < len; i++) p += (char)payload[i];

    if (t == mqttCmdTopic) {
      // Comando JSON simple: {"relay":"on","setpointA":12,"overPct":0.2,"underPct":0.1,"setpointV":120,"voltTol":0.1,"recMin":15}
      String lp = p; lp.toLowerCase();
      auto getVal = [&](const char* key) -> float {
        int idx = lp.indexOf(key);
        if (idx < 0) return NAN;
        idx = lp.indexOf(':', idx);
        if (idx < 0) return NAN;
        int end = lp.indexOf(',', idx + 1);
        if (end < 0) end = lp.indexOf('}', idx + 1);
        if (end < 0) end = lp.length();
        String sub = lp.substring(idx + 1, end);
        sub.trim();
        sub.replace("\"", "");
        return sub.toFloat();
      };
      auto has = [&](const char* key) { return lp.indexOf(key) >= 0; };

      // Relay
    if (lp.indexOf("\"relay\"") >= 0) {
    if (lp.indexOf("on", lp.indexOf("\"relay\"")) >= 0) {
      manualOverride = true;
      manualRelay = true;
      manualOffState = false;
      relayOn = true;
      sysState = STATE_RUNNING;
      workStartMs = millis();
      recStartMs = 0;
      voltWaitStart = 0;
      voltOutStart = 0;
    } else if (lp.indexOf("off", lp.indexOf("\"relay\"")) >= 0) {
      manualOverride = true;
      manualRelay = false;
      manualOffState = true; // para reportar estado
      relayOn = false;
      sysState = STATE_FAULT;
    }
  }

      // Setpoints
      if (has("setpointa")) {
        float v = getVal("setpointa");
        if (isfinite(v) && v >= 1 && v <= 30) {
          setpointA = v;
          EEPROM.put(EEPROM_ADDR_SETPOINT, setpointA);
          EEPROM.commit();
        }
      }
      if (has("overpct")) {
        float v = getVal("overpct");
        if (isfinite(v) && v > 0 && v < 1) {
          rangoSobre = v;
          EEPROM.put(EEPROM_ADDR_OVER_PCT, rangoSobre);
          EEPROM.commit();
        }
      }
      if (has("underpct")) {
        float v = getVal("underpct");
        if (isfinite(v) && v > 0 && v < 1) {
          rangoBaja = v;
          EEPROM.put(EEPROM_ADDR_UNDER_PCT, rangoBaja);
          EEPROM.commit();
        }
      }
      if (has("setpointv")) {
        float v = getVal("setpointv");
        if (isfinite(v) && v >= 50 && v <= 260) {
          setpointV = v;
          EEPROM.put(EEPROM_ADDR_SETPOINT_V, setpointV);
          EEPROM.commit();
        }
      }
      if (has("volttol")) {
        float v = getVal("volttol");
        if (isfinite(v) && v > 0 && v < 1) {
          rangoVolt = v;
          EEPROM.put(EEPROM_ADDR_VOLT_TOL, rangoVolt);
          EEPROM.commit();
        }
      }
      if (has("recmin")) {
        float v = getVal("recmin");
        if (isfinite(v) && v >= 1 && v <= 1440) {
          tiempoRecuperacionMs = (unsigned long)(v * 60000UL);
          uint16_t recMin = (uint16_t)v;
          EEPROM.put(EEPROM_ADDR_RECUP_MIN, recMin);
          EEPROM.commit();
        }
      }
      if (has("getcfg")) {
        publishConfig();
      } else {
        publishConfig(); // responde con la nueva config tras cambios
      }
    }
  });

  SerialMon.print("Conectando MQTT a ");
  SerialMon.print(mqttHost);
  SerialMon.print(":");
  SerialMon.println(mqttPort);

  String clientId = String("sim7000g-") + String((uint32_t)millis(), HEX);
  if (!mqtt.connect(clientId.c_str(), mqttUser, mqttPass)) {
    SerialMon.print("MQTT connect falló, rc=");
    SerialMon.println(mqtt.state());
    return;
  }
  SerialMon.println("MQTT conectado.");
  mqtt.subscribe(mqttCmdTopic);
}

bool sampleRms(uint8_t pin, float& vrms, float& vmean) {
  uint64_t sum = 0;
  uint64_t sumsq = 0;
  uint32_t samples = 0;
  unsigned long start = millis();
  while (millis() - start < sampleWindowMs) {
    int raw = analogRead(pin);
    sum += raw;
    sumsq += (uint64_t)raw * (uint64_t)raw;
    samples++;
    if ((samples & 0x7F) == 0) { // revisa botón periódicamente para no perder pulsos
      handleButton();
    }
    delayMicroseconds(200);
  }
  if (samples == 0) return false;
  float mean = (float)sum / samples;
  float variance = (float)sumsq / samples - mean * mean;
  if (variance < 0) variance = 0;
  vmean = mean * (3.3f / 4095.0f);
  vrms = sqrtf(variance) * (3.3f / 4095.0f);
  return true;
}

bool getLbsLocation(float& lat, float& lon, float& acc) {
  modem.sendAT("+CLBS=1,1");  // ubicación por estaciones base
  // espera la respuesta con prefijo +CLBS:
  if (modem.waitResponse(10000, "+CLBS:") != 1) {
    modem.waitResponse(1000); // drena
    return false;
  }
  String line = modem.stream.readStringUntil('\n');
  line.trim();
  // Formato típico:  <err>,<lat>,<lon>,<acc>
  char buf[64];
  line.toCharArray(buf, sizeof(buf));
  float alat = 0, alon = 0, aacc = 0;
  int err = -1;
  if (sscanf(buf, "%d,%f,%f,%f", &err, &alat, &alon, &aacc) == 4 && err == 0) {
    // Algunos CLBS entregan lon,lat; intercambiamos para que quede lat,lon
    lat = alon;
    lon = alat;
    acc = aacc;
    return true;
  }
  return false;
}

void measureCurrentAndVoltage() {
  float vrmsI = 0, vmeanI = 0;
  if (sampleRms(CURRENT_PIN, vrmsI, vmeanI)) {
    float irms = vrmsI / 0.01f;  // SCT013 100A/1V -> 0.01 V/A
    bool clipped = false;
    if (irms < 0.5f) {
      irms = 0.0f; // recorte bajo
      clipped = true;
    }
    lastIrms = irms;
    haveCurrent = true;

    if (DEBUG_ADC_CURRENT && (millis() - lastAdcDebug >= ADC_DEBUG_INTERVAL_MS)) {
      lastAdcDebug = millis();
      const float meanCnt = vmeanI * (4095.0f / 3.3f);
      const float vrmsCnt = vrmsI * (4095.0f / 3.3f);
      const int rawNow = analogRead(CURRENT_PIN);
      SerialMon.print("ADC corriente: raw=");
      SerialMon.print(rawNow);
      SerialMon.print(" meanCnt=");
      SerialMon.print(meanCnt, 1);
      SerialMon.print(" vrmsCnt=");
      SerialMon.print(vrmsCnt, 1);
      SerialMon.print(" vmean=");
      SerialMon.print(vmeanI, 3);
      SerialMon.print(" vrmsV=");
      SerialMon.print(vrmsI, 4);
      SerialMon.print(" irms=");
      SerialMon.print(irms, 3);
      SerialMon.print(" offset=");
      SerialMon.print(adcOffset);
      SerialMon.print(" noiseA=");
      SerialMon.print(noiseFloorA, 3);
      SerialMon.print(" clipped=");
      SerialMon.println(clipped ? "yes" : "no");
    }
  }

  float vrmsV = 0, vmeanV = 0;
  if (sampleRms(VOLT_PIN, vrmsV, vmeanV)) {
    float volts = vrmsV * voltageScale;
    if (!isfinite(volts) || volts < 1.0f) volts = 0.0f;
    lastVolts = volts;
    lastVoltAdc = vrmsV;
    haveVoltage = true;
  }

  // Publica GPS cada gpsIntervalMs
  unsigned long nowGps = millis();
  if (mqtt.connected() && nowGps - lastGpsPublish >= gpsIntervalMs) {
    lastGpsPublish = nowGps;
    float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
    int vsat = 0, usat = 0;
    int year, month, day, hour, min, sec;
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &min, &sec)) {
      // Valida que el fix sea razonable
      if (lat != 0.0f && lon != 0.0f) {
        char payload[128];
        snprintf(payload, sizeof(payload),
                 "{\"lat\":%.6f,\"lng\":%.6f,\"alt\":%.1f,\"spd\":%.1f,\"acc\":%.1f,\"vsat\":%d,\"usat\":%d}",
                 lat, lon, alt, speed, accuracy, vsat, usat);
        if (mqtt.publish(mqttGpsTopic, payload, true)) {
          SerialMon.print("MQTT gps -> ");
          SerialMon.println(payload);
        } else {
          SerialMon.println("MQTT publish gps falló");
        }
      } else {
        SerialMon.println("Sin fix GNSS (lat/lon = 0)");
      }
    } else {
      SerialMon.println("getGPS falló o sin datos, probando LBS...");
      float accLbs = 0;
      if (getLbsLocation(lat, lon, accLbs)) {
        char payload[96];
        snprintf(payload, sizeof(payload),
                 "{\"lat\":%.6f,\"lng\":%.6f,\"acc\":%.1f,\"src\":\"lbs\"}", lat, lon, accLbs);
        if (mqtt.publish(mqttGpsTopic, payload, true)) {
          SerialMon.print("MQTT gps (LBS) -> ");
          SerialMon.println(payload);
        }
      } else {
        SerialMon.println("LBS sin datos");
      }
    }
  }
}

// Medición inicial para llenar lastIrms/lastVolts sin esperar al primer ciclo
void measureOnce() {
  float vrms = 0, vmean = 0;
  if (sampleRms(CURRENT_PIN, vrms, vmean)) {
    float irms = vrms / 0.01f;
    if (irms < 2.0f) irms = 0.0f;
    lastIrms = irms;
    haveCurrent = true;
  }
  if (sampleRms(VOLT_PIN, vrms, vmean)) {
    float volts = vrms * voltageScale;
    if (!isfinite(volts) || volts < 1.0f) volts = 0.0f;
    lastVolts = volts;
    haveVoltage = true;
  }
}

void applyRelayOutput() {
  digitalWrite(RELAY_PIN, relayOn ? (RELAY_ACTIVE_LOW ? LOW : HIGH)
                                  : (RELAY_ACTIVE_LOW ? HIGH : LOW));
}

void setMessage(const char* line1, const char* line2, unsigned long durationMs = 1800) {
  if (!lcdReady) return;
  strncpy(msgLine1, line1, 16);
  msgLine1[16] = '\0';
  strncpy(msgLine2, line2, 16);
  msgLine2[16] = '\0';
  messageUntil = millis() + durationMs;
  // Se pintará en updateLcd sin limpiar en cada llamada
}

void enterRunning() {
  sysState = STATE_RUNNING;
  relayOn = true;
  workStartMs = millis();
}

void enterRecovery() {
  sysState = STATE_RECOVERY;
  relayOn = false;
  recStartMs = millis();
  workedBeforeOffMin = workStartMs ? (float)(millis() - workStartMs) / 60000.0f : 0.0f;
  lastFaultI = lastIrms; // baja corriente
}

void enterFault() {
  sysState = STATE_FAULT;
  relayOn = false;
  lastFaultI = lastIrms; // sobrecorriente
}

File openFileSafe(const char* path, const char* mode) {
  (void)path; (void)mode;
  return File();
}

bool removeSafe(const char* path) {
  (void)path;
  return false;
}

// Apertura/selección de archivo de log (A, B o C)
bool openLogFile(int idx, bool truncate) {
  (void)idx; (void)truncate; return false;
}

// Log simple en CSV con rotación por tamaño
bool logSample(float irms, float volts) { (void)irms; (void)volts; return false; }
void rotateLogIfNeeded() {}
bool getLastLogs(int lines, String& out) { (void)lines; out = ""; return false; }

// Epoch actual; si no hay hora real, usa millis/1000
unsigned long getTimestamp() {
  time_t now = time(nullptr);
  if (now > 100000) return (unsigned long)now;
  return millis() / 1000UL;
}

const char* estadoToStr(SysState st) {
  if (manualOffState) return "apagado_app";
  if (manualOverride && !manualRelay) return "apagado_remoto";
  switch (st) {
    case STATE_RUNNING:   return "bombeando";
    case STATE_RECOVERY:  return "recuperando";
    case STATE_FAULT:     return "Sobre_corriente";
    case STATE_VOLT_WAIT: return voltFaultHigh ? "Sobre_voltaje" : "Bajo_voltaje";
    case STATE_STARTUP:
    default:              return "inicio";
  }
}

bool mqttPublish(const char* topic, const char* payload, bool retain) {
  if (!mqtt.connected()) return false;
  bool ok = mqtt.publish(topic, payload, retain);
  if (!ok) {
    SerialMon.print("MQTT publish fallo (");
    SerialMon.print(topic);
    SerialMon.print(") len=");
    SerialMon.print(strlen(payload));
    SerialMon.print(" state=");
    SerialMon.println(mqtt.state());
  }
  return ok;
}

void publishStatus() {
  if (!mqtt.connected()) return;
  const char* est = estadoToStr(sysState);
  float tTrabajoMin = 0.0f;
  if (sysState == STATE_RUNNING && workStartMs) {
    tTrabajoMin = (float)(millis() - workStartMs) / 60000.0f;
  } else {
    tTrabajoMin = workedBeforeOffMin;
  }
  float tRecMin = (sysState == STATE_RECOVERY && recStartMs) ? (float)(millis() - recStartMs) / 60000.0f : 0.0f;
  float tRecTotal = (float)tiempoRecuperacionMs / 60000.0f;
  float corrFalla = lastFaultI;
  float voltFalla = lastFaultV;
  unsigned long ts = getTimestamp();

  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"ts\":%lu,\"corriente\":%.2f,\"voltaje\":%.1f,\"estado\":\"%s\","
           "\"t_trabajo_min\":%.1f,\"t_recuperando_min\":%.1f,\"t_rec_total_min\":%.1f,"
           "\"corriente_falla\":%.2f,\"voltaje_falla\":%.1f,\"relay_on\":%s}",
           ts, lastIrms, lastVolts, est, tTrabajoMin, tRecMin, tRecTotal, corrFalla, voltFalla,
           (relayOn ? "true" : "false"));

  if (mqttPublish(mqttStateTopic, payload, false)) {
    SerialMon.print("MQTT estado -> ");
    SerialMon.println(payload);
  }

  char scalar[32];
  snprintf(scalar, sizeof(scalar), "%.2f", lastIrms);
  mqttPublish(mqttCurrentTopic, scalar, false);
  snprintf(scalar, sizeof(scalar), "%.1f", lastVolts);
  mqttPublish(mqttVoltageTopic, scalar, false);
  snprintf(scalar, sizeof(scalar), "%.4f", lastVoltAdc);
  mqttPublish(mqttAdcVTopic, scalar, false);
}

void publishConfig() {
  if (!mqtt.connected()) return;
  char payload[256];
  const float recMin = (float)tiempoRecuperacionMs / 60000.0f;
  snprintf(payload, sizeof(payload),
           "{\"cfg\":1,\"setpointA\":%.2f,\"setpointV\":%.1f,\"overPct\":%.2f,"
           "\"underPct\":%.2f,\"voltTol\":%.2f,\"recMin\":%.1f}",
           setpointA, setpointV, rangoSobre, rangoBaja, rangoVolt, recMin);
  mqttPublish(mqttStateTopic, payload, false);
}
void evaluateProtection() {
  const float limiteBajo = setpointA * (1.0f - rangoBaja);
  const float limiteAlto = setpointA * (1.0f + rangoSobre);
  const float voltMin = setpointV * (1.0f - rangoVolt);
  const float voltMax = setpointV * (1.0f + rangoVolt);
  const unsigned long graceMs = 10000; // 10 s de gracia al entrar en RUNNING

  switch (sysState) {
    case STATE_STARTUP:
      // Se mantiene hasta finalizar la secuencia de arranque en setup
      relayOn = true;
      break;
    case STATE_FAULT:
      relayOn = false;
      break;
    case STATE_VOLT_WAIT: {
      relayOn = false;
      if ((haveVoltage || haveCurrent) && (millis() - voltWaitStart) >= voltRetryMs) {
        const float voltMin = setpointV * (1.0f - rangoVolt);
        const float voltMax = setpointV * (1.0f + rangoVolt);
        if (lastVolts >= voltMin && lastVolts <= voltMax) {
          enterRunning();
        } else {
          // reinicia espera si sigue fuera
          voltWaitStart = millis();
        }
      }
      break;
    }
    case STATE_RECOVERY: {
      relayOn = false;
      unsigned long elapsed = millis() - recStartMs;
      if (elapsed >= tiempoRecuperacionMs) {
        enterRunning();
      }
      break;
    }
    case STATE_RUNNING:
    default:
      relayOn = true;
      if (haveCurrent || haveVoltage) {
        // Espera de gracia al arrancar
        if (workStartMs && (millis() - workStartMs < graceMs)) {
          break;
        }

        bool voltFuera = (lastVolts > voltMax) || (lastVolts < voltMin);
        if (voltFuera) {
          if (voltOutStart == 0) voltOutStart = millis();
          else if (millis() - voltOutStart >= 3000) {
            voltFaultHigh = lastVolts > voltMax;
            voltFaultVal = lastVolts;
            lastFaultV = lastVolts;
            voltWaitStart = millis();
            voltOutStart = 0;
            sysState = STATE_VOLT_WAIT;
            relayOn = false;
            char buf[17];
            snprintf(buf, sizeof(buf), "V=%.0f", voltFaultVal);
            setMessage(voltFaultHigh ? "Alto voltaje" : "Bajo voltaje", buf, 2000);
            return;
          }
        } else {
          voltOutStart = 0;
        }

        if (lastIrms > limiteAlto) {
          enterFault();
          setMessage("Detenido por", "sobrecorriente", 3500);
        } else if (lastIrms < limiteBajo) {
          enterRecovery();
          setMessage("Recuperando", "Nivel de Agua", 2000);
        }
      }
      break;
  }
}

void updateLcd() {
  if (!lcdReady) return;
  unsigned long now = millis();
  String l1, l2;

  if (messageUntil > now) {
    l1 = msgLine1;
    l2 = msgLine2;
  } else if (editState == EDIT_SETPOINT) {
    l1 = "Edite Corriente";
    l2 = "Valor: " + String((int)setpointA);
  } else if (editState == EDIT_RECUP) {
    l1 = "Edite T Recup...";
    l2 = "Valor: " + String(tiempoRecuperacionMs / 60000UL);
  } else if (manualOverride && !manualRelay) {
    l1 = "APAGADO";
    l2 = "desde app web";
  } else {
    switch (sysState) {
      case STATE_STARTUP:
        l1 = "INICIANDO...";
        l2 = "";
        break;
      case STATE_FAULT:
        l1 = "Detenido por";
        l2 = "sobrecorriente";
        break;
      case STATE_VOLT_WAIT: {
        unsigned long restante = 0;
        if (voltWaitStart > 0 && (millis() - voltWaitStart) < voltRetryMs) {
          restante = (voltRetryMs - (millis() - voltWaitStart)) / 1000UL;
        }
        l1 = voltFaultHigh ? "Alto voltaje" : "Bajo voltaje";
        l2 = "V=" + String((int)voltFaultVal) + " tta=" + String((int)(restante / 60));
        break;
      }
      case STATE_RECOVERY: {
        unsigned long recElapsedMin = (millis() - recStartMs) / 60000UL;
        l1 = "Recup.. tta=" + String((int)workedBeforeOffMin);
        l2 = "Cteo=" + String((int)recElapsedMin) + " time=" + String((int)(tiempoRecuperacionMs / 60000UL));
        break;
      }
      case STATE_RUNNING:
      default: {
        unsigned long minutos = workStartMs ? (millis() - workStartMs) / 60000UL : 0;
        l1 = "Bombeando T=" + String((int)minutos);
        // Formato compacto: "10.0A 120V SP=10" (16 chars máx)
        l2 = String(lastIrms, 1) + "A " + String((int)lastVolts) + "V SP=" + String((int)setpointA);
        break;
      }
    }
  }

  auto writeLine = [&](int row, const String& txt) {
    lcd.setCursor(0, row);
    lcd.print("                ");
    lcd.setCursor(0, row);
    lcd.print(txt.substring(0, 16));
  };

  bool changed = (l1 != lastL1) || (l2 != lastL2);
  bool timeOk = (now - lastLcdUpdate) >= lcdRefreshMs || (messageUntil > now);
  if (changed && timeOk) {
    writeLine(0, l1);
    writeLine(1, l2);
    lastL1 = l1;
    lastL2 = l2;
    lastLcdUpdate = now;
  }
}

void handleButton() {
  const unsigned long now = millis();
  const bool raw = digitalRead(BUTTON_PIN);
  rawButtonState = raw;

  // Debounce robusto: actualiza el temporizador solo cuando cambia la lectura cruda
  if (raw != lastRawButtonState) {
    lastRawButtonState = raw;
    lastDebounceMs = now;
    return;
  }

  // Espera a que la se񣢠al esté estable
  if ((now - lastDebounceMs) < debounceIntervalMs) {
    return;
  }

  if (raw == debouncedState) {
    return;
  }

  prevDebouncedState = debouncedState;
  debouncedState = raw;

  const bool pressedEdge = BUTTON_ACTIVE_LOW
                             ? (prevDebouncedState == HIGH && debouncedState == LOW)
                             : (prevDebouncedState == LOW && debouncedState == HIGH);
  const bool releasedEdge = BUTTON_ACTIVE_LOW
                              ? (prevDebouncedState == LOW && debouncedState == HIGH)
                              : (prevDebouncedState == HIGH && debouncedState == LOW);

  if (pressedEdge) {
    buttonPressStart = now;
    return;
  }

  if (!releasedEdge) {
    return;
  }

  const unsigned long dur = now - buttonPressStart;

  if (editState == EDIT_NONE) {
    if (dur >= longPressMs) {
      editState = EDIT_SETPOINT;
      lastIncrementMs = 0;
    }
    return;
  }

  if (editState == EDIT_SETPOINT) {
    if (dur >= savePressMs) {
      EEPROM.put(EEPROM_ADDR_SETPOINT, setpointA);
      EEPROM.commit();
      char buf[17];
      snprintf(buf, sizeof(buf), "Registrado:%d", (int)setpointA);
      setMessage("Corriente:", buf, 1800);
      editState = EDIT_RECUP;
      lastIncrementMs = 0;
    } else {
      if (now - lastIncrementMs > 250) {
        setpointA += 1.0f;
        if (setpointA > 30.0f) setpointA = 1.0f;
        lastIncrementMs = now;
      }
    }
    return;
  }

  if (editState == EDIT_RECUP) {
    if (dur >= savePressMs) {
      uint16_t recMin = (uint16_t)(tiempoRecuperacionMs / 60000UL);
      EEPROM.put(EEPROM_ADDR_RECUP_MIN, recMin);
      EEPROM.commit();
      char buf[17];
      snprintf(buf, sizeof(buf), "Registrado:%u", (unsigned int)recMin);
      setMessage("T de Recup:", buf, 1800);
      editState = EDIT_NONE;
      lastIncrementMs = 0;
    } else {
      if (now - lastIncrementMs > 250) {
        unsigned long recMin = tiempoRecuperacionMs / 60000UL + 1;
        if (recMin > 90) recMin = 1;
        tiempoRecuperacionMs = recMin * 60UL * 1000UL;
        lastIncrementMs = now;
      }
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(CURRENT_PIN, INPUT);
  pinMode(VOLT_PIN, INPUT);

  SerialMon.begin(115200);
  delay(500);
  SerialMon.println("Boot ESP32 + SIM7000G (corriente SCT013 -> MQTT)");
  SerialMon.print("Reset reason: ");
  SerialMon.println(resetReasonStr(esp_reset_reason()));

  Wire.begin(I2C_SDA, I2C_SCL);
  scanI2C();
  lcd.init();
  lcd.begin(16, 2);
  delay(50);
  lcd.backlight();
  lcd.clear();
  lcd.home();
  lcd.noCursor();
  lcd.noBlink();
  lcdReady = true;

  // Muestra la causa de reinicio por 2s al arrancar
  lcd.setCursor(0, 0);
  lcd.print("Reset:");
  lcd.setCursor(0, 1);
  lcd.print(resetReasonStr(esp_reset_reason()));
  delay(2000);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW); // inicia apagado
  pinMode(BUTTON_PIN, BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  rawButtonState = digitalRead(BUTTON_PIN);
  lastRawButtonState = rawButtonState;
  debouncedState = rawButtonState;
  prevDebouncedState = rawButtonState;
  lastDebounceMs = millis();

  EEPROM.begin(128);
  // Cargar setpoint y tiempo desde EEPROM si son válidos
  float eSet = 0;
  uint16_t eRecMin = 0;
  float eSetV = 0, eOver = 0, eUnder = 0, eVoltTol = 0;
  EEPROM.get(EEPROM_ADDR_SETPOINT, eSet);
  EEPROM.get(EEPROM_ADDR_RECUP_MIN, eRecMin);
  EEPROM.get(EEPROM_ADDR_SETPOINT_V, eSetV);
  EEPROM.get(EEPROM_ADDR_OVER_PCT, eOver);
  EEPROM.get(EEPROM_ADDR_UNDER_PCT, eUnder);
  EEPROM.get(EEPROM_ADDR_VOLT_TOL, eVoltTol);
  if (isfinite(eSet) && eSet >= 1.0f && eSet <= 30.0f) {
    setpointA = eSet;
  }
  if (eRecMin >= 1 && eRecMin <= 1440) {
    tiempoRecuperacionMs = (unsigned long)eRecMin * 60UL * 1000UL;
  }
  if (isfinite(eSetV) && eSetV >= 50.0f && eSetV <= 260.0f) {
    setpointV = eSetV;
  }
  if (isfinite(eOver) && eOver > 0 && eOver < 1) {
    rangoSobre = eOver;
  }
  if (isfinite(eUnder) && eUnder > 0 && eUnder < 1) {
    rangoBaja = eUnder;
  }
  if (isfinite(eVoltTol) && eVoltTol > 0 && eVoltTol < 1) {
    rangoVolt = eVoltTol;
  }

  powerOnModem();
  delay(2000);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(500);

  SerialMon.println("Inicializando módem...");
  if (!modem.init()) {
    SerialMon.println("Fallo modem.init()");
    return;
  }
  modem.enableGPS();

  if (!connectGprs()) {
    SerialMon.println("No hay GPRS, abortando");
    return;
  }

  doMqtt();

  // Calibración de offset en vacío
  SerialMon.println("Calibrando offset ADC...");
  const int calSamples = 4000;
  uint64_t sum = 0;
  uint64_t sumsq = 0;
  for (int i = 0; i < calSamples; i++) {
    int raw = analogRead(CURRENT_PIN);
    sum += raw;
    sumsq += (uint64_t)raw * (uint64_t)raw;
    delay(1);
  }
  adcOffset = sum / calSamples;
  float mean = (float)adcOffset;
  float variance = (float)sumsq / calSamples - mean * mean;
  float stddev = variance > 0 ? sqrtf(variance) : 0.0f;
  // Convierte stddev ADC a amperios: std * (3.3/4095) / 0.01
  float stdA = stddev * (3.3f / 4095.0f) / 0.01f;
  // Umbral base informativo (no se usa para recorte)
  noiseFloorA = fmaxf(0.01f, stdA);

  SerialMon.print("Offset ADC: ");
  SerialMon.print(adcOffset);
  SerialMon.print(" | stddev ADC: ");
  SerialMon.print(stddev, 2);
  SerialMon.print(" | noiseFloorA: ");
  SerialMon.println(noiseFloorA, 2);

  // Secuencia de arranque de 5 s
  sysState = STATE_STARTUP;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("INICIANDO...");
  delay(5000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SetPoint: ");
  lcd.print((int)setpointA);
  delay(1500);

  // Medición inicial para mostrar valores desde el arranque
  measureOnce();

  enterRunning();
  applyRelayOutput();
  updateLcd();
}

void loop() {
  // Atiende el botón antes de entrar en tareas que bloquean un poco
  handleButton();

  if (mqtt.connected()) {
    mqtt.loop();
  } else {
    doMqtt();
  }

  unsigned long now = millis();
  if (now - lastStatusPublish >= statusIntervalMs) {
    lastStatusPublish = now;
    measureCurrentAndVoltage();
    publishStatus();
  }
  evaluateProtection();
  // Forzar apagado en estados distintos a RUNNING
  if (sysState != STATE_RUNNING) {
    relayOn = false;
  }
  // Comando manual: solo enciende si estamos en RUNNING; apagar siempre se permite
  if (manualOverride) {
    if (manualRelay && sysState == STATE_RUNNING) {
      relayOn = true;
    } else if (!manualRelay) {
      relayOn = false;
    }
  }
  applyRelayOutput();
  updateLcd();

  // Parpadeo de vida sin bloquear
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (now - lastBlink >= 500) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}
