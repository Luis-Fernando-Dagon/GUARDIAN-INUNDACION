/*
  GUARDIAN DE INUNDACION COMUNITARIO - Nodo (version 3: ESP32 o ESP8266)
  ---------------------------------------------------------------------
  El mismo archivo sirve para las dos placas. Se elige en Arduino IDE:
    - ESP32   -> Herramientas > Placa > "ESP32 Dev Module"
    - ESP8266 -> Herramientas > Placa > "NodeMCU 1.0 (ESP-12E Module)"
  La carpeta del sketch tiene que llamarse igual que el archivo: guardian_nodo.

  Sensores  : JSN-SR04T (nivel de agua, ultrasonico) + sensor de agua en el piso (pistas rojas)
  Actuadores: buzzer pasivo (tonos por nivel), semaforo LED
  Nube      : ThingSpeak (8 campos) con millis(), sin delay() largos

  DIFERENCIAS ENTRE PLACAS
    ESP32  : 3 botones de respuesta (bien / asistencia / evacuar). Recalibrar: pulsacion larga de SILENCIAR
             (o mantenerlo apretado al encender).
    ESP8266: faltan pines, asi que hay UN solo boton de respuesta: 1 pulsacion = bien, 2 = asistencia,
             3 = evacuar (se cuentan dentro de 2 segundos). Recalibrar: mantener SILENCIAR 5 s (solo si
             no hay agua). NO se puede mantener apretado al encender: GPIO0 en LOW pone la placa en
             modo de programacion.

  CAMPOS DEL CANAL (field2 se llama agua_piso_pct)
    field1 = nivel_cm            (agua sobre el piso, calculado contra la distancia en seco)
    field2 = agua_piso_pct       (0 a 100, sensor de agua en el piso)
    field3 = velocidad_cm_min    (subida del agua, cm por minuto)
    field4 = estado              (0 seguro, 1 aviso 5 cm, 2 alerta 15 cm, 3 peligro 30 cm, 4 evacuacion 60 cm)
    field5 = respuesta           (0 sin respuesta, 1 presente/bien, 2 necesito asistencia, 3 quiero evacuar)
    field6 = dist_cruda_cm       (para depurar)
    field7 = baseline_cm         (distancia en seco)
    field8 = modo_noche          (0 dia, 1 noche)

  CONEXIONES ESP32 (DevKit de 30/38 pines)
    JSN-SR04T : VCC al pin VIN (5V), GND, TRIG -> GPIO5, ECHO -> GPIO18 PASANDO POR DIVISOR 1k/2k
                (Echo ----[1k]----+---- GPIO18 ;  +----[2k]---- GND)
    Agua piso : S -> GPIO34, + -> GPIO13 (alimenta solo al medir), - -> GND
    Buzzer    : S -> GPIO25, - -> GND, y el pin + (si el modulo lo trae) a 3V3
    Semaforo  : verde GPIO26, amarillo GPIO27, rojo GPIO14, GND comun
    Botones   : un lado al pin y el otro a GND (INPUT_PULLUP): silenciar 32, bien 33, asistencia 19, evacuar 23

  CONEXIONES ESP8266 (NodeMCU: pin impreso -> GPIO)
    JSN-SR04T : VCC al pin VIN (5V con USB), GND, TRIG -> D1 (GPIO5), ECHO -> D2 (GPIO4) POR DIVISOR 1k/2k
    Agua piso : S -> A0, + -> D8 (GPIO15, alimenta solo al medir), - -> GND
    Buzzer    : S -> D5 (GPIO14), - -> GND, y el pin + (si el modulo lo trae) a 3V3
    Semaforo  : verde D6 (GPIO12), amarillo D7 (GPIO13), rojo D0 (GPIO16), GND comun
    Botones   : un lado al pin y el otro a GND: SILENCIAR en D3 (GPIO0), RESPUESTA en D4 (GPIO2)
                No los mantengas apretados al enchufar o reiniciar la placa (D3 y D4 son pines de arranque).

  NOTA: codigo SIN PROBAR en hardware. Hay que calibrar AGUA_SECO_RAW / AGUA_MOJADO_RAW y verificar los pines.
  Ajustar los umbrales con datos reales (prueba con un balde).
  Probado solo con una comprobacion de sintaxis. Cores: Arduino-ESP32 2.x/3.x y ESP8266 Community 3.x.
*/

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClient.h>
  #include <EEPROM.h>
#else
  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <Preferences.h>
#endif
#include <time.h>

// ===================== CONFIGURACION =====================
const char* WIFI_SSID    = "TU_WIFI";
const char* WIFI_PASS    = "TU_CLAVE";
const char* TS_WRITE_KEY = "TU_WRITE_API_KEY";     // NO la subas a GitHub ni la compartas

// TalkBack opcional para comandos desde la web (SILENCIAR / PROBAR). Dejar "" para desactivar.
const char* TB_ID  = "";
const char* TB_KEY = "";

// ===================== PINES =====================
#if defined(ESP8266)
const int PIN_TRIG = 5;        // D1
const int PIN_ECHO = 4;        // D2  OJO: con divisor de tension 1k/2k (el sensor entrega 5V)
const int PIN_AGUA = A0;       // unica entrada analogica
const int PIN_AGUA_VCC = 15;   // D8, alimenta el sensor de agua solo al medir
const int PIN_BUZZER = 14;     // D5
const int PIN_LED_V = 12;      // D6
const int PIN_LED_A = 13;      // D7
const int PIN_LED_R = 16;      // D0
const int PIN_BTN_SILENCIO = 0; // D3
const int PIN_BTN_RESP = 2;    // D4: 1, 2 o 3 pulsaciones
#else
const int PIN_TRIG = 5;
const int PIN_ECHO = 18;       // OJO: con divisor de tension 1k/2k (el sensor entrega 5V)
const int PIN_AGUA = 34;       // entrada analogica (ADC1)
const int PIN_AGUA_VCC = 13;   // alimenta el sensor de agua solo al medir
const int PIN_BUZZER = 25;
const int PIN_LED_V = 26;
const int PIN_LED_A = 27;
const int PIN_LED_R = 14;
const int PIN_BTN_SILENCIO = 32;
const int PIN_BTN_BIEN = 33;
const int PIN_BTN_ASIST = 19;
const int PIN_BTN_EVAC = 23;
#endif

// ===================== UMBRALES Y TIEMPOS =====================
const float UMBRALES_CM[4] = {5.0, 15.0, 30.0, 60.0};  // aviso, alerta, peligro, evacuacion
const float SUBIDA_RAPIDA_CM_MIN = 3.0;                // si sube mas rapido, el estado sube un escalon
const int   NOCHE_DESDE = 22;                          // hora local
const int   NOCHE_HASTA = 7;
const long  GMT_OFFSET_SEG = -3 * 3600;                // Argentina UTC-3
const int   ESTADO_MIN_RESPUESTA = 2;                  // los botones de respuesta se habilitan desde la alerta

const unsigned long MEDIR_CADA_MS      = 5000;
const unsigned long ENVIAR_CADA_MS     = 20000;        // ThingSpeak gratis: minimo 15 s
const unsigned long ENVIAR_ALERTA_MS   = 15000;
const unsigned long TALKBACK_CADA_MS   = 30000;
const unsigned long VENTANA_VEL_MS     = 60000;
const unsigned long SILENCIO_MS        = 10UL * 60UL * 1000UL;
const unsigned long PRUEBA_MS          = 5000;
const unsigned long GUARDAR_CADA_MS    = 30UL * 60UL * 1000UL;   // cuidar la memoria flash
const unsigned long RECAL_LARGA_MS     = 5000;         // pulsacion larga de SILENCIAR = recalibrar en seco
const unsigned long VENTANA_PULSOS_MS  = 2000;         // ESP8266: tiempo para contar 1, 2 o 3 pulsaciones

const int SUBE_CONFIRMA = 3;   // lecturas seguidas para subir de estado
const int BAJA_CONFIRMA = 6;   // lecturas seguidas para bajar de estado

// Calibracion del sensor de agua en el piso. Medir en seco y con agua (monitor serie, 115200).
// Este sensor da MAS valor cuanto mas mojado esta (al reves que el FC-37).
// El ESP32 lee de 0 a 4095 y el ESP8266 de 0 a 1023.
const int AGUA_SECO_RAW = 0;
#if defined(ESP8266)
const int AGUA_MOJADO_RAW = 500;
#else
const int AGUA_MOJADO_RAW = 2000;
#endif

// ===================== ESTADO GLOBAL =====================
#if !defined(ESP8266)
Preferences prefs;
#endif
float baselineCm = -1;       // distancia en seco
float baselineGuardada = -1;
float distCruda = 0;
float nivelCm = 0;
float nivelPrevio = 0;
float velocidad = 0;
int   aguaPisoPct = 0;
int   estado = 0;
int   respuesta = 0;
bool  noche = false;
int   horaActual = -1;

unsigned long silenciadoHasta = 0;
unsigned long probarHasta = 0;
unsigned long tMedir = 0, tEnviar = 0, tVel = 0, tTalk = 0, tBtn = 0, tGuardar = 0;

// ===================== MEMORIA (distancia en seco) =====================
#if defined(ESP8266)
struct Guardado { uint32_t magic; float base; };
const uint32_t MAGIC = 0xA5A55A5AUL;
#endif

void almacenamientoInit() {
#if defined(ESP8266)
  EEPROM.begin(16);
#else
  prefs.begin("guardian", false);
#endif
}

float leerBaselineGuardada() {
#if defined(ESP8266)
  Guardado g;
  EEPROM.get(0, g);
  if (g.magic == MAGIC && g.base > 0 && g.base < 600) return g.base;
  return -1;
#else
  return prefs.getFloat("base", -1);
#endif
}

void escribirBaseline(float b) {
#if defined(ESP8266)
  Guardado g;
  g.magic = MAGIC;
  g.base = b;
  EEPROM.put(0, g);
  EEPROM.commit();
#else
  prefs.putFloat("base", b);
#endif
}

void guardarBaseline() {
  if (baselineCm <= 0) return;
  escribirBaseline(baselineCm);
  baselineGuardada = baselineCm;
  tGuardar = millis();
}

// ===================== BUZZER =====================
void buzzerInit() {
#if defined(ESP8266)
  pinMode(PIN_BUZZER, OUTPUT);
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_BUZZER, 2000, 10);
#else
  ledcSetup(0, 2000, 10);
  ledcAttachPin(PIN_BUZZER, 0);
#endif
}

void sonar(int freq) {
  static int ultima = -1;
  if (freq == ultima) return;          // no reiniciar el tono si es el mismo
  ultima = freq;
#if defined(ESP8266)
  if (freq <= 0) noTone(PIN_BUZZER); else tone(PIN_BUZZER, freq);
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(PIN_BUZZER, freq);
#else
  ledcWriteTone(0, freq);
#endif
}

// Patrones por nivel efectivo: frecuencia 1, frecuencia 2, ms encendido, ms apagado
struct Patron { int f1; int f2; unsigned long on; unsigned long off; };
const Patron PATRONES[5] = {
  {0, 0, 0, 0},
  {500, 500, 150, 3000},     // 1 pitido suave
  {800, 800, 250, 1500},     // 2 sonido medio
  {600, 1000, 400, 400},     // 3 sirena
  {1000, 1800, 150, 150}     // 4 sirena rapida (evacuacion)
};

bool estaSilenciada() { return (long)(silenciadoHasta - millis()) > 0; }

void actualizarAlarma(int nivelEfectivo, bool ignorarSilencio) {
  static unsigned long t0 = 0;
  static bool faseOn = false;
  if (nivelEfectivo <= 0 || (!ignorarSilencio && estaSilenciada())) {
    sonar(0);
    faseOn = false;
    return;
  }
  const Patron& p = PATRONES[nivelEfectivo];
  unsigned long ahora = millis();
  unsigned long dur = faseOn ? p.on : p.off;
  if (ahora - t0 >= dur) { faseOn = !faseOn; t0 = ahora; }
  if (faseOn) {
    int f = ((ahora / 120) % 2 == 0) ? p.f1 : p.f2;   // alterna tonos: efecto sirena
    sonar(f);
  } else {
    sonar(0);
  }
}

// ===================== SENSORES =====================
float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);   // timeout ~5 m
  if (dur == 0) return -1;
  return dur * 0.0343f / 2.0f;
}

float distanciaMediana() {
  float v[7];
  int n = 0;
  for (int i = 0; i < 7; i++) {
    float d = medirDistanciaCm();
    if (d > 0) v[n++] = d;
    delay(60);
  }
  if (n < 3) return -1;
  for (int i = 1; i < n; i++) {                            // orden por insercion
    float k = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  return v[n / 2];
}

int leerAguaPisoPct() {
  digitalWrite(PIN_AGUA_VCC, HIGH);                        // se enciende solo para medir
  delay(30);
  long suma = 0;
  for (int i = 0; i < 4; i++) { suma += analogRead(PIN_AGUA); delay(5); }
  digitalWrite(PIN_AGUA_VCC, LOW);
  int raw = suma / 4;
  return constrain(map(raw, AGUA_SECO_RAW, AGUA_MOJADO_RAW, 0, 100), 0, 100);
}

// ===================== LOGICA DE ESTADO =====================
int estadoPorNivel(float nivel, float vel) {
  int e = 0;
  for (int i = 0; i < 4; i++) if (nivel >= UMBRALES_CM[i]) e = i + 1;
  if (e >= 1 && e < 4 && vel >= SUBIDA_RAPIDA_CM_MIN) e++;   // sube rapido: un escalon mas
  return e;
}

void actualizarEstado(int candidato) {
  static int cuentaSube = 0, cuentaBaja = 0;
  int previo = estado;
  if (candidato > estado) {
    cuentaSube++; cuentaBaja = 0;
    if (cuentaSube >= SUBE_CONFIRMA) { estado = candidato; cuentaSube = 0; }
  } else if (candidato < estado) {
    cuentaBaja++; cuentaSube = 0;
    if (cuentaBaja >= BAJA_CONFIRMA) { estado = estado - 1; cuentaBaja = 0; }   // baja de a un escalon
  } else {
    cuentaSube = 0; cuentaBaja = 0;
  }
  if (estado > previo) silenciadoHasta = 0;   // si empeora, se vuelve a activar la alarma
  if (estado == 0) respuesta = 0;             // se borra la respuesta al terminar el temporal
}

bool horaLocal(struct tm* t) {
#if defined(ESP8266)
  time_t ahora = time(nullptr);
  if (ahora < 1700000000L) return false;      // el reloj todavia no se sincronizo
  localtime_r(&ahora, t);
  return true;
#else
  return getLocalTime(t, 10);
#endif
}

bool esDeNoche() {
  struct tm t;
  if (!horaLocal(&t)) { horaActual = -1; return false; }
  int h = t.tm_hour;
  horaActual = h;
  return (h >= NOCHE_DESDE || h < NOCHE_HASTA);
}

void medir() {
  float d = distanciaMediana();
  aguaPisoPct = leerAguaPisoPct();
  noche = esDeNoche();

  if (d < 0) { Serial.println("Sin eco del ultrasonico"); return; }
  distCruda = d;
  if (baselineCm < 0) { baselineCm = d; guardarBaseline(); }

  nivelCm = baselineCm - d;
  if (nivelCm < 0) nivelCm = 0;

  if (millis() - tVel >= VENTANA_VEL_MS) {
    velocidad = nivelCm - nivelPrevio;                     // cm por minuto
    nivelPrevio = nivelCm;
    tVel = millis();
  }

  // Recalibracion lenta del "seco" para compensar el calor, solo si no hay agua en el piso
  if (estado == 0 && aguaPisoPct < 10 && nivelCm < 1.5) {
    baselineCm = 0.98f * baselineCm + 0.02f * d;
    if (millis() - tGuardar > GUARDAR_CADA_MS && fabsf(baselineCm - baselineGuardada) > 0.5f) guardarBaseline();
  }

  actualizarEstado(estadoPorNivel(nivelCm, velocidad));

  Serial.printf("hora=%d dist=%.1f base=%.1f nivel=%.1f vel=%.1f aguaPiso=%d%% estado=%d noche=%d\n",
                horaActual, distCruda, baselineCm, nivelCm, velocidad, aguaPisoPct, estado, noche);
}

// ===================== BOTONES =====================
void registrarRespuesta(int valor) {
  // Solo se registra durante el temporal, desde la alerta (estado >= 2).
  // Antes de eso el boton no hace nada, asi nadie puede saber si hay alguien en la casa.
  if (estado >= ESTADO_MIN_RESPUESTA) respuesta = valor;
}

void recalibrarEnSeco() {
  if (estado != 0 || aguaPisoPct >= 10) {
    Serial.println("Recalibracion rechazada: hay agua en el piso");
    return;
  }
  float d = distanciaMediana();
  if (d > 0) {
    baselineCm = d;
    guardarBaseline();
    Serial.printf("Recalibrado en seco: %.1f cm\n", baselineCm);
  }
}

void leerBotones() {
  unsigned long ahora = millis();

  // ---- SILENCIAR (en las dos placas) ----
  static bool silPrev = false, recalHecho = false;
  static unsigned long tSilPress = 0, tSilDeb = 0;
  bool sil = (digitalRead(PIN_BTN_SILENCIO) == LOW);
  if (sil && !silPrev) {
    tSilPress = ahora;
    recalHecho = false;
    if (ahora - tSilDeb > 200) {
      tSilDeb = ahora;
      silenciadoHasta = ahora + SILENCIO_MS;
      registrarRespuesta(1);                               // silenciar durante la alerta = presente
    }
  }
  if (sil && !recalHecho && ahora - tSilPress >= RECAL_LARGA_MS) {
    recalHecho = true;
    recalibrarEnSeco();
  }
  silPrev = sil;

#if defined(ESP8266)
  // ---- UN solo boton de respuesta: cuenta pulsaciones ----
  static bool respPrev = false;
  static int pulsos = 0;
  static unsigned long tUltimo = 0, tDeb = 0;
  bool resp = (digitalRead(PIN_BTN_RESP) == LOW);
  if (resp && !respPrev && ahora - tDeb > 200) {
    tDeb = ahora;
    pulsos++;
    tUltimo = ahora;
  }
  respPrev = resp;
  if (pulsos > 0 && ahora - tUltimo > VENTANA_PULSOS_MS) {
    registrarRespuesta(pulsos > 3 ? 3 : pulsos);           // 1 bien, 2 asistencia, 3 evacuar
    pulsos = 0;
  }
#else
  // ---- 3 botones de respuesta ----
  if (ahora - tBtn >= 300) {
    if (digitalRead(PIN_BTN_BIEN) == LOW) { registrarRespuesta(1); tBtn = ahora; }
    else if (digitalRead(PIN_BTN_ASIST) == LOW) { registrarRespuesta(2); tBtn = ahora; }
    else if (digitalRead(PIN_BTN_EVAC) == LOW) { registrarRespuesta(3); tBtn = ahora; }
  }
#endif
}

void actualizarLeds() {
  digitalWrite(PIN_LED_V, estado == 0);
  digitalWrite(PIN_LED_A, estado == 1 || estado == 2);
  digitalWrite(PIN_LED_R, estado >= 3);
}

// ===================== THINGSPEAK =====================
void enviarThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) return;
  String url = String("http://api.thingspeak.com/update?api_key=") + TS_WRITE_KEY;
  url += "&field1=" + String(nivelCm, 1);
  url += "&field2=" + String(aguaPisoPct);
  url += "&field3=" + String(velocidad, 1);
  url += "&field4=" + String(estado);
  url += "&field5=" + String(respuesta);
  url += "&field6=" + String(distCruda, 1);
  url += "&field7=" + String(baselineCm, 1);
  url += "&field8=" + String(noche ? 1 : 0);
  HTTPClient http;
#if defined(ESP8266)
  WiFiClient cliente;
  http.begin(cliente, url);
#else
  http.begin(url);
#endif
  int code = http.GET();
  Serial.printf("ThingSpeak HTTP %d\n", code);
  http.end();
}

void leerTalkBack() {
  if (strlen(TB_ID) == 0 || WiFi.status() != WL_CONNECTED) return;
  String url = String("http://api.thingspeak.com/talkbacks/") + TB_ID + "/commands/execute?api_key=" + TB_KEY;
  HTTPClient http;
#if defined(ESP8266)
  WiFiClient cliente;
  http.begin(cliente, url);
#else
  http.begin(url);
#endif
  int code = http.GET();
  if (code == 200) {
    String cmd = http.getString();
    cmd.trim();
    if (cmd == "SILENCIAR") silenciadoHasta = millis() + SILENCIO_MS;
    else if (cmd == "PROBAR") probarHasta = millis() + PRUEBA_MS;
  }
  http.end();
}

void mantenerWiFi() {
  static unsigned long tIntento = 0;
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - tIntento > 15000) {
    tIntento = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ===================== CALIBRACION AL ENCENDER =====================
void calibrarAlEncender() {
  almacenamientoInit();
  baselineGuardada = leerBaselineGuardada();
#if defined(ESP8266)
  bool forzar = false;   // GPIO0 en LOW al encender = modo programacion: en ESP8266 se recalibra con pulsacion larga
#else
  bool forzar = (digitalRead(PIN_BTN_SILENCIO) == LOW);    // boton SILENCIAR apretado al encender
#endif

  float acum[3];
  int ok = 0;
  for (int i = 0; i < 3; i++) {
    float d = distanciaMediana();
    if (d > 0) acum[ok++] = d;
  }
  float d0 = (ok > 0) ? acum[ok / 2] : -1;
  int agua = leerAguaPisoPct();
  bool hayAgua = (agua > 50) || (baselineGuardada > 0 && d0 > 0 && (baselineGuardada - d0) >= 3.0f);

  if (baselineGuardada > 0 && hayAgua && !forzar) {
    baselineCm = baselineGuardada;                          // reinicio con agua: NO recalibrar
    Serial.println("Reinicio con agua en el piso: uso la distancia en seco guardada");
  } else if (d0 > 0) {
    baselineCm = d0;                                        // instalacion en seco: calibrar y guardar
    guardarBaseline();
    Serial.println(forzar ? "Recalibracion forzada" : "Calibracion en seco guardada");
  } else {
    baselineCm = baselineGuardada;                          // sin eco y sin dato: queda en -1 si nunca se calibro
  }
  Serial.printf("Baseline inicial: %.1f cm\n", baselineCm);
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_AGUA_VCC, OUTPUT);
  digitalWrite(PIN_AGUA_VCC, LOW);
  pinMode(PIN_LED_V, OUTPUT);
  pinMode(PIN_LED_A, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BTN_SILENCIO, INPUT_PULLUP);
#if defined(ESP8266)
  pinMode(PIN_BTN_RESP, INPUT_PULLUP);
#else
  pinMode(PIN_BTN_BIEN, INPUT_PULLUP);
  pinMode(PIN_BTN_ASIST, INPUT_PULLUP);
  pinMode(PIN_BTN_EVAC, INPUT_PULLUP);
#endif
  buzzerInit();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTime(GMT_OFFSET_SEG, 0, "pool.ntp.org");

  calibrarAlEncender();
}

void loop() {
  unsigned long ahora = millis();

  leerBotones();

  if (ahora - tMedir >= MEDIR_CADA_MS) {
    tMedir = ahora;
    medir();
  }

  bool probando = (long)(probarHasta - millis()) > 0;
  int nivelEfectivo = estado;
  if (noche && estado >= 1) nivelEfectivo = min(estado + 1, 4);   // de noche suena un escalon mas fuerte
  if (probando) nivelEfectivo = 4;
  actualizarAlarma(nivelEfectivo, probando);
  actualizarLeds();

  unsigned long cadaEnvio = (estado >= 3) ? ENVIAR_ALERTA_MS : ENVIAR_CADA_MS;
  if (ahora - tEnviar >= cadaEnvio) {
    tEnviar = ahora;
    enviarThingSpeak();
  }

  if (ahora - tTalk >= TALKBACK_CADA_MS) {
    tTalk = ahora;
    leerTalkBack();
  }

  mantenerWiFi();
}
