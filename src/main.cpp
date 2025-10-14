#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Adafruit_NeoPixel.h>
#include <DNSServer.h>
#include <Update.h>
#include "esp_log.h"

// === OLED (U8g2) ===
#include <U8g2lib.h>
#include <SPI.h>

// ---------- Pines OLED (NO CAMBIAR) ----------
#define OLED_SCK  36   // D0 / SCL / CLK
#define OLED_MOSI 35   // D1 / SDA / MOSI
#define OLED_CS   37
#define OLED_DC   38
#define OLED_RST  39
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(
  U8G2_R0,
  /* cs=*/ OLED_CS,
  /* dc=*/ OLED_DC,
  /* reset=*/ OLED_RST
);

// ===== Versión (sólo Splash) =====
static const char* FW_VERSION = "v1.2.1";

/* ==============================================================
   NMEA Link (ESP32 / ESP32-S3)  —  AP + Menú + Monitor + Generator + OTA
   ============================================================== */

// ===== Verbose de arranque (0 = silencioso tras boot) =====
#define VERBOSE 0
#if VERBOSE
  #define DPRINTLN(x)   Serial.println(x)
  #define DPRINTF(...)  Serial.printf(__VA_ARGS__)
#else
  #define DPRINTLN(x)   do{}while(0)
  #define DPRINTF(...)  do{}while(0)
#endif

// ===== AP / Captive =====
const char* AP_SSID     = "NMEA_Link";
const char* AP_PASSWORD = "12345678";
const uint16_t DNS_PORT = 53;
DNSServer dnsServer;

// ===== LED =====
#define LED_PIN 48
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
unsigned long ledMillis = 0;
const int LED_DURATION = 50;
bool ledOn = false;

// ===== UART =====
HardwareSerial NMEA_Serial(1);
#define RX_PIN 16
#define TX_PIN 17
volatile int currentBaud = 4800;

// ===== UDP =====
WiFiUDP udp;
IPAddress udpAddress;
const int udpPort = 10110;

// ===== Web =====
WebServer server(80);

// ===== Buffers =====
#define BUFFER_LINES 50
String nmeaBuffer[BUFFER_LINES];
int bufferIndex = 0;
String currentLine = "";
unsigned long lineStartMs = 0;            // timeout para líneas rotas
static const size_t MAX_LINE_LEN = 320;   // tope de seguridad
static const uint32_t LINE_TIMEOUT_MS = 1200;

#define GEN_BUFFER_LINES 200
String genBuffer[GEN_BUFFER_LINES];
int genIndex = 0;

// ===== Estado app =====
enum AppMode { MODE_MONITOR=0, MODE_GENERATOR=1 };
volatile AppMode appMode = MODE_MONITOR;
volatile bool monitorRunning   = false;  // arranca pausado
volatile bool generatorRunning = false;  // arranca pausado
const int baudRates[4] = {4800,9600,38400,115200};

// ===== Generator =====
const int MAX_SLOTS = 4;
struct GenSlot {
  bool   enabled;
  String sensor;
  String sentence;
  String text;
};
GenSlot slots[MAX_SLOTS] = {
  {true,  "GPS",      "RMC", ""},   // slot 0
  {false, "GPS",      "VTG", ""},   // slot 1
  {false, "VELOCITY", "VHW", ""},   // slot 2
  {false, "HEADING",  "HDT", ""},   // slot 3
};
unsigned long slotInterval[MAX_SLOTS] = {500,500,500,500};
unsigned long lastSentMs  [MAX_SLOTS] = {0,0,0,0};

// ===== Sync =====
SemaphoreHandle_t nmeaBufMutex;
SemaphoreHandle_t genBufMutex;
SemaphoreHandle_t serialMutex;

// ====== ESTADO para OLED ======
volatile bool     otaActive = false;
volatile uint32_t bootStartMs = 0;

// Persistencia de detección
const uint32_t SENSOR_PRESENT_TTL_MS = 8000;   // 8s (marca “visto” por consola/web)
const uint32_t SPLASH_MS             = 2500;   // 2.5s splash
const uint32_t OLED_IDLE_WINDOW_MS   = 3500;   // ventana para mostrar sensores en OLED

// Marcas de última detección/generación por sensor (incluye CUSTOM)
struct SensorTrack {
  const char* name;
  volatile uint32_t lastSeenMs;
  volatile uint32_t lastGenMs;
};
SensorTrack sensors[] = {
  {"GPS",        0, 0},
  {"WEATHER",    0, 0},
  {"HEADING",    0, 0},
  {"SOUNDER",    0, 0},
  {"VELOCITY",   0, 0},
  {"RADAR",      0, 0},
  {"TRANSDUCER", 0, 0},
  {"AIS",        0, 0},
  {"CUSTOM",     0, 0}
};
const int SENSOR_COUNT = sizeof(sensors)/sizeof(sensors[0]);

// ============ LED ============
void flashLed(uint32_t color){
  pixels.setPixelColor(0,color);
  pixels.show();
  ledOn = true;
  ledMillis = millis();
}
void updateLed(){
  if(ledOn && millis()-ledMillis>=LED_DURATION){
    pixels.setPixelColor(0,0);
    pixels.show();
    ledOn=false;
  }
}

// ============ NMEA helpers ============
bool processNMEA(const String &line){ return (line.startsWith("$")||line.startsWith("!")); }

String detectSentenceType(const String &line){
  if (line.startsWith("!")) return "AIS";
  if (line.length()>=6 && line[0]=='$'){
    String f = line.substring(3,6); f.toUpperCase();
    if (f=="GLL"||f=="RMC"||f=="VTG"||f=="GGA"||f=="GSA"||f=="GSV"||f=="DTM"||f=="ZDA"||
        f=="GNS"||f=="GST"||f=="GBS"||f=="GRS"||f=="RMB"||f=="RTE"||f=="BOD"||f=="XTE") return "GPS";
    if (f=="DBT"||f=="DPT"||f=="DBK"||f=="DBS") return "SOUNDER";
    if (f=="MWD"||f=="MWV"||f=="VWR"||f=="VWT"||f=="MTW"||f=="MTA"||f=="MMB"||f=="MHU"||f=="MDA") return "WEATHER";
    if (f=="HDG"||f=="HDT"||f=="HDM"||f=="THS"||f=="ROT"||f=="RSA") return "HEADING";
    if (f=="VHW"||f=="VLW"||f=="VBW") return "VELOCITY";
    if (f=="TLL"||f=="TTM"||f=="TLB"||f=="OSD") return "RADAR";
    if (f=="XDR") return "TRANSDUCER";
  }
  return "OTROS";
}
int sensorIndexByName(const String& n){
  for(int i=0;i<SENSOR_COUNT;i++) if(n.equalsIgnoreCase(sensors[i].name)) return i;
  return -1;
}
void stampSeen(const String& cat){
  int idx = sensorIndexByName(cat);
  if(idx>=0) sensors[idx].lastSeenMs = millis();
}
void stampGen(const String& cat){
  int idx = sensorIndexByName(cat);
  if(idx>=0) sensors[idx].lastGenMs = millis();
}

void sendUDP(const String &line){
  udp.beginPacket(udpAddress, udpPort);
  udp.print(line);
  udp.endPacket();
}

// ============ Builders / checksum ============
String nmeaChecksum(const String &payload){
  uint8_t cs=0; for(size_t i=0;i<payload.length();i++) cs^=(uint8_t)payload[i];
  char b[3]; snprintf(b,sizeof(b),"%02X",cs); return String(b);
}
String buildDollarSentence(const String& talker,const String& code,const String& fields){
  String payload = talker+code+","+fields;
  return "$"+payload+"*"+nmeaChecksum(payload);
}
String buildAISSentence_VDM(){
  String p="AIVDM,1,1,,A,13aG?P0P00PD;88MD5MT?wvl0<0,0";
  return "!"+p+"*"+nmeaChecksum(p);
}
String buildAISSentence_VDO(){
  String p="AIVDO,1,1,,A,13aG?P0P00PD;88MD5MT?wvl0<0,0";
  return "!"+p+"*"+nmeaChecksum(p);
}
String talkerForSensor(const String& s){
  if (s=="GPS") return "GP";
  if (s=="AIS") return "AI";
  if (s=="SOUNDER") return "SD";
  if (s=="HEADING") return "HC";
  if (s=="CUSTOM") return "";
  return "II"; // WEATHER / VELOCITY / RADAR / TRANSDUCER
}
String generateSentence(const String& sensor,const String& codeIn){
  if (sensor.equalsIgnoreCase("CUSTOM")||codeIn.equalsIgnoreCase("CUSTOM")) return "";
  String c=codeIn; c.toUpperCase();
  String t=talkerForSensor(sensor);

  if(sensor=="AIS"){ if(c=="AIVDO") return buildAISSentence_VDO(); return buildAISSentence_VDM(); }

  // GPS
  if(sensor=="GPS"){
    if(c=="RMC") return buildDollarSentence("GP",c,"123519,A,4807.038,N,01131.000,E,5.5,054.7,230394,003.1,W");
    if(c=="GGA") return buildDollarSentence("GP",c,"123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    if(c=="GLL") return buildDollarSentence("GP",c,"4916.45,N,12311.12,W,225444,A");
    if(c=="VTG") return buildDollarSentence("GP",c,"054.7,T,034.4,M,005.5,N,010.2,K");
    if(c=="GSA") return buildDollarSentence("GP",c,"A,3,04,05,09,12,24,25,29,31,,,,,2.5,1.3,2.1");
    if(c=="GSV") return buildDollarSentence("GP",c,"2,1,08,01,40,083,41,02,17,308,43,12,07,021,42,14,25,110,45");
    if(c=="DTM") return buildDollarSentence("GP",c,"W84,,0.0,N,0.0,E,0.0,W84");
    if(c=="ZDA") return buildDollarSentence("GP",c,"201530.00,04,07,2002,00,00");
    if(c=="GNS") return buildDollarSentence("GN",c,"123519,4807.038,N,01131.000,E,AN,08,0.9,545.4,46.9,,");
    if(c=="GST") return buildDollarSentence("GP",c,"123519,1.2,1.0,0.8,45.0,0.5,0.5,1.0");
    if(c=="GBS") return buildDollarSentence("GP",c,"123519,0.5,0.5,0.8,01,0.75,0.00,1.00");
    if(c=="GRS") return buildDollarSentence("GP",c,"123519,1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0");
    if(c=="RMB") return buildDollarSentence("GP",c,"A,0.66,L,ORIG,DEST,4916.45,N,12311.12,W,12.3,054.7,5.5,V");
    if(c=="RTE") return buildDollarSentence("GP",c,"1,1,c,ROUTE1,WP1,WP2,WP3");
    if(c=="BOD") return buildDollarSentence("GP",c,"045.0,T,023.0,M,DEST,ORIG");
    if(c=="XTE") return buildDollarSentence("GP",c,"A,A,0.66,L,N");
  }

  // WEATHER
  if(t=="II" && c=="MWD") return buildDollarSentence(t,c,"054.7,T,034.4,M,10.5,N,5.4,M");
  if(t=="II" && c=="MWV") return buildDollarSentence(t,c,"054.7,R,10.5,N,A");
  if(t=="II" && c=="VWR") return buildDollarSentence(t,c,"054.7,R,10.5,N,5.4,M,19.4,K");
  if(t=="II" && c=="VWT") return buildDollarSentence(t,c,"054.7,T,10.5,N,5.4,M,19.4,K");
  if(t=="II" && c=="MTW") return buildDollarSentence(t,c,"18.0,C");
  if(t=="II" && c=="MTA") return buildDollarSentence(t,c,"19.5,C");
  if(t=="II" && c=="MMB") return buildDollarSentence(t,c,"29.92,I");
  if(t=="II" && c=="MHU") return buildDollarSentence(t,c,"45.0,P");
  if(t=="II" && c=="MDA") return buildDollarSentence(t,c,"29.92,I,1.013,B,19.5,C,18.0,C,,");

  // HEADING
  if(t=="HC" && c=="HDG") return buildDollarSentence(t,c,"238.5,,E,0.5");
  if(t=="HC" && c=="HDT") return buildDollarSentence(t,c,"238.5,T");
  if(t=="HC" && c=="HDM") return buildDollarSentence(t,c,"236.9,M");
  if(t=="HC" && c=="THS") return buildDollarSentence(t,c,"238.5,A");
  if(t=="HC" && c=="ROT") return buildDollarSentence(t,c,"0.0,A");
  if(t=="HC" && c=="RSA") return buildDollarSentence(t,c,"0.0,A,0.0,A");

  // SOUNDER
  if(t=="SD" && c=="DBT") return buildDollarSentence(t,c,"036.4,f,011.1,M,006.0,F");
  if(t=="SD" && c=="DPT") return buildDollarSentence(t,c,"11.2,0.5");
  if(t=="SD" && c=="DBK") return buildDollarSentence(t,c,"036.4,f,011.1,M,006.0,F");
  if(t=="SD" && c=="DBS") return buildDollarSentence(t,c,"036.4,f,011.1,M,006.0,F");

  // VELOCITY
  if(t=="II" && c=="VHW") return buildDollarSentence(t,c,"054.7,T,034.4,M,5.5,N,10.2,K");
  if(t=="II" && c=="VLW") return buildDollarSentence(t,c,"12.4,N,0.5,N");
  if(t=="II" && c=="VBW") return buildDollarSentence(t,c,"5.5,0.1,0.0,5.3,0.1,0.0");

  // RADAR
  if(t=="II" && c=="TLL") return buildDollarSentence(t,c,"1,4916.45,N,12311.12,W,225444,TGT1");
  if(t=="II" && c=="TTM") return buildDollarSentence(t,c,"1,2.5,N,054.7,T,0.0,N,054.7,T,0.0,54.7,TGT1");
  if(t=="II" && c=="TLB") return buildDollarSentence(t,c,"1,LOCK,4916.45,N,12311.12,W,225444");
  if(t=="II" && c=="OSD") return buildDollarSentence(t,c,"054.7,A,5.5,N,10.2,K");

  // TRANSDUCER
  if(t=="II" && c=="XDR") return buildDollarSentence(t,c,"C,19.5,C,AirTemp");

  return buildDollarSentence(t,c,"");
}

// ============ HTML utils ============
String htmlEscape(const String& s){
  String o; o.reserve(s.length()+8);
  for(size_t i=0;i<s.length();++i){
    char c=s[i];
    if(c=='&') o += F("&amp;");
    else if(c=='<') o += F("&lt;");
    else if(c=='>') o += F("&gt;");
    else if(c=='\"') o += F("&quot;");
    else if(c=='\'') o += F("&#39;");
    else o += c;
  } return o;
}
String fullToEditable(const String& full){
  if(full.length()==0) return "";
  String s=full;
  char ch = (s[0]=='$'||s[0]=='!')? s[0] : 0;
  if(ch) s.remove(0,1);
  int star=s.indexOf('*');
  if(star>=0) s=s.substring(0,star);
  return (ch?String(ch):String(""))+s;
}
void pushGen(const String& line){
  xSemaphoreTake(genBufMutex,portMAX_DELAY);
  genIndex=(genIndex+1)%GEN_BUFFER_LINES;
  genBuffer[genIndex]=line;
  xSemaphoreGive(genBufMutex);
}

/* ===========================================================
   SOPORTE IEC61162-450 (UdPbC) + NMEA Tag Block (\ ... \)
   =========================================================== */

static String trimCopy(const String& in){
  String t=in; t.trim(); return t;
}
static int idxOfFirstSentenceStart(const String& s){
  int i1 = s.indexOf('$');
  int i2 = s.indexOf('!');
  if(i1<0) return i2;
  if(i2<0) return i1;
  return (i1<i2)? i1 : i2;
}
static bool parseHexByte(const String& hh, uint8_t &val){
  if(hh.length()<2) return false;
  char c1=hh[0], c2=hh[1];
  auto nib=[&](char c)->int{
    if(c>='0'&&c<='9') return c-'0';
    if(c>='A'&&c<='F') return 10 + (c-'A');
    if(c>='a'&&c<='f') return 10 + (c-'a');
    return -1;
  };
  int n1=nib(c1), n2=nib(c2);
  if(n1<0||n2<0) return false;
  val=(uint8_t)((n1<<4)|n2);
  return true;
}
static bool verifyTagChecksum(const String& inner){
  int asterisk = inner.lastIndexOf('*');
  if(asterisk<0 || asterisk+2 >= inner.length()) return true; // sin checksum → aceptamos
  String payload = inner.substring(0, asterisk);
  String hh = inner.substring(asterisk+1);
  if(hh.length()<2) return false;
  uint8_t want=0; if(!parseHexByte(hh.substring(0,2), want)) return false;
  uint8_t cs=0; for(size_t i=0;i<payload.length();++i) cs ^= (uint8_t)payload[i];
  return (cs==want);
}
static void parseTagPairs(const String& inner, String &meta){
  int asterisk = inner.lastIndexOf('*');
  String body = (asterisk>0)? inner.substring(0,asterisk) : inner;
  meta = "";
  int start=0;
  while(start < body.length()){
    int comma = body.indexOf(',', start);
    String tok = (comma<0)? body.substring(start) : body.substring(start, comma);
    tok.trim();
    int colon = tok.indexOf(':');
    if(colon>0){
      String key = tok.substring(0, colon); key.trim();
      String val = tok.substring(colon+1);  val.trim();
      if(meta.length()) meta += " ";
      meta += key + "=" + val;
    }
    if(comma<0) break;
    start = comma+1;
  }
}
static bool parseNMEALine(const String& rawIn, String &outSentence, String &outMeta, bool &hadTag, bool &hadUdPbC){
  outSentence = ""; outMeta = ""; hadTag=false; hadUdPbC=false;
  String s = trimCopy(rawIn);
  if(s.length()==0) return false;

  // Tag Block
  if(s[0]=='\\'){
    int end = s.indexOf('\\', 1);
    if(end>1){
      String inner = s.substring(1, end);
      hadTag = true;
      if(verifyTagChecksum(inner)) parseTagPairs(inner, outMeta);
      else {
        parseTagPairs(inner, outMeta);
        if(outMeta.length()) outMeta += " ";
        outMeta += "cs=BAD";
      }
      s = s.substring(end+1);
      s.trim();
    }
  }

  // Prefijo UdPbC
  if(s.startsWith("UdPbC") || s.startsWith("UDPBC") || s.startsWith("udpbc") || s.startsWith("udPbc")){
    hadUdPbC = true;
    int pos = idxOfFirstSentenceStart(s);
    if(pos>=0) s = s.substring(pos);
  }

  // Buscar primer '$'/'!'
  if(!(s.startsWith("$")||s.startsWith("!"))){
    int pos = idxOfFirstSentenceStart(s);
    if(pos<0) return false;
    s = s.substring(pos);
  }

  outSentence = s;
  return true;
}

// ============ Serial control ============
void startSerial(int baud){
  xSemaphoreTake(serialMutex,portMAX_DELAY);
  NMEA_Serial.end(); delay(5);
  NMEA_Serial.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN);
  while(NMEA_Serial.available()) (void)NMEA_Serial.read();
  currentBaud = baud;
  xSemaphoreGive(serialMutex);
}

// ============ Web helpers ============
void noCache(){
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma","no-cache");
  server.sendHeader("Expires","0");
}
void handle204(){ noCache(); server.send(204,"text/plain",""); }
void handleCaptive(){
  noCache();
  server.send(200,"text/html; charset=utf-8",
    F("<!doctype html><html><meta name='viewport' content='width=device-width, initial-scale=1.0'>"
      "<title>NMEA Link</title><body style='background:#000;color:#0f0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,Noto Sans,Liberation Sans,sans-serif;'>"
      "<p>Redirecting…</p><script>location.href='/'</script></body></html>")
  );
}

// ============ MENU ============
void handleMenu(){
  // Parar TODO al entrar al menú
  generatorRunning = false;
  monitorRunning  = false;
  otaActive       = false;

  String html = F("<!doctype html><html><head><meta charset='utf-8'><title>NMEA Link</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
  "<style>body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,Noto Sans,Liberation Sans,sans-serif;background:#000;color:#0f0;margin:0;padding:10px}"
  "h2{text-align:center;color:#0ff;margin:8px 0}.btn{padding:14px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:10px;font-size:18px;cursor:pointer;text-align:center;display:block;width:100%}"
  ".btn:hover{background:#0f0;color:#000}.stack{display:flex;flex-direction:column;gap:10px;max-width:680px;margin:12px auto}"
  "footer{text-align:center;color:#666;font-size:12px;margin-top:10px}"
  ".lang{position:absolute;top:10px;right:10px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:6px;padding:4px}</style></head><body>");

  html += F("<select id='lang' class='lang' onchange='setLang(this.value)'><option value='en'>EN</option><option value='es'>ES</option><option value='fr'>FR</option></select>"
            "<h2 id='ttl'>NMEA Link</h2><div class='stack'>"
            "<button type='button' class='btn' id='b1' onclick='goMon()'>NMEA Monitor</button>"
            "<button type='button' class='btn' id='b2' onclick='goGen()'>NMEA Generator</button>"
            "<button type='button' class='btn' id='b3' onclick='goOTA()'>OTA Update</button>"
            "</div><footer>© 2025 Matías Scuppa — by Themys</footer>");

  html +=
    "<script>"
    "let lang=localStorage.getItem('lang')||'en';"
    "const L={en:{t:'NMEA Link',m:'NMEA Monitor',g:'NMEA Generator',o:'OTA Update'},"
    "es:{t:'NMEA Link',m:'NMEA Monitor',g:'NMEA Generator',o:'Actualizar Firmware'},"
    "fr:{t:'NMEA Link',m:'NMEA Monitor',g:'NMEA Generator',o:'Mise à jour OTA'}};"
    "function setLang(l){lang=l;localStorage.setItem('lang',l);apply();}"
    "function apply(){document.getElementById('ttl').innerText=L[lang].t||'NMEA Link';document.getElementById('b1').innerText=L[lang].m;document.getElementById('b2').innerText=L[lang].g;document.getElementById('b3').innerText=L[lang].o;document.getElementById('lang').value=lang;}"
    "async function goMon(){try{await fetch('/togglegen?state=0');await fetch('/setmonitor?state=0');await fetch('/setmode?m=monitor');}catch(e){} location.href='/monitor';}"
    "async function goGen(){try{await fetch('/togglegen?state=0');await fetch('/setmonitor?state=0');await fetch('/setmode?m=generator');}catch(e){} location.href='/generator';}"
    "async function goOTA(){try{await fetch('/togglegen?state=0');await fetch('/setmonitor?state=0');}catch(e){} location.href='/update';}"
    "document.addEventListener('DOMContentLoaded',apply);"
    "</script></body></html>";

  noCache(); server.send(200,"text/html; charset=utf-8",html);
}

// ============ MONITOR ============
void handleMonitor(){
  otaActive = false;
  String html = F("<!doctype html><html><head><meta charset='utf-8'><title>NMEA Reader</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
  "<style>body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,Noto Sans,Liberation Sans,sans-serif;background:#000;color:#0f0;margin:0;padding:10px}"
  "h2{text-align:center;color:#0ff;margin:8px 0}.lang{position:absolute;top:10px;right:10px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:6px;padding:4px}"
  "#console{width:100%;max-width:100%;box-sizing:border-box;height:40vh;overflow:auto;border:1px solid #0f0;padding:5px;background:#000;font-size:14px;white-space:pre-wrap;word-wrap:break-word;overflow-wrap:anywhere;margin-top:12px;font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,'Liberation Mono',monospace}"
  ".btnc{display:flex;flex-wrap:wrap;gap:5px;margin:8px 0}.btn{flex:1;padding:10px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:8px;font-size:16px;text-align:center;cursor:pointer}"
  ".btn.active{background:#0f0;color:#000;font-weight:bold}.fbtn{flex:1 1 calc(33.33% - 6px);padding:5px 0;border-radius:5px;margin:2px;text-align:center;transition:.2s;border:1px solid #333}"
  ".fbtn:not(.active){background:#111;color:#666;border-color:#444}.fbtn.active{font-weight:600;border:1px solid #222}"
  ".fbtn.active.GPS{background:#0ff;color:#000}.GPS{color:#0ff}"
  ".fbtn.active.AIS{background:#ff0;color:#000}.AIS{color:#ff0}"
  ".fbtn.active.SOUNDER{background:#0f0;color:#000}.SOUNDER{color:#0f0}"
  ".fbtn.active.VELOCITY{background:#f0f;color:#000}.VELOCITY{color:#f0f}"
  ".fbtn.active.HEADING{background:#1e90ff;color:#000}.HEADING{color:#1e90ff}"
  ".fbtn.active.RADAR{background:#ff4500;color:#000}.RADAR{color:#ff4500}"
  ".fbtn.active.WEATHER{background:#7fffd4;color:#000}.WEATHER{color:#7fffd4}"
  ".fbtn.active.TRANSDUCER{background:#ffa500;color:#000}.TRANSDUCER{color:#ffa500}"
  ".fbtn.active.OTROS{background:#aaa;color:#000}.OTROS{color:#aaa}"
  "footer{text-align:center;color:#666;font-size:12px;margin-top:10px}</style></head><body>");

  html += F("<select id='lang' class='lang' onchange='setLang(this.value)'><option value='en'>EN</option><option value='es'>ES</option><option value='fr'>FR</option></select>"
            "<h2 id='title'>NMEA Reader</h2><div class='btnc' id='filterC'></div><div id='console'></div>");

  // baud
  html += "<div class='btnc'>";
  for(int i=0;i<4;i++){
    html += "<button type='button' id='baud_"+String(baudRates[i])+"' class='btn baud' onclick='setBaud("+String(baudRates[i])+")'>"+String(baudRates[i])+"</button>";
  }
  html += "</div>";

  // start/clear
  html += "<div class='btnc'><button type='button' id='pauseBtn' class='btn' onclick='togglePause()'>▶ Start</button>"
          "<button type='button' id='clearBtn' class='btn' onclick='clearConsole()'>🧹 Clear</button></div>";

  // speed
  html += "<div class='btnc'>"
          "<button type='button' class='btn' onclick='setSpeed(0.25,this)'>25%</button>"
          "<button type='button' class='btn active' onclick='setSpeed(0.5,this)'>50%</button>"
          "<button type='button' class='btn' onclick='setSpeed(0.75,this)'>75%</button>"
          "<button type='button' class='btn' onclick='setSpeed(1,this)'>100%</button></div>";

  // nav
  html += "<div class='btnc'><button type='button' class='btn' onclick='gotoGen()'>➡ NMEA Generator</button></div>"
          "<div class='btnc'><button type='button' class='btn' onclick='gotoMenu()'>🏠 Main Menu</button></div>"
          "<footer>© 2025 Matías Scuppa — by Themys</footer>";

  html +=
    "<script>"
    "let lang=localStorage.getItem('lang')||'en';"
    "const Lb={en:{pause:'⏸ Pause',resume:'▶ Start',clear:'🧹 Clear'},"
    "es:{pause:'⏸ Pausar',resume:'▶ Iniciar',clear:'🧹 Limpiar'},"
    "fr:{pause:'⏸ Pause',resume:'▶ Démarrer',clear:'🧹 Effacer'}};"
    "const cat={"
      "en:{GPS:'GPS',AIS:'AIS',SOUNDER:'SOUNDER',VELOCITY:'VELOCITY',HEADING:'HEADING',RADAR:'RADAR',WEATHER:'WEATHER',TRANSDUCER:'TRANSDUCER',OTROS:'OTHER'},"
      "es:{GPS:'GPS',AIS:'AIS',SOUNDER:'ECOSONDA',VELOCITY:'VELOCIDAD',HEADING:'RUMBO',RADAR:'RADAR',WEATHER:'METEO',TRANSDUCER:'TRANSDUCTOR',OTROS:'OTROS'},"
      "fr:{GPS:'GPS',AIS:'AIS',SOUNDER:'SONDEUR',VELOCITY:'VITESSE',HEADING:'CAP',RADAR:'RADAR',WEATHER:'MÉTÉO',TRANSDUCER:'TRANSDUCTEUR',OTROS:'AUTRES'}"
    "};"
    "let filters=['GPS','AIS','SOUNDER','VELOCITY','HEADING','RADAR','WEATHER','TRANSDUCER','OTROS'];let filtersState={};filters.forEach(f=>filtersState[f]=true);"
    "let paused=true, intervalMs=1000, intervalId=null;"
    "function setLang(l){lang=l;localStorage.setItem('lang',l);applyLang();}"
    "function applyLang(){document.getElementById('pauseBtn').innerText=paused?Lb[lang].resume:Lb[lang].pause;document.getElementById('clearBtn').innerText=Lb[lang].clear;drawFilters();}"
    "function drawFilters(){let c=document.getElementById('filterC');c.innerHTML='';filters.forEach(f=>{let b=document.createElement('button');b.type='button';b.className='fbtn '+f;if(filtersState[f])b.classList.add('active');b.innerText=cat[lang][f]||f;b.onclick=()=>{filtersState[f]=!filtersState[f];b.classList.toggle('active',filtersState[f]);};c.appendChild(b);});let all=document.createElement('button');all.type='button';all.className='fbtn';all.innerText='ALL/NONE';all.onclick=()=>{let any=Object.values(filtersState).some(v=>v);Object.keys(filtersState).forEach(k=>filtersState[k]=!any);drawFilters();};c.appendChild(all);}"
    "function togglePause(){paused=!paused;applyLang();fetch('/setmonitor?state='+(paused?0:1)).catch(()=>{});}"
    "function clearConsole(){document.getElementById('console').innerHTML='';fetch('/clearnmea').catch(()=>{});}"
    "async function setBaud(b){await fetch('/setbaud?baud='+b).catch(()=>{});document.querySelectorAll('.baud').forEach(x=>x.classList.remove('active'));let el=document.getElementById('baud_'+b);if(el)el.classList.add('active');}"
    "function setSpeed(mult,btn){document.querySelectorAll('.btn').forEach(b=>{if(b.innerText.includes('%'))b.classList.remove('active');});btn.classList.add('active');intervalMs=Math.max(100,Math.round(1000/mult));if(intervalId)clearInterval(intervalId);intervalId=setInterval(poll,intervalMs);}"
    "function poll(){if(paused)return;fetch('/getnmea?ts='+Date.now()).then(r=>r.text()).then(t=>{let c=document.getElementById('console');let lines=t.trim()?t.trim().split('\\n'):[];"
      "let out=lines.map(l=>{if(!l)return'';let lb=l.indexOf(']');let typ=(lb>0&&l[0]=='[')?l.substring(1,lb):'OTROS';if(!filtersState[typ])return null;"
      "let rest=(lb>=0)?l.substring(lb+1):l;return '<span class=\"'+typ+'\">['+(cat[lang][typ]||typ)+']'+rest+'</span>';}).filter(Boolean).join('<br>');"
      "c.innerHTML=out;c.scrollTop=c.scrollHeight;}).catch(()=>{});}"
    "async function gotoGen(){paused=true;applyLang();try{await fetch('/setmonitor?state=0');await fetch('/setmode?m=generator');}catch(e){} location.href='/generator';}"
    "async function gotoMenu(){paused=true;try{await fetch('/setmonitor?state=0');await fetch('/togglegen?state=0');}catch(e){} location.href='/';}"
    "document.addEventListener('DOMContentLoaded',async()=>{await fetch('/setmode?m=monitor');try{const st=await (await fetch('/getstatus')).json();paused=!st.monRunning;applyLang();let b=document.getElementById('baud_'+(st.baud||4800));if(b)b.classList.add('active');}catch(e){applyLang();}intervalId=setInterval(poll,intervalMs);});"
    "window.addEventListener('beforeunload',()=>{if(intervalId)clearInterval(intervalId);});"
    "</script></body></html>";

  noCache(); server.send(200,"text/html; charset=utf-8",html);
}

// ===== listas Generator (lado servidor) =====
const char* SENSOR_LIST[]={"GPS","WEATHER","HEADING","SOUNDER","VELOCITY","RADAR","TRANSDUCER","AIS","CUSTOM"};
const int SENSOR_COUNT_CONST=9;

static void appendOption(String &out,const char* v,const String &selected){
  out += "<option value='"; out += v; out += "'";
  if(selected==v) out += " selected";
  out += ">"; out += v; out += "</option>";
}
String optionsForSensorSelect(const String& current){
  String s; for(int i=0;i<SENSOR_COUNT_CONST;i++){ const char* v=SENSOR_LIST[i];
    s += "<option value='"; s += v; s += "'";
    if(String(v)==current) s += " selected";
    s += ">"; s += v; s += "</option>";
  } return s;
}
String optionsForSentence(const String& sensor,const String& selected){
  String out;
  if(sensor=="GPS"){
    const char* arr[]={"GLL","RMC","VTG","GGA","GSA","GSV","DTM","ZDA","GNS","GST","GBS","GRS","RMB","RTE","BOD","XTE"};
    for(size_t i=0;i<sizeof(arr)/sizeof(arr[0]);i++) appendOption(out,arr[i],selected);
  } else if(sensor=="WEATHER"){
    const char* arr[]={"MWD","MWV","VWR","VWT","MTW","MTA","MMB","MHU","MDA"};
    for(size_t i=0;i<sizeof(arr)/sizeof(arr[0]);i++) appendOption(out,arr[i],selected);
  } else if(sensor=="HEADING"){
    const char* arr[]={"HDG","HDT","HDM","THS","ROT","RSA"};
    for(size_t i=0;i<sizeof(arr)/sizeof(arr[0]);i++) appendOption(out,arr[i],selected);
  } else if(sensor=="SOUNDER"){
    const char* arr[]={"DBT","DPT","DBK","DBS"};
    for(size_t i=0;i<sizeof(arr)/sizeof(arr[0]);i++) appendOption(out,arr[i],selected);
  } else if(sensor=="VELOCITY"){
    const char* arr[]={"VHW","VLW","VBW"};
    for(size_t i=0;i<sizeof(arr)/sizeof(arr[0]);i++) appendOption(out,arr[i],selected);
  } else if(sensor=="RADAR"){
    const char* arr[]={"TLL","TTM","TLB","OSD"};
    for(size_t i=0;i<sizeof(arr)/sizeof(arr[0]);i++) appendOption(out,arr[i],selected);
  } else if(sensor=="TRANSDUCER"){
    const char* arr[]={"XDR"};
    for(size_t i=0;i<sizeof(arr)/sizeof(arr[0]);i++) appendOption(out,arr[i],selected);
  } else if(sensor=="AIS"){
    const char* arr[]={"AIVDM","AIVDO"};
    for(size_t i=0;i<sizeof(arr)/sizeof(arr[0]);i++) appendOption(out,arr[i],selected);
  } else {
    appendOption(out,"CUSTOM",selected);
  }
  if(out.length()==0) appendOption(out,"CUSTOM",selected);
  return out;
}
String initialEditableForSlot(int i){
  String full;
  if(slots[i].text.length()) full=slots[i].text;
  else {
    if(slots[i].sensor=="CUSTOM"||slots[i].sentence=="CUSTOM"){
      String payload="GPCUS,FIELD1,FIELD2";
      full="$"+payload+"*"+nmeaChecksum(payload);
    } else full=generateSentence(slots[i].sensor,slots[i].sentence);
  }
  return htmlEscape(fullToEditable(full));
}

// ============ GENERATOR ============
void handleGenerator(){
  otaActive = false;
  String html = F("<!doctype html><html><head><meta charset='utf-8'><title>NMEA Generator</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
  "<style>body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,Noto Sans,Liberation Sans,sans-serif;background:#000;color:#0f0;margin:0;padding:10px}"
  "h2{text-align:center;color:#0ff;margin:8px 0}"
  ".grid{display:grid;grid-template-columns:1fr;gap:10px}"
  ".card{border:1px solid #0f0;border-radius:8px;padding:8px;background:#000;text-align:left}"
  "label{display:block;margin:6px 0 4px 0;font-weight:bold;text-align:left !important}"
  ".col{display:flex;flex-direction:column;align-items:flex-start;text-align:left}"
  ".label-inline{display:inline-flex;align-items:center;gap:8px;justify-content:flex-start;text-align:left}"
  ".label-inline input[type=checkbox]{margin:0 6px 0 0;transform:scale(1.1);accent-color:#0f0}"
  "select,input{width:100%;box-sizing:border-box;padding:6px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:6px;text-align:left}"
  ".row{display:flex;gap:10px;flex-wrap:wrap;align-items:flex-start;justify-content:flex-start}"
  ".row>*{flex:1;min-width:220px;text-align:left}"
  ".row.spaceTop{margin-top:8px}"
  ".btn{padding:10px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:8px;font-size:16px;cursor:pointer;text-align:center;display:inline-block;box-sizing:border-box;line-height:1.2}"
  ".btn.small{padding:6px 8px;font-size:14px;border-radius:6px}"
  ".btn.active{background:#0f0;color:#000;font-weight:bold}"
  "#genconsole{width:100%;box-sizing:border-box;height:40vh;overflow:auto;border:1px solid #0f0;padding:5px;background:#000;margin-top:10px;font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,'Liberation Mono',monospace}"
  ".btn-row{display:flex;gap:6px;margin-top:10px;align-items:stretch}"
  ".btn-row .start{flex:2}"
  ".btn-row .clear{flex:1}"
  ".btn-full{width:100%;display:block}"
  "footer{text-align:center;color:#666;font-size:12px;margin-top:10px}"
  "a.btn{text-decoration:none}"
  "</style></head><body>");

  html += "<h2 id='genTitle'>NMEA Generator</h2><div class='grid' id='slots'>";
  for(int i=0;i<MAX_SLOTS;i++){
    unsigned long ms=slotInterval[i];
    bool a100=(ms==100),a500=(ms==500),a1000=(ms==1000),a2000=(ms==2000);
    html += "<div class='card' id='slot_"+String(i)+"'>";

    html += "  <div class='row'>";
    html += "    <div class='col'>";
    html += "      <label class='label-inline'><input type='checkbox' id='en_"+String(i)+"'"+(slots[i].enabled?" checked":"")+"><span class='lblSensor'>Sensor</span></label>";
    html += "      <select id='sensor_"+String(i)+"'>"+optionsForSensorSelect(slots[i].sensor)+"</select>";
    html += "    </div>";
    html += "    <div class='col'>";
    html += "      <label class='lblSentence'>Sentence type</label>";
    html += "      <select id='sentence_"+String(i)+"'>"+optionsForSentence(slots[i].sensor,slots[i].sentence)+"</select>";
    html += "    </div>";
    html += "  </div>";

    html += "  <div class='row spaceTop'><div style='flex:1 1 100%'><input id='text_"+String(i)+"' placeholder='$GPRMC,...' autocomplete='off' value='"+initialEditableForSlot(i)+"'></div></div>";

    html += "  <div class='row spaceTop'><div style='flex:1 1 100%'><label class='lblIntervalSlot'>Interval</label><div id='intgrp_"+String(i)+"' class='row' style='gap:8px'>";
    html += String("    <button type='button' class='btn small int-btn") + (a100? " active" : "") + "' onclick='setIntervalSlot(" + String(i) + ",100,this)'>0.1s</button>";
    html += String("    <button type='button' class='btn small int-btn") + (a500? " active" : "") + "' onclick='setIntervalSlot(" + String(i) + ",500,this)'>0.5s</button>";
    html += String("    <button type='button' class='btn small int-btn") + (a1000? " active" : "") + "' onclick='setIntervalSlot(" + String(i) + ",1000,this)'>1s</button>";
    html += String("    <button type='button' class='btn small int-btn") + (a2000? " active" : "") + "' onclick='setIntervalSlot(" + String(i) + ",2000,this)'>2s</button>";
    html += "  </div></div></div>";

    html += "</div>";
  }
  html += "</div>";

  // baud
  html += "<label id='lblBaud'>Baudrate</label><div class='row'>";
  for(int i=0;i<4;i++){
    html += "<button type='button' id='gen_baud_"+String(baudRates[i])+"' class='btn gen-baud' onclick='setGenBaud("+String(baudRates[i])+",this)'>"+String(baudRates[i])+"</button>";
  }
  html += "</div>";

  // visor + botones + NAV extra hacia MONITOR
  html += "<div id='genconsole'></div>"
          "<div class='btn-row'>"
          "<button type='button' id='startBtn' class='btn start' onclick='toggleGen(event)'>▶ Iniciar</button>"
          "<button type='button' id='clearBtn' class='btn clear' onclick='clearGen(event)'>🧹 Limpiar</button>"
          "</div>"
          "<div class='btn-row'><a class='btn btn-full' href='/monitor' onclick='try{fetch(\"/togglegen?state=0\");}catch(e){}'>⬅ NMEA Monitor</a></div>"
          "<div class='btn-row'><a class='btn btn-full' href='/' onclick='try{fetch(\"/togglegen?state=0\");}catch(e){}'>🏠 Main Menu</a></div>";

  html +=
    "<script>"
    "const sentencesBySensor={GPS:['GLL','RMC','VTG','GGA','GSA','GSV','DTM','ZDA','GNS','GST','GBS','GRS','RMB','RTE','BOD','XTE'],"
    "WEATHER:['MWD','MWV','VWR','VWT','MTW','MTA','MMB','MHU','MDA'],HEADING:['HDG','HDT','HDM','THS','ROT','RSA'],"
    "SOUNDER:['DBT','DPT','DBK','DBS'],VELOCITY:['VHW','VLW','VBW'],RADAR:['TLL','TTM','TLB','OSD'],TRANSDUCER:['XDR'],AIS:['AIVDM','AIVDO'],CUSTOM:[]};"
    "let lang=localStorage.getItem('lang')||'en';"
    "const L={en:{title:'NMEA Generator',sensor:'Sensor',sentenceSel:'Sentence type',sentenceInline:'Sentence',interval:'Interval',start:'▶ Start',pause:'⏸ Pause',clear:'🧹 Clear',back:'⬅ NMEA Monitor',baud:'Baudrate'},"
    "es:{title:'NMEA Generator',sensor:'Sensor',sentenceSel:'Tipo de sentencia',sentenceInline:'Sentencia',interval:'Intervalo',start:'▶ Iniciar',pause:'⏸ Pausar',clear:'🧹 Limpiar',back:'⬅ NMEA Monitor',baud:'Baudrate'},"
    "fr:{title:'NMEA Generator',sensor:'Capteur',sentenceSel:'Type de trame',sentenceInline:'Trame',interval:'Intervalle',start:'▶ Démarrer',pause:'⏸ Pause',clear:'🧹 Effacer',back:'⬅ NMEA Monitor',baud:'Baudrate'}};"
    "function hex2(n){return n.toString(16).toUpperCase().padStart(2,'0');}"
    "function csPayload(s){let cs=0;for(let i=0;i<s.length;i++){cs^=s.charCodeAt(i);}return hex2(cs);} "
    "function buildFullFromEditor(str){ if(!str) return ''; str=str.trim(); let ch=null; if(str[0]==='$'||str[0]==='!'){ ch=str[0]; str=str.slice(1);} let up=str.toUpperCase(); if(!ch) ch=(up.startsWith('AIVDM')||up.startsWith('AIVDO'))?'!':'$'; let payload=str; return ch+payload+'*'+csPayload(payload);} "
    "function refillSent(sensorSel,sentSel){sentSel.innerHTML='';const arr=sentencesBySensor[sensorSel.value]||[];if(arr.length===0){let o=document.createElement('option');o.value='CUSTOM';o.text='CUSTOM';sentSel.appendChild(o);}else{for(let i=0;i<arr.length;i++){let o=document.createElement('option');o.value=arr[i];o.text=arr[i];sentSel.appendChild(o);}}}"
    "async function getStatus(){try{const r=await fetch('/getstatus');return await r.json();}catch(e){return {baud:4800,genRunning:false};}}"
    "function initSlot(i){const en=document.getElementById('en_'+i),sensorSel=document.getElementById('sensor_'+i),sentSel=document.getElementById('sentence_'+i),txt=document.getElementById('text_'+i);"
    " en.addEventListener('change',e=>{fetch('/gen_slot_enable?i='+i+'&en='+(e.target.checked?1:0)).catch(()=>{});});"
    " sensorSel.addEventListener('change',async ()=>{refillSent(sensorSel,sentSel);const newSent=sentSel.value;try{await fetch('/gen_slot_sensor?i='+i+'&sensor='+sensorSel.value);await fetch('/gen_slot_sentence?i='+i+'&sentence='+newSent);const r=await fetch('/gen_slot_template?i='+i);const t=await r.text();const ch=(t&&(t[0]==='$'||t[0]==='!'))?t[0]:'';let s=t? t.slice(ch?1:0):'';let star=s.indexOf('*'); if(star>=0) s=s.slice(0,star);txt.value=(ch?s?ch+s:s:s);}catch(e){}});"
    " sentSel.addEventListener('change',async ()=>{try{await fetch('/gen_slot_sentence?i='+i+'&sentence='+sentSel.value);const r=await fetch('/gen_slot_template?i='+i);const t=await r.text();const ch=(t&&(t[0]==='$'||t[0]==='!'))?t[0]:'';let s=t? t.slice(ch?1:0):'';let star=s.indexOf('*'); if(star>=0) s=s.slice(0,star);txt.value=(ch?s?ch+s:s:s);}catch(e){}});"
    " txt.addEventListener('input',e=>{ if(e.target.value.indexOf('*')>=0){ e.target.value=e.target.value.replace(/\\*/g,''); } const full=buildFullFromEditor(e.target.value); fetch('/gen_slot_text',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'i='+i+'&text='+encodeURIComponent(full)}).catch(()=>{});});"
    "} "
    "function setActive(sel,scope,el){(scope||document).querySelectorAll(sel).forEach(b=>b.classList.remove('active')); if(el) el.classList.add('active');}"
    "function setIntervalSlot(i,ms,btn){fetch('/gen_slot_interval?i='+i+'&ms='+ms).then(()=>{const g=document.getElementById('intgrp_'+i);if(!g)return;setActive('.int-btn',g,btn);}).catch(()=>{});} "
    "async function setGenBaud(b,btn){try{await fetch('/setbaud?baud='+b);setActive('.gen-baud',document,btn);}catch(e){}}"
    "let running=false;"
    "async function toggleGen(e){if(e)e.preventDefault();try{running=!running;const r=await fetch('/togglegen?state='+(running?'1':'0'));const t=await r.text();running=(t==='RUNNING');document.getElementById('startBtn').innerText=running?L[lang].pause:L[lang].start;}catch(err){}}"
    "function clearGen(e){if(e)e.preventDefault();fetch('/cleargen').catch(()=>{});document.getElementById('genconsole').innerHTML='';}"
    "function pollGen(){fetch('/getgen?ts='+Date.now()).then(r=>r.text()).then(t=>{let c=document.getElementById('genconsole');c.innerHTML=(t||'').split('\\n').join('<br>');c.scrollTop=c.scrollHeight;}).catch(()=>{});} setInterval(pollGen,300);"
    "function applyLang(){document.getElementById('genTitle').innerText=L[lang].title;document.getElementById('startBtn').innerText=running?L[lang].pause:L[lang].start;document.getElementById('clearBtn').innerText=L[lang].clear;document.getElementById('lblBaud').innerText=L[lang].baud;document.querySelectorAll('.lblSensor').forEach(e=>e.innerText=L[lang].sensor);document.querySelectorAll('.lblSentence').forEach(e=>e.innerText=L[lang].sentenceSel);document.querySelectorAll('.lblIntervalSlot').forEach(e=>e.innerText=L[lang].interval);}"
    "document.addEventListener('DOMContentLoaded',async()=>{fetch('/setmode?m=generator');lang=localStorage.getItem('lang')||'en';for(let i=0;i<"+ String(MAX_SLOTS) +";i++){initSlot(i);}const st=await getStatus();running=!!st.genRunning;applyLang();var b=document.getElementById('gen_baud_'+(st.baud||4800));if(b)b.classList.add('active');});"
    "</script><footer>© 2025 Matías Scuppa — by Themys</footer></body></html>";

  noCache(); server.send(200,"text/html; charset=utf-8",html);
}

// ============ OTA ============
void handleUpdatePage(){
  generatorRunning=false; monitorRunning=false;
  otaActive = true; // WARNING en OLED

  // Contenedor verde y botón Main Menu igual al de Upload (mismo <button>, mismas clases)
  String html = F(
    "<!doctype html><html><head><meta charset='utf-8'><title>OTA Update</title>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<style>"
      "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,Noto Sans,Liberation Sans,sans-serif;background:#000;color:#0f0;margin:0;padding:0}"
      ".wrap{min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px;box-sizing:border-box}"
      ".card{width:100%;max-width:680px;background:#002900;border:1px solid #0f0;border-radius:12px;padding:16px;box-sizing:border-box;margin:0 auto}"
      "h2{text-align:center;color:#0ff;margin:8px 0 12px}"
      ".warn{border:1px solid #ff0;color:#ff0;padding:8px;border-radius:8px;background:#111;margin:8px 0}"
      "input[type=file]{width:100%;box-sizing:border-box;padding:8px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:8px;margin:10px 0}"
      ".btn{padding:10px;background:#111;color:#0f0;border:1px solid #0f0;border-radius:8px;font-size:16px;cursor:pointer;text-align:center;display:inline-block;box-sizing:border-box;line-height:1.2}"
      ".btn:hover{background:#0f0;color:#000}"
      ".btn-full{width:100%;display:block}"
      "#status{margin-top:8px;color:#7fffd4;min-height:1.2em}"
      "footer{text-align:center;color:#666;font-size:12px;margin-top:10px}"
    "</style></head><body>"
    "<div class='wrap'><div class='card'>"
      "<h2 id='ttl'>OTA Update</h2>"
      "<div class='warn'>⚠️ <b>Do not disconnect power</b> while the firmware is uploading.</div>"
      "<input id='file' type='file' accept='.bin'>"
      "<button type='button' class='btn btn-full' id='btnUp' onclick='doUpload()'>Upload</button>"
      "<div id='status'></div>"
      "<button type='button' class='btn btn-full' id='btnMenu' onclick=\"location.href='/'\">🏠 Main Menu</button>"
      "<footer>© 2025 Matías Scuppa — by Themys</footer>"
    "</div></div>"
    "<script>"
    "let lang=localStorage.getItem('lang')||'en';const T={"
      "en:{title:'OTA Update',msg:'Select the firmware .bin file and upload. The device will reboot automatically.',upload:'Upload',menu:'🏠 Main Menu',ok:'Upload OK. Rebooting…',fail:'Upload failed.'},"
      "es:{title:'Actualizar Firmware',msg:'Selecciona el archivo .bin y súbelo. El equipo se reiniciará automáticamente.',upload:'Subir',menu:'🏠 Menú Principal',ok:'Subida OK. Reiniciando…',fail:'Fallo en la subida.'},"
      "fr:{title:'Mise à jour OTA',msg:'Sélectionnez le fichier .bin et téléversez-le. L’appareil redémarrera automatiquement.',upload:'Téléverser',menu:'🏠 Menu Principal',ok:'Téléversement OK. Redémarrage…',fail:'Échec du téléversement.'}"
    "};"
    "function apply(){document.getElementById('ttl').innerText=T[lang].title;document.getElementById('btnUp').innerText=T[lang].upload;document.getElementById('btnMenu').innerText=T[lang].menu;}apply();"
    "function doUpload(){const f=document.getElementById('file').files[0];if(!f){document.getElementById('status').innerText='No file';return;}const fd=new FormData();fd.append('update',f,f.name);document.getElementById('status').innerText=T[lang].msg;fetch('/update',{method:'POST',body:fd}).then(r=>r.text()).then(t=>{if(t.trim()==='OK'){document.getElementById('status').innerText=T[lang].ok;setTimeout(()=>{location.href='/'},8000);}else{document.getElementById('status').innerText=T[lang].fail+' ('+t+')';}}).catch(()=>{document.getElementById('status').innerText=T[lang].fail;});}"
    "</script></body></html>"
  );
  noCache(); server.send(200,"text/html; charset=utf-8",html);
}

void handleUpdateUpload(){
  HTTPUpload& up=server.upload();
  if(up.status==UPLOAD_FILE_START){
    generatorRunning=false; monitorRunning=false;
    otaActive = true;
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if(up.status==UPLOAD_FILE_WRITE){
    Update.write(up.buf, up.currentSize);
  } else if(up.status==UPLOAD_FILE_END){
    Update.end(true);
  }
}

// ============ API Monitor/Gen ============
void handleToggleGen(){ if(server.hasArg("state")) generatorRunning = (server.arg("state")=="1"); otaActive=false; noCache(); server.send(200,"text/plain",generatorRunning?"RUNNING":"STOPPED"); }
void handleGetGen(){
  String out;
  xSemaphoreTake(genBufMutex,portMAX_DELAY);
  for(int i=0;i<GEN_BUFFER_LINES;i++){ int idx=(genIndex+i)%GEN_BUFFER_LINES; if(genBuffer[idx].length()>0) out+=genBuffer[idx]+"\n"; }
  xSemaphoreGive(genBufMutex);
  noCache(); server.send(200,"text/plain",out);
}
void handleClearGen(){ xSemaphoreTake(genBufMutex,portMAX_DELAY); for(int i=0;i<GEN_BUFFER_LINES;i++) genBuffer[i]=""; genIndex=0; xSemaphoreGive(genBufMutex); noCache(); server.send(200,"text/plain","OK"); }
void handleSetMode(){ String m=server.hasArg("m")?server.arg("m"):"monitor"; appMode=(m=="generator")?MODE_GENERATOR:MODE_MONITOR; generatorRunning=false; monitorRunning=false; otaActive=false; noCache(); server.send(200,"text/plain",(appMode==MODE_GENERATOR)?"GENERATOR":"MONITOR"); }
void handleSetMonitor(){ if(server.hasArg("state")) monitorRunning=(server.arg("state")=="1"); otaActive=false; noCache(); server.send(200,"text/plain",monitorRunning?"RUNNING":"PAUSED"); }
void handleGetNMEA(){
  String out; xSemaphoreTake(nmeaBufMutex,portMAX_DELAY);
  for(int i=0;i<BUFFER_LINES;i++){ int idx=(bufferIndex+i)%BUFFER_LINES; if(nmeaBuffer[idx].length()>0) out+=nmeaBuffer[idx]+"\n"; }
  xSemaphoreGive(nmeaBufMutex);
  noCache(); server.send(200,"text/plain",out);
}
void handleSetBaud(){ noCache(); if(server.hasArg("baud")){ int b=server.arg("baud").toInt(); if(b==4800||b==9600||b==38400||b==115200) startSerial(b); server.send(200,"text/plain","OK"); } else server.send(400,"text/plain","Error"); }
void handleClearNMEA(){ xSemaphoreTake(nmeaBufMutex,portMAX_DELAY); for(int i=0;i<BUFFER_LINES;i++) nmeaBuffer[i]=""; bufferIndex=0; currentLine=""; lineStartMs=0; xSemaphoreGive(nmeaBufMutex); noCache(); server.send(200,"text/plain","OK"); }

int argIndex(){ if(!server.hasArg("i")) return -1; int i=server.arg("i").toInt(); if(i<0||i>=MAX_SLOTS) return -1; return i; }
void handleGenSlotEnable(){ int i=argIndex(); if(i<0){server.send(400,"text/plain","Bad slot");return;} bool en=server.hasArg("en")&&(server.arg("en").toInt()==1); slots[i].enabled=en; server.send(200,"text/plain",en?"1":"0"); }
void handleGenSlotSensor(){ int i=argIndex(); if(i<0){server.send(400,"text/plain","Bad slot");return;} if(server.hasArg("sensor")){ slots[i].sensor=server.arg("sensor"); if(slots[i].sensor=="CUSTOM") slots[i].sentence="CUSTOM"; } server.send(200,"text/plain",slots[i].sensor); }
void handleGenSlotSentence(){ int i=argIndex(); if(i<0){server.send(400,"text/plain","Bad slot");return;} if(server.hasArg("sentence")) slots[i].sentence=server.arg("sentence"); server.send(200,"text/plain",slots[i].sentence); }
void handleGenSlotText_POST(){ int i=-1; if(server.hasArg("i")) i=server.arg("i").toInt(); if(i<0||i>=MAX_SLOTS){server.send(400,"text/plain","Bad slot");return;} String incoming=server.hasArg("text")?server.arg("text"):""; slots[i].text=incoming; server.send(200,"text/plain",incoming); }
void handleGenSlotText_GET(){ int i=argIndex(); if(i<0){server.send(400,"text/plain","Bad slot");return;} String incoming=server.hasArg("text")?server.arg("text"):""; slots[i].text=incoming; server.send(200,"text/plain",incoming); }
void handleGenSlotTemplate(){ int i=argIndex(); if(i<0){server.send(400,"text/plain","Bad slot");return;} String t; if(slots[i].sensor=="CUSTOM"||slots[i].sentence=="CUSTOM"){ t=slots[i].text.length()?slots[i].text:"$GPCUS,FIELD1,FIELD2*00"; if(t.startsWith("$")||t.startsWith("!")){ int star=t.indexOf('*'); String payload=(star>=0)?t.substring(1,star):t.substring(1); t=String(t[0])+payload+"*"+nmeaChecksum(payload);} else { String up=t; up.toUpperCase(); char ch=(up.startsWith("AIVDM")||up.startsWith("AIVDO"))?'!':'$'; String payload=t; t=String(ch)+payload+"*"+nmeaChecksum(payload);} } else { t=generateSentence(slots[i].sensor,slots[i].sentence); } slots[i].text=t; server.send(200,"text/plain",t); }
void handleGenSlotInterval(){ int i=argIndex(); if(i<0){server.send(400,"text/plain","Bad slot");return;} if(!server.hasArg("ms")){server.send(400,"text/plain","Missing ms");return;} long ms=server.arg("ms").toInt(); if(ms<50) ms=50; slotInterval[i]=(unsigned long)ms; server.send(200,"text/plain",String(slotInterval[i])); }
void handleGetStatus(){
  String json="{";
  json += "\"mode\":\""+String(appMode==MODE_GENERATOR?"generator":"monitor")+"\","; // para front
  json += "\"baud\":"+String(currentBaud)+",";
  json += "\"genRunning\":"; json += (generatorRunning?"true":"false"); json += ",";
  json += "\"monRunning\":"; json += (monitorRunning?"true":"false");
  json += "}";
  noCache(); server.send(200,"application/json",json);
}

// ===================== UI (OLED) =====================
// Fuentes U8g2
static const uint8_t *FONT_TITLE     = u8g2_font_8x13B_tf; // splash título
static const uint8_t *FONT_SUB       = u8g2_font_7x13_tf;  // splash sub
static const uint8_t *FONT_HDR_SMALL = u8g2_font_5x8_mf;   // encabezado compacto
static const uint8_t *FONT_LIST      = u8g2_font_5x8_mf;   // lista sensores

void drawCentered(const char* text, int y, const uint8_t* font){
  u8g2.setFont(font);
  int w = u8g2.getStrWidth(text);
  int x = (128 - w)/2;
  if(x<0) x = 0;
  u8g2.drawStr(x, y, text);
}
void drawRight(const char* text, int y, const uint8_t* font, int xRight=127){
  u8g2.setFont(font);
  int w = u8g2.getStrWidth(text);
  int x = xRight - w;
  if(x<0) x=0;
  u8g2.drawStr(x, y, text);
}

// Splash centrado + versión (SOLO aquí)
void drawSplash(uint32_t now){
  uint32_t elapsed = now - bootStartMs;
  if(elapsed > SPLASH_MS) elapsed = SPLASH_MS;
  float p = (float)elapsed / (float)SPLASH_MS; // 0..1

  u8g2.clearBuffer();

  const int panelW = 116;
  const int panelH = 44;
  const int px = (128 - panelW) / 2;
  const int py = (64  - panelH) / 2;

  u8g2.drawRFrame(px, py, panelW, panelH, 4);

  drawCentered("NMEA Link", py + 16, FONT_TITLE);
  drawCentered("Themys SA", py + 28, FONT_SUB);

  const int bh = 8;
  const int bottomPad = 2;
  const int bx = px + 8;
  const int bw = panelW - 16;
  const int by = py + panelH - bottomPad - bh;

  u8g2.drawRFrame(bx, by, bw, bh, 2);
  int fill = (int)((bw-2) * p);
  if(fill<0) fill=0;
  u8g2.drawBox(bx+1, by+1, fill, bh-2);

  // Versión chiquita abajo a la derecha (solo en splash)
  drawRight(FW_VERSION, 63, FONT_HDR_SMALL);

  u8g2.sendBuffer();
}

inline bool recentTs(uint32_t ts, uint32_t windowMs){
  if (ts==0) return false;
  return (millis() - ts) <= windowMs;
}

// Header compacto
void drawHeader(){
  const char* modeCode = (appMode==MODE_GENERATOR) ? "GEN" : "MON";
  char left[24];  snprintf(left,  sizeof(left),  "Mode: %s", modeCode);
  char right[24]; snprintf(right, sizeof(right), "Baud: %d", currentBaud);

  const int minSep = 6;

  u8g2.setFont(FONT_HDR_SMALL);
  int wL = u8g2.getStrWidth(left);
  int wR = u8g2.getStrWidth(right);

  if (wL + wR + minSep <= 128) {
    u8g2.drawStr(0, 9, left);
    drawRight(right, 9, FONT_HDR_SMALL);
    u8g2.drawHLine(0, 12, 128);
    return;
  }

  // Ultra compacto
  const char* leftS  = (appMode==MODE_GENERATOR) ? "M: GEN" : "M: MON";
  char rightS[16]; snprintf(rightS, sizeof(rightS), "B: %d", currentBaud);
  u8g2.drawStr(0, 9, leftS);
  drawRight(rightS, 9, FONT_HDR_SMALL);
  u8g2.drawHLine(0, 12, 128);
}

void drawStatus(){
  u8g2.clearBuffer();

  if(otaActive){
    drawCentered("OTA UPDATE", 24, FONT_TITLE);
    drawCentered("DO NOT POWER OFF", 40, FONT_SUB);
    // (SIN versión aquí)
    u8g2.sendBuffer();
    return;
  }

  // Header
  drawHeader();

  // Zona de sensores
  const int topY    = 18;
  const int bottomY = 60;  // 63 reservado para estado RUN/PAUSE
  const int colAX   = 2;
  const int colBX   = 66;

  const char* active[12];
  int nActive = 0;

  if(appMode==MODE_GENERATOR){
    for(int i=0;i<SENSOR_COUNT;i++)
      if(recentTs(sensors[i].lastGenMs, OLED_IDLE_WINDOW_MS))
        active[nActive++] = sensors[i].name;
  } else {
    for(int i=0;i<SENSOR_COUNT;i++)
      if(recentTs(sensors[i].lastSeenMs, OLED_IDLE_WINDOW_MS))
        active[nActive++] = sensors[i].name;
  }

  u8g2.setFont(FONT_LIST);
  if(nActive > 0){
    const int rowsPerCol = (nActive + 1) / 2;  // ceil(n/2)
    const int lineH = 8;
    const int blockH = (rowsPerCol>1 ? (rowsPerCol-1)*lineH : 0);
    int startY = topY + ((bottomY - topY) - blockH)/2;
    if(startY < topY) startY = topY;

    for(int i=0;i<rowsPerCol && i<nActive;i++){
      int y = startY + i*lineH;
      u8g2.drawStr(colAX, y, active[i]);
    }
    for(int i=rowsPerCol, r=0; i<nActive; ++i, ++r){
      int y = startY + r*lineH;
      u8g2.drawStr(colBX, y, active[i]);
    }
  }

  // Estado RUN/PAUSE abajo a la derecha
  const bool isRun = (appMode==MODE_MONITOR) ? monitorRunning : generatorRunning;
  drawRight(isRun ? "RUN" : "PAUSE", 63, FONT_LIST);

  // (SIN versión aquí)
  u8g2.sendBuffer();
}

// ===================== Tasks =====================
void TaskNet(void*){
  for(;;){
    dnsServer.processNextRequest();
    server.handleClient();
    vTaskDelay(1);
  }
}

void TaskNMEA(void*){
  for(;;){
    if(appMode==MODE_MONITOR && monitorRunning){
      xSemaphoreTake(serialMutex,portMAX_DELAY);

      // Timeout de línea rota
      if(currentLine.length()>0 && (millis() - lineStartMs) > LINE_TIMEOUT_MS){
        currentLine = "";
        lineStartMs = 0;
      }

      while(NMEA_Serial.available()){
        char c=(char)NMEA_Serial.read();
        if(lineStartMs==0) lineStartMs = millis();

        if(c=='\n' || c=='\r'){
          if(currentLine.length()==0) continue;

          xSemaphoreGive(serialMutex);

          String raw=currentLine;
          currentLine="";
          lineStartMs=0;
          raw.trim();

          if(raw.length()==0){ xSemaphoreTake(serialMutex,portMAX_DELAY); continue; }

          // Parseo TagBlock / UdPbC
          String sentence, meta;
          bool hadTag=false, hadUdPbC=false;
          bool ok = parseNMEALine(raw, sentence, meta, hadTag, hadUdPbC);

          String effective = ok ? sentence : raw;   // si no se pudo parsear, seguimos con la cruda
          bool valid = processNMEA(effective);

          flashLed(valid?pixels.Color(0,255,0):pixels.Color(255,0,0));

          String type=detectSentenceType(effective);
          if(valid && type!="OTROS"){ stampSeen(type); }

          String formatted="["+type+"] "+effective;
          if(hadTag || hadUdPbC){
            if(meta.length()==0){
              meta = String(hadUdPbC ? "UdPbC" : "");
            }
            formatted += "  ⟨" + meta + "⟩";
          }

          xSemaphoreTake(nmeaBufMutex,portMAX_DELAY);
          bufferIndex=(bufferIndex+1)%BUFFER_LINES;
          nmeaBuffer[bufferIndex]=formatted;
          xSemaphoreGive(nmeaBufMutex);

          if(valid) sendUDP(effective);

          xSemaphoreTake(serialMutex,portMAX_DELAY);
        }
        else if(c>=32 && c<=126){
          if(currentLine.length() < MAX_LINE_LEN){
            currentLine+=c;
          } else {
            currentLine="";
            lineStartMs=0;
          }
        }
      }
      xSemaphoreGive(serialMutex);
    }

    // GENERATOR
    if(appMode==MODE_GENERATOR && generatorRunning){
      unsigned long now=millis();
      for(int i=0;i<MAX_SLOTS;i++){
        if(!slots[i].enabled) continue;
        if(now-lastSentMs[i] >= slotInterval[i]){
          lastSentMs[i]=now;
          String out = slots[i].text.length()?slots[i].text:generateSentence(slots[i].sensor,slots[i].sentence);
          if(out.length()==0) continue;

          stampGen(slots[i].sensor);

          xSemaphoreTake(serialMutex,portMAX_DELAY);
          NMEA_Serial.println(out);
          xSemaphoreGive(serialMutex);
          sendUDP(out);
          pushGen(out);
          flashLed(pixels.Color(0,0,255)); // TX azul
        }
      }
    }

    updateLed();
    vTaskDelay(1);
  }
}

void TaskUI(void*){
  // Splash
  bootStartMs = millis();
  while(millis() - bootStartMs < SPLASH_MS){
    drawSplash(millis());
    vTaskDelay(40);
  }
  for(;;){
    drawStatus();
    vTaskDelay(120);
  }
}

// ============ Setup/Loop ============
void setup(){
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  esp_log_level_set("*", ESP_LOG_NONE);

  SPI.end();
  SPI.begin(OLED_SCK, -1 /*MISO*/, OLED_MOSI, OLED_CS);
  u8g2.begin();

  pixels.begin(); pixels.show();

  nmeaBufMutex=xSemaphoreCreateMutex();
  genBufMutex =xSemaphoreCreateMutex();
  serialMutex =xSemaphoreCreateMutex();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();

  dnsServer.start(DNS_PORT, "*", apIP);
  MDNS.begin("nmeareader"); MDNS.addService("http","tcp",80);

  flashLed(pixels.Color(0,255,255)); // boot
  startSerial(currentBaud);

  udpAddress = apIP; udpAddress[3]=255; // broadcast 192.168.4.255

  // Captive helpers
  server.on("/generate_204", handleCaptive);
  server.on("/gen_204",      handleCaptive);
  server.on("/hotspot-detect.html", handleCaptive);
  server.on("/ncsi.txt",     handle204);
  server.on("/favicon.ico",  handle204);
  server.on("/robots.txt",   handle204);
  server.on("/wpad.dat",     handle204);

  // Páginas
  server.on("/",          handleMenu);
  server.on("/monitor",   handleMonitor);
  server.on("/generator", handleGenerator);
  server.on("/update",    HTTP_GET, handleUpdatePage);
  server.on("/update",    HTTP_POST,
    [](){ noCache(); server.sendHeader("Connection","close"); server.send(200,"text/plain",Update.hasError()?"FAIL":"OK"); delay(800); ESP.restart(); },
    handleUpdateUpload
  );

  // API comunes
  server.on("/getnmea",   handleGetNMEA);
  server.on("/setbaud",   handleSetBaud);
  server.on("/setmode",   handleSetMode);
  server.on("/setmonitor",handleSetMonitor);
  server.on("/clearnmea", handleClearNMEA);

  // API generator
  server.on("/togglegen",        handleToggleGen);
  server.on("/getgen",           handleGetGen);
  server.on("/cleargen",         handleClearGen);
  server.on("/getstatus",        handleGetStatus);
  server.on("/gen_slot_enable",  handleGenSlotEnable);
  server.on("/gen_slot_sensor",  handleGenSlotSensor);
  server.on("/gen_slot_sentence",handleGenSlotSentence);
  server.on("/gen_slot_template",handleGenSlotTemplate);
  server.on("/gen_slot_text",    HTTP_POST, handleGenSlotText_POST);
  server.on("/gen_slot_text",    HTTP_GET,  handleGenSlotText_GET);
  server.on("/gen_slot_interval",handleGenSlotInterval);

  // NotFound → redirige a menú
  server.onNotFound([](){
    noCache();
    server.sendHeader("Location", String("http://")+WiFi.softAPIP().toString()+"/", true);
    server.send(302,"text/plain","");
  });

  server.begin();

  // Logs de arranque
  Serial.println("\n🚀 NMEA Link - boot");
  Serial.printf("📶 AP SSID: %s\n", AP_SSID);
  Serial.print( "📄 IP (AP): " ); Serial.println(apIP.toString());
  Serial.print( "🌐 UDP broadcast: " ); Serial.print(udpAddress.toString()); Serial.print(":"); Serial.println(udpPort);
  Serial.printf("🔧 UART RX=%d  TX=%d  baud=%d\n", RX_PIN, TX_PIN, currentBaud);
  Serial.println("✅ HTTP server + DNS (captive) listos");
  Serial.println("🧵 Tasks: Net(core0) + NMEA(core1) + UI(core0)");

  xTaskCreatePinnedToCore(TaskNet,  "TaskNet",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskNMEA, "TaskNMEA", 6144, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskUI,   "TaskUI",   4096, NULL, 1, NULL, 0);
}

void loop(){ /* vacío (todo corre en tasks) */ }
