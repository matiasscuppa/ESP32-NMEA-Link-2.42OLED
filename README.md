# ESP32 NMEA Link + OLED Display + OTA Update

Access Point + captive portal that serves a **web UI** with two modes:

- **NMEA Monitor** (UART RX=16, category filters, UDP forward).
- **NMEA Generator** (UART TX=17 + UDP, up to **4 slots** with editable templates and **automatic checksum**).

Runs on **both cores**: networking/HTTP on Core 0, NMEA/LED on Core 1 for a smooth UI.

---

## 🔄 What’s new in this update (vs original)

- **OLED polish**
  - Centered **splash** with progress bar and extra spacing between “Themys SA” and the bar (no “Initializing…” text).
  - Compact, always-oneline header: **`Mode: MON|GEN`** and **`Baud: N`** with a space after `:`; auto-shortens if it ever gets tight.
  - **Dynamic two-column sensor list** with **vertical centering** (no “corner clustering” when 3–4 sensors); list starts **empty** on boot.
  - **RUN / PAUSE** indicator at bottom-right in both modes.
  - **CUSTOM** now appears on the OLED when generating custom frames.

- **UX & stability**
  - **Menu stops everything**: switching to `/` halts Monitor & Generator cleanly.
  - Monitor/Generator do **not** auto-pause unexpectedly (state is honored until user changes it).
  - **115200 layout fixed**: header never overlaps or truncates.
  - Sensor “presence” persists visually for **8 s** for a more natural feel.

- **Web UI refinements**
  - Dark, mobile-friendly UI; System UI fonts + monospace for consoles.
  - Generator editor hides `*HH` and **recomputes checksum live**.
  - Full-width **Back to NMEA Monitor** button; clearer controls & labels.

- **LED feedback & logging**
  - NeoPixel (GPIO 48): **boot cyan**, **valid RX green**, **invalid RX red**, **TX blue**.
  - **Quiet logs**: only boot information on Serial (no frame/UI spam).

- **Network**
  - Valid monitor frames and all generator frames go out via **UDP 10110** to AP broadcast.

---

## 🛠️ Features

- **Wi-Fi AP**: `SSID: NMEA_Link`, `password: 12345678`, with **captive portal** (auto-opens the web UI).
- **Dark, mobile-friendly UI** (monospace).
- **Monitor**
  - UART **RX=16** (baud: **4800 / 9600 / 38400 / 115200**).
  - Category filters (GPS, AIS, WEATHER, HEADING, SOUNDER, VELOCITY, RADAR, TRANSDUCER, OTHER).
  - **Start/Pause**, **Clear**, polling speed (25/50/75/100%).
  - Valid frames forwarded via **UDP 10110** (broadcast).
- **Generator**
  - UART **TX=17** + **UDP 10110**.
  - Up to **4 simultaneous slots**, each with:
    - **Sensor** + **sentence** (NMEA 0183) or **CUSTOM**.
    - **Editable template** (editor hides `*HH`; checksum recalculated live).
    - **Per-slot interval**: 0.1 s / 0.5 s / 1 s / 2 s.
    - Slot enable/disable.
  - **Start/Pause**, **Clear output**, and full-width **Back to NMEA Monitor**.

---

## 📡 Network / Access

1. Power the ESP32 and connect to **`NMEA_Link`** (password `12345678`).
2. The **captive portal** should open the UI; if not, go to `http://192.168.4.1/`.
3. mDNS: `http://nmeareader.local` (availability varies by OS).

**UDP**: device emits on **10110** to the AP broadcast (**x.x.x.255**). Works with OpenPlotter/Signal K/other NMEA 0183 apps.

---

## 🔌 Pins / Hardware

- **UART RX** (Monitor): **GPIO 16**  
- **UART TX** (Generator): **GPIO 17**  
- **NeoPixel** (1 LED): **GPIO 48**  

**OLED (SSD1309 128×64 SPI)**  
SCK=36 · MOSI=35 · CS=37 · DC=38 · RST=39  
(U8g2: `U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI`)

Tested with PlatformIO on **ESP32-S3-DevKitC-1**.

---

## 🚀 Quick start

### Monitor
1. Select **baudrate** (active one is highlighted).
2. Press **Start** to begin viewing incoming frames.
3. Use **category filters** and **Clear**.
4. Adjust **polling speed** if needed.
5. OLED shows `Mode: MON  |  Baud: N`, dynamic two-column sensor list, and **RUN/PAUSE**.

### Generator
1. For each **slot**: enable it, pick **sensor** and **sentence** (or **CUSTOM**).
2. Edit the **template** (without `*HH`); checksum updates automatically.
3. Set the slot **interval** (0.1/0.5/1/2 s).
4. Select **baudrate**, press **Start** to transmit. UDP 10110 broadcast is enabled.
5. **Back to NMEA Monitor** pauses the generator automatically.
6. OLED shows `Mode: GEN  |  Baud: N`, sensors generated recently (includes **CUSTOM**), and **RUN/PAUSE**.

---

### 📑 Supported sentences (NMEA 0183)

**GPS**: GLL, RMC, VTG, GGA, GSA, GSV, DTM, ZDA, GNS, GST, GBS, GRS, RMB, RTE, BOD, XTE  
**WEATHER**: MWD, MWV, VWR, VWT, MTW, MTA, MMB, MHU, MDA  
**HEADING**: HDG, HDT, HDM, THS, ROT, RSA  
**SOUNDER**: DBT, DPT, DBK, DBS  
**VELOCITY**: VHW, VLW, VBW  
**RADAR**: TLL, TTM, TLB, OSD  
**TRANSDUCER**: XDR  
**AIS**: AIVDM, AIVDO  
**CUSTOM**: free-form (editor with auto checksum)

---

### 🧭 Integration

OpenPlotter / Signal K / navigation software: listen to **UDP 10110** on the Wi-Fi interface connected to **NMEA_Link**.

---

### 🔒 Notes / Limitations

- UI is served over HTTP (not HTTPS) for simplicity on the ESP32.
- Captive portal behavior depends on the client OS (might not always auto-open).
- mDNS support varies by OS.

---

### 🗺️ Roadmap

- Persist configuration (NVS).
- Export/Import templates.
- Static assets (minified/gzip).
- More locales in language selector.

---

### ✨ Credits

Firmware & UI by **Matías Scuppa** — by **Themys**.

---

### 📝 License

MIT — use and modify freely; PRs welcome.
