#include "New/Config.hpp"
#include "New/App.hpp"

// =============================================================================
//  main.cpp
//  Translation unit that owns every shared global, the geiger ISRs, and the
//  Arduino setup()/loop() entry points.
// =============================================================================

// Shared globals declared in Config.hpp.
Preferences prefs;
uint32_t    runId = 0;
volatile uint32_t geigerCh1Pulses = 0;
volatile uint32_t geigerCh2Pulses = 0;

// Log-file header — must match the field order produced by App::buildCsvRow_
// (pipe-separated "key = value" pairs, not classic comma CSV).
const char* CSV_HEADER =
  "Run_ID|Now_Ms|Accsloration_mg(x,y,z)|Gyro_mrad_s(x,y,z)|Mag_mgauss(x,y,z)|"
  "Pressure_hPa|Temperature_cC|Geiger_1|Geiger_2|LoRa_Temp_cC|LoRa_Volt_mV|"
  "LoRa_Retries|LoRa_Drops|Storage_Fails";

// Geiger ISRs — called from RISING-edge attachInterrupt() in App::begin().
void IRAM_ATTR geigerCh1Isr() { geigerCh1Pulses++; }
void IRAM_ATTR geigerCh2Isr() { geigerCh2Pulses++; }

static App app;

void setup() { app.begin(); }
void loop()  { app.tick();  }
