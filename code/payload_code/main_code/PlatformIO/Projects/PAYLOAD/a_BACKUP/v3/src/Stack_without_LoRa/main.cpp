#include "Stack_without_LoRa/Config.hpp"
#include "Stack_without_LoRa/App.hpp"

// =============================================================================
//  main.cpp  -  Stack_without_LoRa
//  Owns the shared globals and the Arduino entry points.
// =============================================================================

Preferences prefs;
uint32_t    runId = 0;

static App app;

void setup() { app.begin(); }
void loop()  { app.tick();  }
