// Build-system placeholder: prints the firmware version. Replaced by the
// bridge application once the display and VESC libraries are in place.
#include <Arduino.h>
#include "build_version.h"

void setup() {
    Serial.begin(115200);
    Serial.printf("Kukirin bridge %s (%s %s)\n", FW_VERSION, BUILD_DATE, BUILD_TIME);
}

void loop() {}
