// Core Overrides must come before Core is included

#include "espcore.h"
#include "firecracker.h"

void setup() {
    ecc.begin();
    fcc.begin();
}

void loop() {
    ecc.loop();
    fcc.loop();
}

