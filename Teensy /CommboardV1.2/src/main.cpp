#include <DynamixelController.hpp>
void setup() {
    Serial.begin(9600);
    init();
}

void loop() {
    requestHandler();
}
