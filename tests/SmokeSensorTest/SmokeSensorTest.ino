#define SMOKE_PIN A0

void setup() {
    Serial.begin(9600);

    pinMode(SMOKE_PIN, INPUT);
}

void loop() {
    Serial.println(analogRead(SMOKE_PIN));
    delay(1000);
}