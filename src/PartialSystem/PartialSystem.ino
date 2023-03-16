/* LIBRARIES */
#include <LiquidCrystal_I2C.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

/* PIN CONFIGURATION */
#define LOAD_CELL_DOUT_PIN 2
#define LOAD_CELL_SCK_PIN 3
#define BUZZER_PIN 4
#define LOW_LED_PIN 5
#define MID_LED_PIN 6
#define HIGH_LED_PIN 7
#define GAS_SENSOR_PIN A0
/* LCD PINS */
/* SDA -> A4 */
/* SCL -> A5 */

/* CONSTANTS */
const int GAS_ATMOSPHERE_THRESHOLD = 100;
const float LOW_WEIGHT_THRESHOLD = 0.1; // 10%
const float MID_WEIGHT_THRESHOLD = 0.5; // 50% 
const float HIGH_WEIGHT_THRESHOLD = 0.8; // 80%
const int GAS_CONTAINER_WEIGHT = 5;
const int GAS_FULL_CAPACITY = 30;

float raw_gas_weight = 0;
float computed_gas_weight = 0;
int gas_level = 0;
int gas_atmosphere = 0;

// LCD OBJECT
LiquidCrystal_I2C lcd(0x27, 16, 2);

byte smile[8] = {
    0b00000,
    0b00000,
    0b01010,
    0b00000,
    0b10001,
    0b01110,
    0b00000,
    0b00000
};

byte heart[8] = {
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b11111,
    0b01110,
    0b00100,
    0b00000
};

// LOAD CELL VARIABLES
HX711_ADC LoadCell(LOAD_CELL_DOUT_PIN, LOAD_CELL_SCK_PIN);
const int calVal_eepromAdress = 0;
unsigned long t = 0;

void setup() {
    Serial.begin(9600);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LOW_LED_PIN, OUTPUT);
    pinMode(MID_LED_PIN, OUTPUT);
    pinMode(HIGH_LED_PIN, OUTPUT);

    pinMode(GAS_SENSOR_PIN, INPUT);

    // LCD Initialization
    lcd.init();
    lcd.begin(16, 2);
    lcd.backlight();
    // creates the emojis
    lcd.createChar(1, smile);
    lcd.createChar(2, heart);

    initializeLoadCell();
}

void loop() {
    // read gas sensor
    gas_atmosphere = analogRead(GAS_SENSOR_PIN);

    Serial.print("Gas Atmosphere: ");
    Serial.println(gas_atmosphere);

    // if gas detected
    if (gas_atmosphere > GAS_ATMOSPHERE_THRESHOLD) {
        // alarms every actuators if gas is detected 
        alarmGasDetected();
        return; // restricts continuing the actions below
    }

    // else if gas is not detected 
    // read weight
    readWeight();

    // compute gas level
    computeGasLevel();

    Serial.print("Computed Gas Weight: ");
    Serial.println(computed_gas_weight);
    
    // show weight on LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("GAS LEVEL: "); 
    int gasLevelToPrint = computed_gas_weight * 100;
    lcd.setCursor(12,0);
    lcd.print(gasLevelToPrint); 
    lcd.setCursor(15,0);
    lcd.print("%"); 
    lcd.setCursor(0,1);
    lcd.print("NO LEAKING GAS"); 
    lcd.setCursor(14,1);
    lcd.write(1);
    lcd.write(2);

    // turn off all LED before lighting base on gas level to avoid conflicts
    turnOffAllLED();

    // light up corresponding LED base on gas level
    if (gas_level == 1) digitalWrite(LOW_LED_PIN, HIGH);
    if (gas_level == 2) digitalWrite(MID_LED_PIN, HIGH);
    if (gas_level == 3) digitalWrite(HIGH_LED_PIN, HIGH);

    delay(1000);
}

void computeGasLevel() {
    // computes the remaining gas percentage
    computed_gas_weight = (raw_gas_weight - GAS_CONTAINER_WEIGHT) / GAS_FULL_CAPACITY;

    if (computed_gas_weight >= HIGH_WEIGHT_THRESHOLD) gas_level = 3;
    if (computed_gas_weight < HIGH_WEIGHT_THRESHOLD) gas_level = 2;
    if (computed_gas_weight < MID_WEIGHT_THRESHOLD) gas_level = 1;
}

void alarmGasDetected() {
    // update LCD
    lcd.clear();
    lcd.setCursor(6,0);
    lcd.print("GAS"); 
    lcd.setCursor(3,1);
    lcd.print("DETECTED!"); 

    // alarm buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    // light up all LED
    digitalWrite(LOW_LED_PIN, HIGH);
    digitalWrite(MID_LED_PIN, HIGH);
    digitalWrite(HIGH_LED_PIN, HIGH);
    delay(1000);

    // turns off alarm after an interval of 2 seconds
    // buzzer off 
    digitalWrite(BUZZER_PIN, LOW);
    // lights off all LED
    turnOffAllLED();
    delay(1000);
}

void turnOffAllLED() {
    digitalWrite(LOW_LED_PIN, LOW);
    digitalWrite(MID_LED_PIN, LOW);
    digitalWrite(HIGH_LED_PIN, LOW);
}

void initializeLoadCell() {
    LoadCell.begin();
    LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
    float calibrationValue; // calibration value (see example file "Calibration.ino")
    calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch

    EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

    unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
    boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
    LoadCell.start(stabilizingtime, _tare);
    if (LoadCell.getTareTimeoutFlag()) {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
        while (1);
    }
    else {
        LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
        Serial.println("Startup is complete");
    }
}

void readWeight() {
    static boolean newDataReady = 0;
    const int serialPrintInterval = 0; //increase value to slow down serial print activity

    // check for new data/start next conversion:
    if (LoadCell.update()) newDataReady = true;

    // get smoothed value from the dataset:
    if (newDataReady) {
        if (millis() > t + serialPrintInterval) {
            float i = LoadCell.getData();
            Serial.print("Load_cell output val: ");
            Serial.println(i);
            raw_gas_weight = i;

            newDataReady = 0;
            t = millis();
        }
    }
}