#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    lcd.init();
    lcd.begin(16, 2);
    lcd.backlight();

    lcd.clear();
    lcd.setCursor(6,0);
    lcd.print("GAS"); 
    lcd.setCursor(3,1);
    lcd.print("DETECTED!"); 
}

void loop() {

}