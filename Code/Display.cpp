#include "Display.h"

void initializeDisplay() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("GestAir");
  lcd.setCursor(0,1); lcd.print("Booting...");
}
