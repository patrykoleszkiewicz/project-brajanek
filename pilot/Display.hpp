#define LCD_ADDRESS     0x3F
#define LCD_WIDTH       16
#define LCD_HEIGHT      2
#define LCD_INTERVAL_US 300000

#include <LCD_I2C.h>

class Display
{
  public:
    Display()
        : lcd(LCD_ADDRESS, LCD_WIDTH, LCD_HEIGHT)
    {
    }

    void init()
    {
        lcd.begin();
        lcd.backlight();
    }

    void update()
    {
        unsigned long now = micros();
        if(now - lcdTimer > LCD_INTERVAL_US)
        {
            lcdTimer = now;
            lcd.clear();

            lcd.setCursor(0, 0);
            lcd.print(receivedData.sensorLeft);

            lcd.setCursor(5, 0);
            lcd.print(receivedData.sensorRight);

            lcd.setCursor(LCD_WIDTH - 4, 0);
            lcd.print("100%");

            if(controls.getHandbrake())
            {
                lcd.setCursor(0, 1);
                lcd.print(F("(!)"));
            }

            if(sentData.reverse)
            {
                lcd.setCursor(LCD_WIDTH - 1, 1);
                lcd.print(F("R"));
            }
        }
    }

  private:
    LCD_I2C lcd;
    long lcdTimer;
} display;
