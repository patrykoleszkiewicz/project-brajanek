#define LCD_ADDRESS     0x3F
#define LCD_WIDTH       16
#define LCD_HEIGHT      2
#define LCD_INTERVAL_US 200000

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
            for(int line = 0; line < LCD_HEIGHT; ++line)
            {
                lcd.setCursor(0, line);
                lcd.print(lines[line]);
            }
        }
    }

  private:
    LCD_I2C lcd;
    long lcdTimer;

    String lines[LCD_HEIGHT];
};
