/*
 * This demo is for the ESP32 with a ILI9488 3.5" Capacitive Touch screen.
 *
 * http://skpang.co.uk/catalog/esp32-canbus-board-with-35-touch-lcd-p-1590.html
 *
 * Ensure the follow library are installed first.
 * https://github.com/littlevgl/lv_arduino
 * https://github.com/littlevgl/lv_examples
 *
 * In lv_ex_conf.h change
 * 
 * #define LV_USE_DEMO 0
 * to
 * #define LV_USE_DEMO 1
 * 
 * 
 */

#include <lvgl.h>
#define LV_CONF_INCLUDE_SIMPLE

#include "test_first.h"
#include "lv_tests/test.h"
#include "lv_apps/test_theme.h"
#include "lv_apps/demo/demo.h"

#include <Ticker.h>
#include <TFT_eSPI.h>
#include "Wire.h"

#define TOUCH_I2C_ADD 0x38
#define TOUCH_REG_XL 0x04
#define TOUCH_REG_XH 0x03
#define TOUCH_REG_YL 0x06
#define TOUCH_REG_YH 0x05
#define COLOR_BACKGROUND 0x1084
#define COLOR_TOUCH_POINT TFT_BLUE
#define TOUCH_INT_PIN 35
#define ON  LOW
#define OFF HIGH

int LCD_BL = 12;
int LCD_RST = 16;

int LED_R = 22;
int LED_B = 2;
int LED_G = 4;
int touchX = 0;
int touchY = 0;
int oldTouchX = 0;
int oldTouchY = 0;

#define LVGL_TICK_PERIOD 60

Ticker tick; /* timer for interrupt handler */
TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

int screenWidth = 480;
int screenHeight = 320;

int getTouchInterruptPinValue()
{
  return digitalRead(TOUCH_INT_PIN);
}

int readTouchReg(int reg)
{
  int data = 0;
  Wire.beginTransmission(TOUCH_I2C_ADD);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(TOUCH_I2C_ADD, 1);
  if (Wire.available())
  {
    data = Wire.read();
  }
  return data;
}

int getTouchPointX()
{
  int XL = 0;
  int XH = 0;

  XH = readTouchReg(TOUCH_REG_XH);
  XL = readTouchReg(TOUCH_REG_XL);

  return ((XH & 0x0F) << 8) | XL;
}

int getTouchPointY()
{
  int YL = 0;
  int YH = 0;

  YH = readTouchReg(TOUCH_REG_YH);
  YL = readTouchReg(TOUCH_REG_YL);

  return ((YH & 0x0F) << 8) | YL;
}

void touchInit()
{
  Wire.begin(15, 14);
  Wire.setClock(400000);
  pinMode(35, INPUT);

  Wire.beginTransmission(0x38);
  Wire.write(0xA4);
  Wire.write(0x00); //turn on interrupt
  Wire.endTransmission();
  
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint16_t c;

  tft.startWrite(); /* Start new TFT transaction */
  tft.setAddrWindow(area->x1, area->y1, (area->x2 - area->x1 + 1), (area->y2 - area->y1 + 1)); /* set the working window */
  for (int y = area->y1; y <= area->y2; y++) {
    for (int x = area->x1; x <= area->x2; x++) {
      c = color_p->full;
      tft.writeColor(c, 1);
      color_p++;
    }
  }
  tft.endWrite(); /* terminate TFT transaction */
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{

    uint16_t touchX, touchY; 
    int touch;

    touchX = getTouchPointY();
    touchY = getTouchPointX();
    touchY = 320-touchY;          //Rotate data
    
    if ((touchX != oldTouchX) || (touchY != oldTouchY))
    {
  
      oldTouchY = touchY;
      oldTouchX = touchX;

      data->state = LV_INDEV_STATE_PR; 
 
      data->point.x = touchX;
      data->point.y = touchY;

      Serial.println("X: " + String(touchX));
      Serial.println("Y: " + String(touchY));

  }

    return false; 
}

/* Interrupt driven periodic handler */
static void lv_tick_handler(void)
{

  lv_tick_inc(LVGL_TICK_PERIOD);
}


void printEvent(String Event, lv_event_t event)
{
  
  Serial.print(Event);
  printf(" ");

  switch(event) {
      case LV_EVENT_PRESSED:
          printf("Pressed\n");
          break;

      case LV_EVENT_SHORT_CLICKED:
          printf("Short clicked\n");
          break;

      case LV_EVENT_CLICKED:
          printf("Clicked\n");
          break;

      case LV_EVENT_LONG_PRESSED:
          printf("Long press\n");
          break;

      case LV_EVENT_LONG_PRESSED_REPEAT:
          printf("Long press repeat\n");
          break;

      case LV_EVENT_RELEASED:
          printf("Released\n");
          break;
  }
}

void loop() {

  lv_task_handler(); /* let the GUI do its work */
  delay(5);
}

void setup() {

    pinMode(LCD_BL,OUTPUT);
    pinMode(LED_R,OUTPUT);
    pinMode(LED_G,OUTPUT);
    pinMode(LED_B,OUTPUT);
    pinMode(LCD_RST,OUTPUT);

    digitalWrite(LCD_RST,HIGH);
    
    digitalWrite(LED_B, OFF);
    digitalWrite(LED_G, OFF);
    digitalWrite(LED_R, OFF);   
    digitalWrite(LED_R, ON);
    digitalWrite(LCD_BL,HIGH);  
    delay(200); 
    digitalWrite(LED_R, OFF);
    digitalWrite(LCD_BL,LOW);
    digitalWrite(LCD_RST,LOW);
    delay(50);
    digitalWrite(LCD_RST,HIGH);
    
    digitalWrite(LED_G, ON);
    delay(200); 
    digitalWrite(LED_G, OFF);
    
    digitalWrite(LED_B, ON);
    delay(200); 
    digitalWrite(LED_B, OFF);
    digitalWrite(LCD_BL,HIGH);  
    Serial.begin(115200);
    Serial.println("FT6236 demo");
 
    touchInit();    // Init touch panel

    lv_init();      // Init LittlevGL

    tft.begin();    // Init LCD
    tft.setRotation(1);

    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

    /*Initialize the display*/
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);
  
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);             /*Descriptor of a input device driver*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
    indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
    lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/

    demo_create();    // Start demo located at /lv_examples/lv_apps/demo/demo.c

    //lv_ex_btn_1();  // Simple button demo

    tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);
}


static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_CLICKED) {
        printf("Clicked\n");
    }
    else if(event == LV_EVENT_VALUE_CHANGED) {
        printf("Toggled\n");
    }
}

void lv_ex_btn_1(void)
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_set_event_cb(btn1, event_handler);
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 0, -40);

    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Button");

    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_set_event_cb(btn2, event_handler);
    lv_obj_align(btn2, NULL, LV_ALIGN_CENTER, 0, 40);
    lv_btn_set_toggle(btn2, true);
    lv_btn_toggle(btn2);
    lv_btn_set_fit2(btn2, LV_FIT_NONE, LV_FIT_TIGHT);

    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label, "Toggled");
}
