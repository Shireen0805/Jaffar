//AnyScape Object Module

//1. Test removing rend 0 from scan
//2. Test resetting of bus
//3. Use WDT for fault recovery
//4. Use Flash to store image and type after reception in order to save state.


#define ADAFRUIT_TFT 1
//#define BODMER_TFT 1

#define PNGDEC_DEBUG false
#define PNGDEC_DEBUG_SERIAL if(PNGDEC_DEBUG) Serial

#define I2C_DEBUG false
#define I2C_DEBUG_SERIAL if(I2C_DEBUG) Serial

#include <Arduino.h>
#include <Wire.h>

#include <SPI.h>

#include <stdio.h>

#include <PNGdec.h>
#include <img/cubie1.h>
#include <img/cubie2.h>
#include <img/cubie3.h>
#include <img/cubie4.h>
#include <img/cubie5.h>
#include <img/cubie6.h>
#include <img/cubie7.h>
#include <img/cubie8.h>
#include <img/cubie9.h>

#include <FastLED.h>
#define NUM_LEDS 9

CRGBArray<NUM_LEDS> leds;

#ifdef BODMER_TFT
  #include <TFT_eSPI.h> //Remember to set User_setup!
  #include "tftEspiDebug.h"
  #define TFT_MISO 16
  TFT_eSPI tft = TFT_eSPI();
#endif
#ifdef ADAFRUIT_TFT
  #include <Adafruit_GFX.h> 
  #include <Fonts/FreeSans18pt7b.h>
  #include <Adafruit_ST7789.h>
  #define TFT_MISO -1
  #define TFT_MOSI 3
  #define TFT_SCLK 2
  #define TFT_CS 5  // Chip select control pin
  #define TFT_DC 1  // Data Command control pin
  #define TFT_RST 0 // Reset pin (could connect to RST pin) 
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS,  TFT_DC, TFT_RST);
#endif

#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))

#define TFT_WIDTH 240
#define TFT_HEIGHT 135

#define TFT_BL 4
#define PWR_ON 22
#define DSP_BTN_1 6
#define DSP_BTN_2 7
#define RedLED 25
#define BatVol 26

#define LED_PIN 25u
#define BTN_PIN 7

#define BUTTON1 29
#define BUTTON2 28

#define SLAVE_ADDRESS 0x0C

#define IMG_BUFFER_SIZE (1024 * 60) //16k for incomming images
#define SCREEN_BUFFER_SIZE (TFT_WIDTH * TFT_HEIGHT) //Full screen buffer for double buffering

const uint8_t STATE_INIT = 0;
const uint8_t STATE_IDLE = 1;
const uint8_t STATE_RECEIVING = 2;
const uint8_t STATE_AVAILABLE = 3;
const uint8_t STATE_WRITING = 4;
const uint8_t STATE_ON_DISPLAY = 5;

unsigned long ms;

bool received = false;
int receivedCount = 0;
int type = 0;

uint32_t i2cCommand;

uint32_t imageBufferPointer = 0;
uint16_t imageLength;
uint8_t imageBuffer[IMG_BUFFER_SIZE];
uint16_t screenBuffer[SCREEN_BUFFER_SIZE];

char hexMap[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

const uint8_t imgMapLen = 9;
const uint8_t * imgMap[] = {cubie2, cubie2, cubie3, cubie4, cubie5, cubie6, cubie7, cubie8, cubie9};
const uint32_t sizeMap[] = {cubie2_size, cubie2_size, cubie3_size, cubie4_size, cubie5_size, cubie6_size, cubie7_size, cubie8_size, cubie9_size};
uint8_t displayedCubie = 100;

int score = 4;
bool wasLastAnswerCorrect = false;

PNG png;
#define MAX_IMAGE_WIDTH 240
uint16_t lineBuffer[MAX_IMAGE_WIDTH];

enum STATE {
  INIT,
  WAITING_FOR_ANSWER,
  SHOWING_ANSWER_RESULT,
  SHOWING_CURRENT_SCORE,
  FINISHED
};

void setState(STATE newState);
void receiveEvent(int bytes);
void onI2CRequest();
void initTFT();
int drawPNG(uint8_t * buffer, uint32_t len, bool fromFlash);
void displayScore(uint8_t score);

#define MI2C Wire

struct CARD {
  uint8_t correct;
  uint8_t audioOK;
  uint8_t audioNOK;
  uint8_t weight;
};

const CARD cards[8] = {
  {1,0,0,1},
  {1,0,0,1},
  {2,0,0,1},
  {1,0,0,1},
  {1,0,0,1},
  {2,0,0,1},
  {1,0,0,1},
  {1,0,0,1}
};

uint8_t numberOfCards = 8;
int cardNumber = 0;
int displayedCard = 0;

STATE state = INIT;
uint32_t stateTime = 0;

void setup() {
  pinMode(PWR_ON, OUTPUT);
  digitalWrite(PWR_ON, 1);

  Serial.begin(115200);
  delay(1000);
  FastLED.addLeds<WS2812B,27,GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.clear();

  Serial.println("Starting A Day Of Cubie v0.1");
  Serial.printf("chip: %d\n", rp2040.getChipID());
  Serial.printf("core version: %s\n", ARDUINO_PICO_VERSION_STR);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(DSP_BTN_1, INPUT_PULLUP);
  pinMode(DSP_BTN_2, INPUT_PULLUP);

  pinMode(BatVol, INPUT);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, 1);
  analogWrite(TFT_BL, 0);

  initTFT();

  for( uint8_t i = 0 ; i < 9 ; i++ ){
    leds[i].setRGB(128,128,128);
    FastLED.show();
    delay(125);
  }

  //drawMi();

  delay(1000);
  for( uint8_t i = 0 ; i < 9 ; i++ ){
    leds[i].setRGB(0,0,0);
  }
  FastLED.show();
  FastLED.clear();
  displayScore(score);
}

#ifdef BODMER_TFT
void initTFT(){
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  analogWrite(2, -1);                   // turn off PWM
  gpio_set_function(2, GPIO_FUNC_SIO);  // revert pin to SW control of GPIO
  analogWrite(3, -1);                   // turn off PWM
  gpio_set_function(3, GPIO_FUNC_SIO);  // revert pin to SW control of GPIO

  SPI.setTX(3);
  SPI.setSCK(2);
  SPI.setRX(16);
  SPI.setCS(17);
  SPI.begin(false);

  tft.init();
  printTFTSetup(tft);

  tft.invertDisplay(true);
  delay(120);
  tft.setRotation(2);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("AnyScape", tft.width() / 2, (tft.height() / 2));

  for (int i = 0; i < 0xff; i++) {
    delay(5);
    analogWrite(TFT_BL, i);
  }
}
#endif

#ifdef ADAFRUIT_TFT
void initTFT(){
  SPI.setTX(3);
  SPI.setSCK(2);
  tft.initSPI(20000000,SPI_MODE0);
  tft.init(135,240);

  tft.invertDisplay(true);
  delay(120);
  tft.setRotation(1);
  tft.setTextSize(1);
  tft.setFont(&FreeSans18pt7b);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(0,40);
  tft.print("A Day-\nOf Cubie");
  

  for (int i = 0; i < 0xff; i++) {
    delay(5);
    analogWrite(TFT_BL, i);
  }
}
#endif

void drawPNGLine(PNGDRAW * pDraw){
  uint8_t xpos = 0;
  uint8_t ypos = 0;
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_LITTLE_ENDIAN, -1);//0xffffffff);

  PNGDEC_DEBUG_SERIAL.printf("Drawing line from png: (line: %d, width: %d):", pDraw->y, pDraw->iWidth);

#ifdef ADAFRUIT_TFT
  int width = pDraw->iWidth;
  int x = 0;
  int offset = 0;

  if(width > TFT_WIDTH){
    offset = (width - TFT_WIDTH) / 2;
    width = TFT_WIDTH;
  }

  for( int x = 0 ; x < width ; x++ ){
    if(x % 32 == 0) PNGDEC_DEBUG_SERIAL.print("\n\t");
    PNGDEC_DEBUG_SERIAL.printf("%04X ", lineBuffer[x + offset]);
    //tft.drawPixel(x, ypos + pDraw->y, lineBuffer[x + offset]);
    
    screenBuffer[((ypos + pDraw->y) * TFT_WIDTH) + x] = lineBuffer[x + offset];
  }
  //tft.drawRGBBitmap(xpos, ypos + pDraw->y, lineBuffer, pDraw->iWidth, 1);
#endif
#ifdef BODMER_TFT
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
#endif
  PNGDEC_DEBUG_SERIAL.print("\n");
  //PNGDEC_DEBUG_SERIAL.print("\tLine finished\n");
}

int drawPNG(uint8_t * buffer, uint32_t len, bool fromFlash){
  memset(screenBuffer, 0, SCREEN_BUFFER_SIZE * 2);
  int rc = fromFlash ? png.openFLASH( buffer, len, drawPNGLine)
                     : png.openRAM( buffer, len, drawPNGLine);
  if(rc == PNG_SUCCESS){

    Serial.printf("Successfully read png image:\n\timage specs: (%d x %d), %d bpp, pixel type: %d\n",
              png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());

#ifdef BODMER_TFT
    tft.startWrite();
#endif

    //Serial.print("Decoding png: ");
    rc = png.decode(NULL, 0);
    //Serial.print("res: ");
    //Serial.println(rc);
    //png.close();

#ifdef ADAFRUIT_TFT
    tft.drawRGBBitmap(0,0,screenBuffer,TFT_WIDTH,TFT_HEIGHT);
#endif

#ifdef BODMER_TFT
    tft.endWrite();
#endif

    Serial.println();
    Serial.println("Image written to display.");
    rc = PNG_SUCCESS;
  } else {
    Serial.println("Failed to decode PNG image.");
    rc = PNG_DECODE_ERROR;
  }
  
  return rc;
}

bool blink = false;

#ifdef BODMER_TFT
  void updateDisplay(int data){
    TFT_eSprite m = TFT_eSprite(&tft);
    m.setColorDepth(8);
    m.createSprite(240, 128);
    m.fillSprite(TFT_BLACK);

    m.setFreeFont(&FreeSans18pt7b);
    m.setTextSize(1);
    m.setTextDatum(MC_DATUM);

    m.setTextColor(TFT_GREEN);

    if(blink){
      m.drawString(".", m.width() / 2 + 100, m.height() / 2 - 60);
    }
    blink = !blink;

    int y = -48;
    m.drawString("ctr: " + String(data), m.width() / 2, m.height() / 2 + y); y += 26;
    m.drawString("state: " + String(state), m.width() / 2, m.height() / 2 + y);  y += 26;
    m.drawString("ptr: " + String(imageBufferPointer), m.width() / 2, m.height() / 2 + y);  y += 26;
    m.drawString("type: " + String(type), m.width() / 2, m.height() / 2 + y);  y += 26;

    m.pushSprite(tft.width() / 2 - 120, tft.height() / 2 - 64);
    m.deleteSprite();
  }
#endif

void setState(STATE newState){
  Serial.printf("SetState: %d\n", newState);
  if(state != newState){
    stateTime = millis();
    state = newState;
  }
}

bool checkAnswer(int btn){
  CARD card = cards[displayedCard];
  displayedCard += 1;
  if(card.correct == btn){
    score += card.weight;
    if(score > 8) score = 8;
    Serial.printf("Score increased to %d\n", score);
    return true;
  } else {
    score -= card.weight;
    if(score < 0) score = 0;
    Serial.printf("Score decreased to %d\n", score);
    //play audio of card.audioNOK
    return false;
  }
}

void displayCubie(uint8_t num){
  if(num >= imgMapLen) return;
  if(displayedCubie != num){
    drawPNG((uint8_t *) imgMap[num], sizeMap[num], true);
    displayedCubie = num;
  }
}

void showScoreLeds(uint8_t score){
    //All leds off
  for( uint8_t i = 0 ; i < NUM_LEDS ; i++ ){
    leds[i].setRGB(0,0,0);
  }

  //Show score on led strip
  if(score > 7) leds[8].setRGB(96,96,0);
  if(score > 6) leds[7].setRGB(128,128,0);
  if(score > 5) leds[6].setRGB(128,128,32);
  if(score > 4) leds[5].setRGB(128,128,64);
  leds[4].setRGB(128,128,128);
  if(score < 4) leds[3].setRGB(64,128,196);
  if(score < 3) leds[2].setRGB(32,64,196);
  if(score < 2) leds[1].setRGB(0,32,196);
  if(score < 1) leds[0].setRGB(0,0,196);
}

void displayScore(uint8_t score){
  displayCubie(score);
  showScoreLeds(score);
  FastLED.show();
}

void blinkScore(uint8_t score){
  showScoreLeds(score);
  uint32_t ms = millis();
  ms = ms - stateTime;
  ms /= 250;
  bool on = true;
  if(ms % 2) on = false;
  if(score == 8) on ? leds[8].setRGB(96,96,0) : leds[8].setRGB(0,0,0);
  if(score == 7) on ? leds[7].setRGB(128,128,0) : leds[7].setRGB(0,0,0);
  if(score == 6) on ? leds[6].setRGB(128,128,32) : leds[6].setRGB(0,0,0);
  if(score == 5) on ? leds[5].setRGB(128,128,64) : leds[5].setRGB(0,0,0);
  if(score == 4) on ? leds[4].setRGB(128,128,128) : leds[4].setRGB(0,0,0);
  if(score == 3) on ? leds[3].setRGB(64,128,196) : leds[3].setRGB(0,0,0);
  if(score == 2) on ? leds[2].setRGB(32,64,196) : leds[2].setRGB(0,0,0);
  if(score == 1) on ? leds[1].setRGB(0,32,196) : leds[1].setRGB(0,0,0);
  if(score == 0) on ? leds[0].setRGB(0,0,196) : leds[0].setRGB(0,0,0);
  FastLED.show();
}

void loop() {

  if(state == INIT){
    if(stateTime + 5000 < millis()){
      setState(WAITING_FOR_ANSWER);
    }
  } else if(state == WAITING_FOR_ANSWER){
    if(!digitalRead(BUTTON1) || !digitalRead(DSP_BTN_1)){
      wasLastAnswerCorrect = checkAnswer(1);
      setState(SHOWING_ANSWER_RESULT);
    } else if(!digitalRead(BUTTON2) || !digitalRead(DSP_BTN_2)){
      wasLastAnswerCorrect = checkAnswer(2);
      setState(SHOWING_ANSWER_RESULT);
    }
  } else if(state == SHOWING_ANSWER_RESULT){
    if(wasLastAnswerCorrect) displayCubie(8);
    else displayCubie(1);
    if(stateTime + 2000 < millis()){
      setState(SHOWING_CURRENT_SCORE);
    }
    blinkScore(score);
  } else if(state == SHOWING_CURRENT_SCORE){
    displayScore(score);
    if(displayedCard >= numberOfCards){
      setState(FINISHED);
    } else setState(WAITING_FOR_ANSWER);
  } else if(state == FINISHED){
    
  }

  if(!digitalRead(BUTTON1) && !digitalRead(BUTTON2)) rp2040.reboot();
}

void reboot() {
  AIRCR_Register = 0x5FA0004;
  while(1){};
}

void printHex(uint8_t b){
  Serial.print(hexMap[(b>>4) & 0x0F]);
  Serial.print(hexMap[b & 0x0F]);
}

void printIntHex(uint32_t num, uint8_t len, bool newLine){
  int base = (len-1) * 8;
  for( int i = 0 ; i < len ; i++ ){
    printHex((num >> (base - (i * 8))) & 0xFF);
  }
  if(newLine) Serial.println();
}