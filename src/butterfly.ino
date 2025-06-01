#include <FastLED.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define PIN_LED PIN_PB0
#define PIN_LIGHT PIN_PB2
#define PIN_LVD PIN_PB4
#define LED_COUNT 10
#define BRIGHT_INIT 32

#define BRIGHT_MIN 32
#define BRIGHT_MAX 240
#define PHOTOCELL_MAX 128

//#define BODGE

#ifdef BODGE
#define LED(x) leds[x % (LED_COUNT - 1)]
#else
#define LED(x) leds[(x+1)%LED_COUNT]
#endif

#define ADC_DELAY 1 // delay in ms between writing ADMUX and starting conversion
                    // 1 ms according to the datasheet
#define UVLO_ENGAGE   93
#define UVLO_RELEASE 87

bool uvlo = false;

CRGB leds[LED_COUNT];
const CHSV colors[6] = {
  CHSV(140, 200, 255), // blue
  CHSV(0,0,0),         // off
  CHSV(247, 150, 255), // pink
  CHSV(0,0,0),         // off
  CHSV(0, 0, 255),     // white
  CHSV(0,0,0)          // off
};

const CHSV colorsW[10] = {
  CHSV(140, 200, 255), // 0 blue
  CHSV(0,0,128),       // 1 white
  CHSV(247, 150, 255), // 2 pink
  CHSV(247, 75, 224),  // 3 light pink
  CHSV(0, 0, 192),     // 4 white
  CHSV(247, 75, 224),  // 5 light pink
  CHSV(247, 150, 255), // 6 pink
  CHSV(0,0,128),       // 7 white
  CHSV(140, 200, 255), // 8 blue
  CHSV(140, 200, 255), // 9 blue
};

#define BUTTON_DEBOUNCE_MS 100
const PROGMEM uint8_t button_pin[] = {PIN_PB1, PIN_PB3};
volatile uint8_t button_count[] = {0, 0};
volatile bool button_pressed[] = {false, false};
volatile bool button_read[] = {false, false};
volatile uint16_t button_held[] = {0, 0};

ISR(PCINT0_vect){
      // clear interrupt
      GIFR |= (1 << PCIF);
}
/*
ISR(WDT_vect){
  // have to reenable the WDT interrupt every time; otherwise
  // it will reset next time instead
  WDTCR |= (1 << WDIE);

  // don't have to do anything else in the ISR; the point of the
  // interrupt is just to wake up the CPU
}*/

ISR(TIMER0_COMPA_vect){
  // note - this function gets called every 2.048 ms (the details of which are
  // buried in wiring.c within ATTinyCore, but the timer 0 prescaler gets
  // set to 64 there).
  
  for (uint8_t i = 0; i < sizeof(button_count); i++){
    if (digitalRead(pgm_read_byte(&button_pin[i])) == LOW){
      if (!button_read[i]) button_count[i]++;
      button_held[i] += 2;
    } else {
      button_count[i] = 0;
      button_read[i] = false;
      button_held[i] = 0;
    }
    if (button_count[i] > BUTTON_DEBOUNCE_MS/2) button_pressed[i] = true;
  }
}

void clear_button(uint8_t button){
  button_count[button] = 0;
  button_read[button] = false;
  button_pressed[button] = false;
  button_held[button] = false;
}

bool get_button(uint8_t button){
  // return true if the selected button passed as argument has been pressed
  // since last called
  bool r;
  noInterrupts();
  r = button_pressed[button];
  if (r){
    button_pressed[button] = false;
    button_read[button] = true;
    button_count[button] = 0;
  }
  interrupts();
  return r;
}


void setup() {

  // turn off analog comparator; we don't need it
  ACSR   |= (1 << ACD);


  pinMode(PIN_LVD, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LIGHT, INPUT);
  pinMode(PIN_PB1, INPUT_PULLUP);
  pinMode(PIN_PB3, INPUT_PULLUP);
  digitalWrite(PIN_LVD, HIGH);

  FastLED.setBrightness(BRIGHT_INIT);
  FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, LED_COUNT);
  FastLED.setCorrection(Typical8mmPixel);

  // turn on the timer 0 output compare A interrupt
  // for button debouncing. This avoids using timer 1,
  // but with the disadvantage that the period is
  // determined by obscure and arcane magic deep within
  // ATTinyCore
  OCR0A = 0x80;
  TIMSK |= (1 << OCIE0A);

    // enable WDT interrupt, prescaler = 1001 (8 sec)
//  WDTCR = 0b11111001;

    // enable pin change interrupt on PB1
    PCMSK |= (1 << PCINT1);
    GIMSK |= (1 << PCIE);
}

#define NUM_MODES 5

void loop() {
  static uint8_t pos = 0;
  static uint8_t mode = 0;
  static uint8_t voltage = 66;
  static uint32_t bright_accum = 0;

  uint8_t wake_count = 0;

  static uint16_t bright;
  static unsigned long last_check = 0;
  unsigned long now = millis();

  if (now - last_check > 1024){

    // every 1024 ms, check supply voltage by reading the output
    // of the onboard 1.1V bandgap reference
    // since analogRead doesn't include the required settling time
    // we have to roll our own (although maybe this doesn't matter
    // except for the first conversion)

    // note: if in UVLO, cycle doesn't get incremented
    // so this will run every time until out of UVLO
    // turn on ADC
    //         1------- enable ADC
    //         -0------ don't start conversion
    //         --0----- auto trigger disable
    //         ---0---- clear interrupt flag
    //         ----0--- interrupt disable
    //         -----110 prescaler = 64 (8 MHz / 64 = 125 kHz; datasheet recommends 50-200 kHz)
    ADCSRA = 0b10000110;
  
    // set ADC mux

    //        00-x---- ADC voltage reference to Vcc
    //        --1----- left adjust result (don't need 2 lowest bits)
    //        ----1100 read 1.1V reference
    ADMUX = 0b00101100;

    delay(ADC_DELAY);                     // settling time 1ms per datasheet

    ADCSRA |= (1 << ADSC);        // start conversion
    while (ADCSRA & (1 << ADSC)); // wait for conversion to complete
    voltage = ADCH;

    if (!uvlo && (voltage > UVLO_ENGAGE)){
      uvlo = true;
      FastLED.clear(true);
      // if we don't do this the LEDs will still be able to sink current through the data input
      pinMode(PIN_LED, INPUT);
      digitalWrite(PIN_LVD, LOW);
    }  else if (uvlo && voltage < UVLO_RELEASE){
      uvlo = false;
      digitalWrite(PIN_LVD, HIGH);
      pinMode(PIN_LED, OUTPUT);
      // re-enable ADC
      ADCSRA |= (1 << ADEN);
    }
    if (uvlo){

      // SLEEP_MODE_PWR_DOWN is the lowest sleep mode (all clocks off except WDT)
      set_sleep_mode(SLEEP_MODE_PWR_DOWN); // TODO: I think this is duplciated

      // disable the ADC
      ADCSRA &= ~(1 << ADEN);

      sleep_enable();
      // disable BOD to save power
      // this only works on ATTiny85 revision C and newer
      // BOD is needed to prevent EEPROM corruption
      // this doesn't seem to work; I'm not sure why. But it doesn't make
      // a huge difference anyway. (Note: I think it doesn't work on the ATTiny85V.)
      cli(); // disable interrupts during timed sequence
      sleep_bod_disable();
      sei();
      sleep_cpu();

      get_button(0); // since we woke up because of the pin change interrupt, throw
                     // out the button press

      // the following is recommended by the datasheet
      // not super necessary, just makes it harder to accidentally sleep
      sleep_disable();
      return;
    }

    last_check = now;

  }

  // set brightness

  int16_t new_bright = ((((analogRead(A1) / 4 - 50) * 255) / (PHOTOCELL_MAX - 50)) * (BRIGHT_MAX - BRIGHT_MIN)) + BRIGHT_MIN;
  if (new_bright < BRIGHT_MIN) new_bright = BRIGHT_MIN;

  if (new_bright > BRIGHT_MAX) new_bright = BRIGHT_MAX;
  bright_accum = (255 * bright_accum) / 256 + new_bright;
  if (bright_accum > 256L * BRIGHT_MAX) bright_accum = 256L * BRIGHT_MAX;
  bright = bright_accum / 256;
  if (bright < BRIGHT_MIN) bright = BRIGHT_MIN;

  FastLED.setBrightness(bright);



  if (get_button(0)){
    FastLED.clear(true);
    pinMode(PIN_LED, INPUT);
    digitalWrite(PIN_LVD, LOW);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // disable button debounce
    TIMSK &= ~(1 << OCIE0A);

    // disable the ADC
    ADCSRA &= ~(1 << ADEN);

    /*// recite incantation to disable the WDT
    cli();
    wdt_reset();
    MCUSR = 0;
    // set WDE and WDCE in a single operation
    // then within four clock cycles, clear WDE and WDCE in a single operation
    WDTCR |= (1 << WDCE) | (1 << WDE);
    WDTCR = 0;
    sei();*/

    sleep_enable();
    sleep_cpu();

    do {
      // ok now wake up
      sleep_disable();

      delay(1);
      if (digitalRead(PIN_PB1) == HIGH) {
        // button released; back to bed
        wake_count = 0;
        sleep_enable();
        sleep_cpu();
      } else {
        wake_count += 1;
      }
    } while (wake_count < 100);

    // enable the ADC
    ADCSRA |= (1 << ADEN);
    
    digitalWrite(PIN_LVD, HIGH);
    pinMode(PIN_LED, OUTPUT);
    //re-enable button debounce
    TIMSK |= (1 << OCIE0A);


  } else if (get_button(1)){
    mode = (mode + 1) % NUM_MODES;
    pos  = 0;
  }

  static uint8_t cur = 0;

  if (mode == 0){  // trans flag scrolling mode
    for (uint8_t i = 0; i < LED_COUNT/2; i++){
      if (pos < 128){
        LED(i) = blend((CRGB) colorsW[(cur + 2*i) % 10], (CRGB) colorsW[(cur + 2*i + 1) % 10], pos * 2);
      } else {
        LED(i) = blend((CRGB) colorsW[(cur + 2*i + 1) % 10], (CRGB) colorsW[(cur + 2*i + 2) % 10], pos * 2);
      }
      LED(9-i) = LED(i);      
    }
    pos += 1;
    if (pos == 0) {
      cur += 2;
    //  FastLED.show();
    //  delay(1000);
    }

  } else if (mode == 1){ // trans flag static

    for (uint8_t i = 0; i < LED_COUNT/2; i++){
      LED(i) = blend((CRGB) colorsW[(2*i) % 10], (CRGB) colorsW[(2*i + 1) % 10], 0);
      LED(9-i) = LED(i);
    }
  } else if (mode == 2){
    for (uint8_t i = 0; i < LED_COUNT; i++){
      LED(i) = blend((CRGB) colors[(cur) % 6], (CRGB) colors[(cur + 1) % 6], pos & 0xff);  
    }
    pos += 4;
    if (pos == 0) cur++;
  } else if (mode == 3){
    for(uint8_t i = 0; i < LED_COUNT / 2; i++){
      LED(i) = CHSV(i*(255/(LED_COUNT/2))+pos, 255, 255);
      LED(LED_COUNT-1-i)=LED(i);
    }
    pos += 2;
  } else if (mode == 4){

    static unsigned long last_chg = 0; 

    for(uint8_t i = 0; i < LED_COUNT / 2; i++){
      CHSV rainbow_color = CHSV(((LED_COUNT/2-i-1)*(255/(LED_COUNT/2))), 255, 255);
      CHSV trans_color = colorsW[i * 2];
      if (pos < 128)
        LED(i) = blend(rainbow_color, trans_color, 2*pos);
      else
        LED(i) = blend(trans_color, rainbow_color, 2*pos);
      LED(LED_COUNT-1-i)=LED(i);
    }

    if ((pos & 127) == 0){
      if (millis() - last_chg > 7000){
        last_chg = millis();
        pos += 2;
      }
    } else pos += 2; // has to be even
  }
  FastLED.show();
  delay(30);
}
