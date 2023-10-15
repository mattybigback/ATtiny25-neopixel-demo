// ===================================================================================
// Project:   TinyBling25 - Neopixel controller for the ATTiny25
// Author:    Matt Harrison
// Github:    https://github.com/mattybigback
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// References:
// -----------
// The Neopixel implementation is based on NeoController.
// https://github.com/wagiminator/ATtiny13-NeoController

#define __AVR_ATTiny25__
#include <avr/io.h>        // for GPIO
#include <avr/wdt.h>       // for the watchdog timer
#include <avr/sleep.h>     // for sleep functions
#include <avr/interrupt.h> // for interrupts
#include "HsvConverter.h"

// Pin definitions
#define NEO_PIN PB4 // Pin for neopixels
// NeoPixel parameter and macros
#define NEO_RGB                           // type of pixel: NEO_GRB or NEO_RGB
#define NEO_init() DDRB |= (1 << NEO_PIN) // set pixel DATA pin as output
#define NUM_PIXELS 12                     // set the number of pixels to drive

// NeoPixel buffer

uint8_t pixel_red[NUM_PIXELS];
uint8_t pixel_green[NUM_PIXELS];
uint8_t pixel_blue[NUM_PIXELS];

// Send a byte to the pixels string
void NEO_sendByte(uint8_t byte)
{ // CLK  comment
  for (uint8_t bit = 8; bit; bit--)
    asm volatile(                     //  3   8 bits, MSB first
        "sbi  %[port], %[pin]   \n\t" //  2   DATA HIGH
        "sbrs %[byte], 7        \n\t" // 1-2  if "1"-bit skip next instruction
        "cbi  %[port], %[pin]   \n\t" //  2   "0"-bit: DATA LOW after 3 cycles
        "rjmp .+0               \n\t" //  2   delay 2 cycles
        "add  %[byte], %[byte]  \n\t" //  1   byte <<= 1
        "cbi  %[port], %[pin]   \n\t" //  2   "1"-bit: DATA LOW after 7 cycles
        ::
            [port] "I"(_SFR_IO_ADDR(PORTB)),
        [pin] "I"(NEO_PIN),
        [byte] "r"(byte));
}

// Write color to a single pixel
void NEO_writeColor(uint8_t r, uint8_t g, uint8_t b)
{
#if defined(NEO_GRB)
  NEO_sendByte(g);
  NEO_sendByte(r);
  NEO_sendByte(b);
#elif defined(NEO_RGB)
  NEO_sendByte(r);
  NEO_sendByte(g);
  NEO_sendByte(b);
#else
#error Wrong or missing NeoPixel type definition!
#endif
}

// Write buffer to pixels
void NEO_show(void)
{
  for (uint8_t i = 0; i < NUM_PIXELS; i++)
  {
    NEO_writeColor(pixel_red[i], pixel_green[i], pixel_blue[i]);
  }
}

void NEO_clear()
{
  for (uint8_t i = 0; i < NUM_PIXELS; i++)
  {
    NEO_writeColor(0, 0, 0);
  }
}
void NEO_clear_buffer()
{
  for (uint8_t i = 0; i < NUM_PIXELS; i++)
  {
    pixel_red[i] = 0;
    pixel_green[i] = 0;
    pixel_blue[i] = 0;
  }
}

void NEO_set_hsv(uint8_t pixel, uint16_t hue, uint8_t sat, uint8_t val)
{
  uint8_t r, g, b;
  HsvConverter::getRgbFromHSV(hue, sat, val, r, g, b);
  pixel_red[pixel] = r;
  pixel_green[pixel] = g;
  pixel_blue[pixel] = b;
}

// Circle all pixels clockwise
void NEO_cw(void)
{
  uint8_t rtemp = pixel_red[NUM_PIXELS - 1];
  uint8_t gtemp = pixel_green[NUM_PIXELS - 1];
  uint8_t btemp = pixel_blue[NUM_PIXELS - 1];
  for (uint8_t i = NUM_PIXELS - 1; i; i--)
  {
    pixel_red[i] = pixel_red[i - 1];
    pixel_green[i] = pixel_green[i - 1];
    pixel_blue[i] = pixel_blue[i - 1];
  }
  pixel_red[0] = rtemp;
  pixel_green[0] = gtemp;
  pixel_blue[0] = btemp;
}

void NEO_rainbow(void){
    for(uint8_t i=0; i<NUM_PIXELS; i++){
      uint16_t hue = 127;
      for(uint8_t j=0; j<i; j++){
        hue = (hue + 128)-1;
      }
      NEO_set_hsv(i,hue,255,255);
  }
}

// ===================================================================================
// Watchdog Implementation (Sleep Timer)
// ===================================================================================

// Reset Watchdog
void resetWatchdog(void)
{
  cli();                                          // timed sequence coming up
  wdt_reset();                                    // reset watchdog
  MCUSR = 0;                                      // clear various "reset" flags
  WDTCR = (1 << WDCE) | (1 << WDE) | (1 << WDIF); // allow changes, clear interrupt
  WDTCR = (1 << WDIE) | (1 << WDP1);              // set interval 64ms
  sei();                                          // interrupts are required now
}

// Watchdog interrupt service routine
ISR(WDT_vect)
{
  wdt_disable(); // disable watchdog
}

int main(void)
{
  resetWatchdog(); // do this first in case WDT fires
  NEO_init();      // init Neopixels

  // Disable unused peripherals and prepare sleep mode to save power
  ACSR = (1 << ACD);                   // disable analog comparator
  DIDR0 = 0x1F;                        // disable digital intput buffer except button
  PRR = (1 << PRADC) | (1 << PRTIM0);  // shut down ADC and timer0
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // set sleep mode to power down
  NEO_clear_buffer();
  NEO_rainbow();
  NEO_show();

  while (1)
  {
    NEO_cw();
    NEO_show();

    resetWatchdog();
    sleep_mode();
  }
}
