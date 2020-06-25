/*
 * speaker_pcm
 *
 * Plays 8-bit PCM audio on pin 11 using pulse-width modulation (PWM).
 * For Arduino with Atmega168 at 16 MHz.
 *
 * Uses two timers.
 * The first changes the sample value 8000 times a second.
 * The second holds pin 11 high for 0-255 ticks out of a 256-tick cycle,
 * depending on sample value. The second timer repeats 62500 times per second
 * (16000000 / 256), much faster than the playback rate (8000 Hz), so
 * it almost sounds halfway decent, just really quiet on a PC speaker.
 * -----------------------------------------------------------------------------
 * Takes over Timer 1 (16-bit) for the 8000 Hz timer. This breaks PWM
 * (analogWrite()) for Arduino pins 9 and 10.
 *
 * Takes Timer 2 (8-bit)
 * for the pulse width modulation, breaking PWM for pins 11 & 3.
 * -----------------------------------------------------------------------------
 *
 * References:
 *     http://www.uchobby.com/index.php/2007/11/11/arduino-sound-part-1/
 *     http://www.atmel.com/dyn/resources/prod_documents/doc2542.pdf
 *     http://www.evilmadscientist.com/article.php/avrdac
 *     http://gonium.net/md/2006/12/27/i-will-think-before-i-code/
 *     http://fly.cc.fer.hr/GDM/articles/sndmus/speaker2.html
 *     http://www.gamedev.net/reference/articles/article442.asp
 *
 * Michael Smith <michael@hurts.ca>
 */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define SAMPLE_RATE 8000

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PDM_Leonardo.h"

/*
 * The audio data needs to be unsigned, 8-bit, 8000 Hz, and small enough
 * to fit in flash. 10000-13000 samples is about the limit.
 *
 * sounddata.h should look like this:
 *     const int largo_del_array=10000;
 *     const unsigned char array_samples[] PROGMEM = { ..... };
 *
 * You can use wav2c from GBA CSS:
 *     http://thieumsweb.free.fr/english/gbacss.html
 * Then add "PROGMEM" in the right place. I hacked it up to dump the samples
 * as unsigned rather than signed, but it shouldn't matter.
 *
 * http://musicthing.blogspot.com/2005/05/tiny-music-makers-pt-4-mac-startup.html
 * mplayer -ao pcm macstartup.mp3
 * sox audiodump.wav -v 1.32 -c 1 -r 8000 -u -1 macstartup-8000.wav
 * sox macstartup-8000.wav macstartup-cut.wav trim 0 10000s
 * wav2c macstartup-cut.wav sounddata.h sounddata
 *
 * (starfox) nb. under sox 12.18 (distributed in CentOS 5), i needed to run
 * the following command to convert my wav file to the appropriate format:
 * sox audiodump.wav -c 1 -r 8000 -u -b macstartup-8000.wav
 */


/*
CHEQUEAR: QUE PASA CON EL SIGNO DE LOS BYTES DE SAMPLES???? SON UNSIGNED...
ENTONCES EL RAMP FINAL HACIA "CERO" EN REALIDAD TIENDE HACIA EL CERO DIGITAL..
LUEGO DEL PASABAJOS, EL CERO DIGITAL DEBERIA FILTRARSE COMO OFFSET Y QUEDAR
EN EL CERO ANALOGICO (PUNTO MEDIO)...
*/

/*
Para el port UNO >> Leonardo (32u4)
--------------------------------------------------------------------------------
OCR1A   Output Compare Register A del TIMER1
OCR2A   Output Compare Register A del TIMER2
ASSR    Asynchronous status register -- The Timer/Counter can be clocked internally, via the prescaler, or asynchronously clocked from the TOSC1/2 pins
  ..EXCLK "6" When EXCLK is written to one, and asynchronous clock is selected, the external clock input buffer is enabled and an external clock can be input on timer oscillator 1 (TOSC1) pin instead of a 32kHz crystal.
  ..AS2   "5" When the AS2 bit in the ASSR register is written to logic one, the clock source is taken from the Timer/Counter oscillator connected to TOSC1 and TOSC2.
TCCR2A  TIMER2 Control Register A (donde se define la configuracion segun cada bit)
  ..WGM20 "0" bit 0 del Waveform Generator Mode del TIMER2 -- Controlan el modo en que se genera la forma de onda (fast pwm, phase correct, etc)
  ..WGM21 "1" bit 1
TCCR2B  TIMER2 Control Register B (donde se define la configuracion segun cada bit)
  ..WGM22 "3" bit 2
COM2A0    Compare Output Mode TIMER2, si la salida se es set, clear, toggle, etc...
COM2A1
COM2B0
COM2B1

  The compare output mode bits do
  not affect the counting sequence, while the waveform generation mode bits do. The COM0x1:0 bits control whether the
  PWM output generated should be inverted or not (inverted or non-inverted PWM). For non-PWM modes the COM0x1:0 bits
  control whether the output should be set, cleared, or toggled at a compare match
*/

//int pinDeSalida = 11;   // OC2A - TIMER2 Output Compare A (salida PWM)
int pinDeSalida = 5;   // OC3A - TIMER3 Output Compare A (salida PWM)
const int puertoSalida = DDC6;   // DDC6 es el pin 5, no es lo mismo el pin del arduino que el pin del chip

unsigned char const *array_samples=0;
int largo_del_array=0;
volatile uint16_t posicion;
byte ultimoSample;  // va a almacenar el valor del ultimo sample

// This is called at 8000 Hz to load the next sample.
// TIMERx_COMPA_vect se invoca cuando coincide el timer con un numero en el registro correspondiente
// TIMER1 16 bit
ISR(TIMER1_COMPA_vect) {
  if (posicion >= largo_del_array) {       // si se llego al ultimo sample (final del audio)
        if (posicion == largo_del_array + ultimoSample) { // detiene si llego al final + el valor del ultimo sample
          detener();
        }
        else {
          // hace una bajada lineal sample a sample desde el valor final para evitar los clicks
          // es ingenioso: cuanto mas chico sea el valor del sample, menos tarda el ramp
          // tal vez se puedan setear juntos con OCR3A sin usar H L
          // al ser de 16 bit conviene frenar las interrupciones
          //cli();
          OCR3A = ((largo_del_array + ultimoSample - posicion));// | B01111111;  // Salida (tiempo de on vs off PWM)
          //sei();
          // chequear el tema de los 16 bits cuando se hace el ramp
          // setear siempre o se puede "dejar seteado" ??
          // OCR3AL = 0x7F; // esto tiene poco sentido... habria que aprovechar los 16 bit
        }
  }
  else {
          // si hay que seguir reproduciendo (lo normal), lee un sample y lo pone en OCR2A
          OCR3A = ((pgm_read_byte(&array_samples[posicion])));// | B01111111;
          //  el byte bajo, 0 o 127 ?????
          //OCR3AL = 0x7F; // esto tiene poco sentido... habria que aprovechar los 16 bit
  }

  posicion++;
}

void reproducir(unsigned char const *datos, int largo)
{
  array_samples = datos;
  largo_del_array = largo;

  pinMode(pinDeSalida, OUTPUT);
  //DDRC |= _BV(puertoSalida);  // Setear como OUTPUT el pin DDCx de salida

  // -------------- Setear el TIMER3 para hacer PWM en el pin de salida.

      // Usar el reloj interno (para usar el prescaler?)
      // entre ambos seteos solo hacen eso, elegir el reloj interno
      //ASSR &= ~(_BV(EXCLK) | _BV(AS2));   // setea ambos en 0 a la vez
      // Setear para reloj interno SIN prescaler
      TCCR3B = (TCCR3B | _BV(CS30)) & ~(_BV(CS31) | _BV(CS32));

      // Configuraciones en los TCCRxx, Timer Control Register
      // Cada bit del TCCRxx define algun aspecto del funcionamiento del timer

      // Setear el modo fast PWM de 8bit en el TIMER3
      // Modo 5 fast PWM con top en 0x00FF //
      //TCCR2A |= _BV(WGM21) | _BV(WGM20);  // "1"
      //TCCR2B &= ~_BV(WGM22);  // "0"
      TCCR3A = (TCCR3A | _BV(WGM30)) & ~_BV(WGM31);
      TCCR3B = (TCCR3B | _BV(WGM32)) & ~_BV(WGM33);

      // Setear PWM no inversor en el pin OC2A (TIMER2)
      // En el arduino UNO es el pin 11, ver en el Leonardo.
      // TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
      // TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));

      // Sin prescaler (p.158)
      //TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

      // Poner de antemano el primer PWM segun el primer sample.
      //cli();    // frena las interrupciones (para escribir en OCR3A que es de 16 bits)
      OCR3A = (pgm_read_byte(&array_samples[0]));// | B01111111; // ojo a ver si funciona con 16bit

  // -------------- Setear el TIMER1 para mandar un sample a cada interrupcion.

  // Poner modo CTC (Clear Timer on Compare Match) (p.133)
  // OCR1A debe cambiarse DESPUES, para que no se ponga en cero aca.
  //TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  //TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

  //poner los tres canales ABC en CTC
  // capaz solo usar el canal A
  TCCR1A |= _BV(COM3A1) | _BV(COM3A0) | _BV(COM3B1) | _BV(COM3B0) | _BV(COM3C1) | _BV(COM3C0);

  // No prescaler (p.134)
  //TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  TCCR1B = (TCCR1B | _BV(CS30)) & ~(_BV(CS31) | _BV(CS32));

  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  OCR1A = F_CPU / SAMPLE_RATE;    // 16mhz / 8000 = 2000

  // Enable interrupt when TCNT1 == OCR1A (p.136)
  // ..o sea se activa la interrupcion segun el SAMPLE_RATE
  TIMSK1 |= _BV(OCIE1A);

  // aca esta la clave de lo que pasa con el ramp final
  // busca en PROGMEM el ultimo sample y lo guarda en "ultimoSample"
  ultimoSample = pgm_read_byte(&array_samples[largo_del_array-1]);

  posicion = 0;
  //sei();    // reanuda las interrupciones
}

void detener()
{
  // Disable playback per-sample interrupt.
  TIMSK1 &= ~_BV(OCIE1A);

  // Disable the per-sample TIMER1 completely.
  TCCR1B &= ~_BV(CS10);

  // Disable the PWM TIMER3
  TCCR3B &= ~_BV(CS10);

  digitalWrite(pinDeSalida, LOW);
}
