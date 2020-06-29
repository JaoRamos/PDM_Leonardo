# PDM_Leonardo

A simple port of the PCM arduino library (made by Michael Smith &lt;michael@hurts.ca>) that implements PWM "sound" output on the Arduino Leonardo/Pro Micro.
The original library was made for the Arduino UNO (atmega328), and because of the different architectures, it does not work on the Leonardo/Pro Micro (atmega32u4).
But since the principle of operation and main features remain the same, it only needed minor tweaks and name changes to have it working on the atmega32u4.
I made this a bit just for fun, and a bit for teaching purposes.

**WARNING**: this library/port is a work in progress. It WORKS, but i haven't yet fully updated the comments from the original Arduino UNO library by Michael Smith, thus it can be somewhat confusing to read at the moment, specially regarding to Timer names etc...
In the meantime, please feel free to contact me if you have any questions &lt;juan.ramos@unq.edu.ar>.

Short demo with MIDI capabilities:
https://www.youtube.com/watch?v=ay138r9ZPcI
