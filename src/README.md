avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o scara.o scara.c && \
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o utils.o utils.c && \
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o USART.o USART.c && \
avr-gcc -mmcu=atmega328p scara.o utils.o USART.o -o scara && \
avr-objcopy -O ihex -R .eeprom scara scara.hex && \
avrdude -v -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 115200 -U flash:w:scara.hex

avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o stepper.o stepper.c && \
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o utils.o utils.c && \
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o USART.o USART.c && \
avr-gcc -mmcu=atmega328p stepper.o USART.o utils.o -o stepper && \
avr-objcopy -O ihex -R .eeprom stepper stepper.hex && \
avrdude -v -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 115200 -U flash:w:stepper.hex

avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o inverseKinematics.o inverseKinematics.c -lm && \
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o USART.o USART.c && \
avr-gcc -mmcu=atmega328p inverseKinematics.o USART.o -o inverseKinematics && \
avr-objcopy -O ihex -R .eeprom inverseKinematics inverseKinematics.hex && \
avrdude -v -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 115200 -U flash:w:inverseKinematics.hex
