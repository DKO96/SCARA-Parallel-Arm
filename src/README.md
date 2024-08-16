avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o scara.o scara.c && \
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o utils.o utils.c && \
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o USART.o USART.c && \
avr-gcc -mmcu=atmega328p scara.o utils.o USART.o -o scara && \
avr-objcopy -O ihex -R .eeprom scara scara.hex && \
avrdude -v -c arduino -p ATMEGA328P -P /dev/ttyUSB0 -b 115200 -U flash:w:scara.hex

