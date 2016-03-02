BIN=main
OBJS=main.o

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=1000000UL -mmcu=attiny2313 -Wall -Wextra -Wl,-Map,main.map
PORT=/dev/ttyACM1

${BIN}.hex: ${BIN}.elf
	${OBJCOPY} -O ihex -R .eeprom $< $@

${BIN}.elf: ${OBJS}
	${CC} -o $@ $^ ${CFLAGS}

install: ${BIN}.hex
	#avrdude -v -p atmega2560 -c arduino -P ${PORT} -b 115200 -U flash:w:$<
	#avrdude -v -q -D -p t2313 -P ${PORT} -c usbtiny -b 115200 -U flash:w:$<
	avrdude -v -q -p t2313 -P ${PORT} -c usbtiny -e -U flash:w:$<

clean:
	rm -f ${BIN}.elf ${BIN}.hex ${OBJS}
