BIN=main
OBJS=main.o

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega2560 -Wall -Wextra -Wl,-Map,main.map
PORT=/dev/ttyACM0

${BIN}.hex: ${BIN}.elf
	${OBJCOPY} -O ihex -R .eeprom $< $@

${BIN}.elf: ${OBJS}
	${CC} -o $@ $^ ${CFLAGS}

install: ${BIN}.hex
	#avrdude -v -p atmega2560 -c arduino -P ${PORT} -b 115200 -U flash:w:$<
	avrdude -v -q -D -p atmega2560 -P ${PORT} -c stk500 -b 115200 -U flash:w:$<

clean:
	rm -f ${BIN}.elf ${BIN}.hex ${OBJS}
