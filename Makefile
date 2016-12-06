BIN=main
OBJS=main.o i2c.o hdc1008.o

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=1000000UL -mmcu=attiny2313 -Wall -Wextra -Wl,-Map,main.map
PORT=/dev/ttyACM1

${BIN}.hex: ${BIN}.elf
	${OBJCOPY} -O ihex -R .eeprom $< $@

${BIN}.elf: ${OBJS}
	${CC} -o $@ $^ ${CFLAGS}

install: ${BIN}.hex
	avrdude -q -p t2313 -c usbtiny -e -U flash:w:$<

clean:
	rm -f ${BIN}.elf ${BIN}.hex ${OBJS}
