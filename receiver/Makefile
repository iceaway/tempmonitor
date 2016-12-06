BIN=main
OBJS=main.o rbuf.o

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Os -DF_CPU=16000000UL -mmcu=atmega328p -Wall -Wextra -Wl,-Map,test.map
PORT=/dev/ttyACM0

${BIN}.hex: ${BIN}.elf
	${OBJCOPY} -O ihex -R .eeprom $< $@

${BIN}.elf: ${OBJS}
	${CC} -o $@ $^ ${CFLAGS}

install: ${BIN}.hex
	avrdude -v -p atmega328p  -c arduino -P ${PORT} -b 115200 -U flash:w:$<

upload: ${BIN}.hex
	scp main.hex pi@raspberrypi:/home/pi

clean:
	rm -f ${BIN}.elf ${BIN}.hex ${OBJS}
