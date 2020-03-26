CC = avr-gcc 
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

MCU   = atmega1284
F_CPU = 1000000UL

#AVR_LIBRARY = /opt/local/avr/include/ 
AVR_LIBRARY += ../../../embedded-systems/AVR/avr_library

# C and C preprocessor directives, flags, etc.
CFLAGS = -Os -g -std=gnu99 -funsigned-char -Wall
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I.
CPPFLAGS += -I$(AVR_LIBRARY)

# Linker stuff.
LDFLAGS = -Wl,-Map,main.map

TARGET_ARCH = -mmcu=$(MCU)

app: main.o lcd.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $(TARGET_ARCH) lcd.o main.o -o app.o
	avr-objcopy -j .text -j .data -O ihex app.o app.hex
main.o: main.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $(TARGET_ARCH) main.c -c
lcd.o:
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $(TARGET_ARCH) $(AVR_LIBRARY)/lcd/lcd.c -c
hex:
	avr-objcopy -j .text -j .data -O ihex app.o app.hex
flash:
	avrdude -p m1284 -c usbtiny -U flash:w:app.hex:i 
clean:
	rm -f *.o *.map *.hex
