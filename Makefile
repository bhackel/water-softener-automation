CC := $(CROSS_COMPILE)gcc
WIRINGPI_PATH := /home/USER_NAME/RPdev/wiringPi_armhf/ # You need to modify it

# Default flags (with optimization, for example -O2)
CFLAGS := -O2
LDFLAGS := -lwiringPi -lpthread -lmicrohttpd -lsqlite3 -I$(WIRINGPI_PATH) -L$(WIRINGPI_PATH)

all:
		$(CC) $(CFLAGS) -o main_section1 main_section1.c database.c assignment1.c $(LDFLAGS)

# New target: compile without optimizations.
noopt:
		$(CC) -O0 -o main_section1 main_section1.c assignment1.c $(LDFLAGS)

clean:
		rm -rf main_section1

run: all
		echo -e "\n\ncompile success\n\n"; sleep 1.5; ./main_section1

drop:
		@echo "Dropping database tables..."
		@sqlite3 sensor_data.db "DROP TABLE IF EXISTS temperature_data; DROP TABLE IF EXISTS tds_data;"

runbg: all
		echo -e "\n\ncompile success\n\n"; sleep 1.5; ./main_section1 &