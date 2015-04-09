#Quiet compilation.
Q=@
GPP = g++
CC = gcc
xLIB = -std=gnu++0x
GLIB_CFLAGS  = -Wall $(shell pkg-config glib-2.0 --cflags) -fPIC
GLIB_LDFLAGS =  $(shell pkg-config glib-2.0 --libs) -lglibivy $(shell pcre-config --libs) -fPIC

GLIB_LDFLAGS2 =  $(shell pkg-config glib-2.0 --libs) -lglibivy $(shell pcre-config --libs) -fPIC
#-o  $@ $^ -lm -lz -lgps $(shell pkg-config gio-2.0 ivy-glib gtk+-3.0 libpcre --libs)
all: ant_tr_pwm gpsd2ivy ant_tr_dc

clean:
	$(Q)rm -f ant_tr_pwm gpsd2ivy ant_tr_dc

ifeq ("$(UNAME)","Darwin")
  C_LIBRARYS = $(shell if test -d /opt/paparazzi/lib; then echo "-L/opt/paparazzi/lib"; elif test -d /opt/local/lib; then echo "-L/opt/local/lib"; fi)
  C_INCLUDES = $(shell if test -d /opt/paparazzi/include; then echo "-I/opt/paparazzi/include"; elif test -d /opt/local/include; then echo "-I/opt/local/include"; fi)
endif

ant_tr_pwm: main.c
	@echo OL $@
	$(Q)$(GPP) $(xLIB) $(GLIB_CFLAGS) bbb-eqep.cpp $(shell pkg-config gio-2.0 ivy-glib gtk+-3.0 --cflags) -o  $@ $^ -lm -lz $(shell pkg-config gio-2.0 ivy-glib gtk+-3.0 libpcre --libs) -DUSE_BBB_PWM
gpsd2ivy: gpsd2ivy.c
	$(CC) $(GLIB_CFLAGS) $(C_LIBRARYS) $(C_INCLUDES) -o $@ $< $(GLIB_LDFLAGS) -lgps

ant_tr_dc: main.c
	@echo OL $@
	$(Q)$(GPP) $(xLIB) $(GLIB_CFLAGS) bbb-eqep.cpp $(shell pkg-config gio-2.0 ivy-glib gtk+-3.0 --cflags) -o  $@ $^ -lm -lz $(shell pkg-config gio-2.0 ivy-glib gtk+-3.0 libpcre --libs)

