#
#  Makefile
#  gauge-ocr
#
#  Created by Andrey Perevozchikov on 17.04.17.
#

CC = g++
CFLAGS = -c -Wall `pkg-config --cflags /usr/local/opt/opencv/lib/pkgconfig/opencv.pc`
LDFLAGS = `pkg-config --libs /usr/local/opt/opencv/lib/pkgconfig/opencv.pc` -rpath /usr/local/opt/opencv/lib
TARGET = ocr
SOURCES = main.cpp marker.cpp detector.cpp
OBJECTS = $(SOURCES:.cpp=.o)

all: $(SOURCES) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o $(TARGET)
