TARGET = fir_filter_perf
LIBS = -L/opt/gnuradio/lib/ -ludev -lboost_date_time -lboost_filesystem -lboost_program_options -lboost_regex -lboost_system -lboost_thread -lgnuradio-blocks -lgnuradio-runtime -lvolk -lgnuradio-pmt
INCLUDE = -I/home/jpendlum/gr-zynq/include/ -I/opt/gnuradio/include/
CC = g++
CFLAGS = -O2

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.cc, %.o, $(wildcard *.cc))
HEADERS = $(wildcard *.h)

%.o: %.cc $(HEADERS)
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)
