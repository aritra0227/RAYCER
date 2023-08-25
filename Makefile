CC = g++
CFLAGS = -Wall -Wextra -std=c++11
SRCS = main.cpp 

OBJS = $(SRCS:.cpp=.o)

OUT = output_image

all: $(OUT)

$(OUT): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(OUT)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(OUT)

run: all
	./$(OUT) > image.ppm

.PHONY: all clean