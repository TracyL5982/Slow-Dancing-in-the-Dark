# Compiler
CC = g++

# Compiler flags
CFLAGS = -c -Wall -O3

# Include directories for header files (preprocessor flags)
CPPFLAGS = -I./ -I/usr/local/opt/jpeg/include

# Linker flags
LDFLAGS = -L/usr/local/opt/jpeg/lib

# Libraries to link
LDLIBS = -ljpeg

# Executable name
EXECUTABLE = movieMaker

# Source files
SOURCES = movieMaker.cpp
OBJECTS = $(SOURCES:.cpp=.o)

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@

clean:
	rm -f *.o $(EXECUTABLE)



# # calls:
# CC         = g++
# CFLAGS     = -c -Wall -O3 -I./
# LDFLAGS    = -ljpeg
# EXECUTABLE = movieMaker

# SOURCES    = movieMaker.cpp
# OBJECTS    = $(SOURCES:.cpp=.o)

# all: $(SOURCES) $(EXECUTABLE)

# $(EXECUTABLE): $(OBJECTS)
# 	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# .cpp.o:
# 	$(CC) $(CFLAGS) $< -o $@

# clean:
# 	rm -f *.o movieMaker
