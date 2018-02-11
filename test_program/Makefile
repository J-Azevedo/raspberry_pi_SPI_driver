IDIR =../include
#CC=gcc
CC=/home/joao/buildroot/buildroot-2017.02.5/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabihf-gcc
#CFLAGS=-I$(IDIR) -g -Walls
CFLAGS= -g 
#ODIR=obj
#LDIR =../lib

#LIBS=-lm

#_DEPS = hellomake.h
#DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = test_program.o 
#OBJ =$(patsubst %,$(ODIR)/%,$(_OBJ))
#OBJ =$(pa$(_OBJ))
test:test_program.o
		$(CC) test_program.o -o test

test_program.o:test_program.c
		$(CC) -c test_program.c 

.PHONY: clean

clean:
	rm -f .o 
