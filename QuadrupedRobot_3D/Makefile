# Makefile for ODE

LIB = -lode -ldrawstuff -lGL -lGLU -lX11 -lrt -lm -lpthread -lstdc++
LIBDIR = -L/usr/lib/x86_64-linux-gnu -L../../drawstuff/src/.libs  -L../../ode/src/.libs -L/usr/lib 
INCDIR = -I../../include -I../../ode/src -I/usr/include

OBJS = main

$(OBJS): $(OBJS).o
	gcc -g -O2 -o $(OBJS) $(OBJS).o $(INCDIR) $(LIBDIR) $(LIB) 

$(OBJS).o: $(OBJS).c
	gcc -c $(OBJS).c $(INCDIR) $(LIBDIR) $(LIB) 

.PHONY: clean
clean:
	 rm -f $(OBJS) $(OBJS).o 
