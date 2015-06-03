
SEC2PATH = ../../t23xsec2

CC = $(CROSS_COMPILE)gcc
CCFLAGS += -I$(SEC2PATH)

LD = $(CROSS_COMPILE)gcc
RM = rm



all:	secexamples

clean:
	$(RM) $(RMFLAGS) $(OBJS) secexamples

OBJS = example_base.o aes.o des.o pk.o rng.o hash.o arc4.o kasumi.o


secexamples:	$(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS)


example_base.o:	example_base.c

aes.o:		aes.c examples.h $(SEC2PATH)/Sec2.h

rng.o:		rng.c examples.h $(SEC2PATH)/Sec2.h

hash.o:		hash.c examples.h $(SEC2PATH)/Sec2.h

pk.o:		pk.c examples.h $(SEC2PATH)/Sec2.h

des.o:		des.c examples.h $(SEC2PATH)/Sec2.h

arc4.o:		arc4.c examples.h $(SEC2PATH)/Sec2.h

kasumi.o:	kasumi.c examples.h $(SEC2PATH)/Sec2.h


%.o:	%.c
	$(CC) $(CCFLAGS) -c -o $@ $<
