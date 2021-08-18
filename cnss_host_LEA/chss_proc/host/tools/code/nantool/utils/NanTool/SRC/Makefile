#INCDIR=-I. -I../inc -I./inc
INCDIR=-I. -I./inc
OBJ=halProxyDaemon.o nan_test.o
#LD=-L../lib_src -L/lib/i386-linux-gnu 
LD=-L/lib/i386-linux-gnu -L./LIB 
all: Nandemotool 

Nandemotool: $(OBJ)
	g++ $(CFLAGS) $(INCDIR) $(LD) $^ -lwifi-hal-qcom -lpthread -lstdc++ -Wl,--no-as-needed -ldl -lnl-3 -lnl-genl-3 -lbsd -o $@

%.o: %.cpp
	g++ -c  $(LD) $(CFLAGS) -D"NAN_CERT_VERSION=4" -Wformat=0 $(INCDIR) $< -o $@ 
clean:
	rm Nandemotool;rm *.o
