obj= main.o unfish.o findframe.o
src= $(obj:.cpp=.o)

BUILDDIR=build

CFLAGS= -g -O2
VSRINC= -I../../versor/include/vsr
VSRLIB= -L../../versor/build/ -lvsr

all: pmlps

$(obj) : %.o : %.cpp
	g++ -std=c++11 -c $(CFLAGS) $(VSRINC) -o $(BUILDDIR)/$@ $<

unfish.o : pmlps.h

pmlps: $(obj)
	g++ $(CFLAGS) -o $(BUILDDIR)/$@ $(addprefix $(BUILDDIR)/, $(obj)) $(VSRLIB) -lm

clean:
	rm -f *~ $(BUILDDIR)/*.o $(BUILDDIR)/pmlps
