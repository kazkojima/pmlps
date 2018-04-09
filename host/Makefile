obj= main.o unfish.o findframe.o mavlink_vo.o yaw.o fish.o
src= $(obj:.cpp=.o)

BUILDDIR=build
bobj= $(addprefix $(BUILDDIR)/, $(obj))

CFLAGS= -g -O2
LIBINC= -I../../versor/include/vsr -I../../c_library_v2 -I/usr/local/include/opencv2 -I/usr/local/include/opencv
VSRLIB= -L../../versor/build/ -lvsr
CVLIB= -L/usr/local/lib -lopencv_core -lopencv_video

all: $(BUILDDIR)/pmlps

$(bobj): | $(BUILDDIR)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(bobj): $(BUILDDIR)/%.o : %.cpp
	g++ -std=c++11 -c $(CFLAGS) $(LIBINC) -o $@ $<

$(bobj): pmlps.h config.h

$(BUILDDIR)/pmlps: $(bobj)
	g++ $(CFLAGS) -o $@ $(bobj) $(VSRLIB) $(CVLIB) -lm -lpthread

clean:
	rm -f *~ $(BUILDDIR)/*.o $(BUILDDIR)/pmlps
