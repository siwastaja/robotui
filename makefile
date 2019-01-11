CC = g++
LD = g++

CXXFLAGS = -fpermissive -Wall -Winline -D_GLIBCXX_USE_CXX11_ABI=0 -Wno-narrowing -Wno-write-strings -O3
LDFLAGS = -D_GLIBCXX_USE_CXX11_ABI=0
#-std=c++98

ROBOTUI_OBJ = robotui.o client_memdisk.o sfml_gui.o

all: robotui

%.o: %.cc
	$(CC) -c $(CXXFLAGS) $*.cc -o $*.o
	$(CC) -MM $(CXXFLAGS) $*.cc > $*.d
	@mv -f $*.d $*.d.tmp
	@sed -e 's|.*:|$*.o:|' < $*.d.tmp > $*.d
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

robotui: $(ROBOTUI_OBJ)
	$(LD) $(LDFLAGS) -o robotui $^ -lm -lsfml-network -lsfml-graphics -lsfml-window -lsfml-system

e:
	gedit --new-window makefile `echo "$(ROBOTUI_OBJ)" | sed s/"\.o"/"\.cc"/g` `echo "$(ROBOTUI_OBJ)" | sed s/"\.o"/"\.h"/g` &

