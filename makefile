CC = g++
LD = g++

CXXFLAGS = -fpermissive -Wall -Winline -D_GLIBCXX_USE_CXX11_ABI=0 -Wno-narrowing -Wno-write-strings -O2 -g
LDFLAGS = -D_GLIBCXX_USE_CXX11_ABI=0
#-std=c++98

ROBOTUI_OBJ = robotui.o client_memdisk.o sfml_gui.o

all: robotui

$(ROBOTUI_OBJ): %.o: %.cc
	$(CC) -c $(CXXFLAGS) $< -o $@

voxmap.o:
	gcc -c ../robotsoft/voxmap.c -O2 -Wall -o ./voxmap.o

voxmap_memdisk.o:
	gcc -c ../robotsoft/voxmap_memdisk.c -O2 -Wall -o ./voxmap_memdisk.o

robotui: $(ROBOTUI_OBJ) voxmap.o voxmap_memdisk.o
	$(LD) $(LDFLAGS) -o robotui $(ROBOTUI_OBJ) voxmap.o voxmap_memdisk.o -lm -lsfml-network -lsfml-graphics -lsfml-window -lsfml-system -lGL -lGLEW -lz

e:
	gedit --new-window makefile `echo "$(ROBOTUI_OBJ)" | sed s/"\.o"/"\.cc"/g` `echo "$(ROBOTUI_OBJ)" | sed s/"\.o"/"\.h"/g` vertex_shader.glsl fragment_shader.glsl &

clean:
	rm *.o
	rm robotui
