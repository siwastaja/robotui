CC = g++
LD = g++

SFML_INCLUDE = /home/hrst/SFML-2.5.1-linux/include
SFML_LIB     = /home/hrst/SFML-2.5.1-linux/lib
TGUI_INCLUDE = /home/hrst/tgui-0.8.5-linux/include
TGUI_LIB     = /home/hrst/tgui-0.8.5-linux/lib

CXXFLAGS = -fpermissive -Wall -Wno-narrowing -Wno-write-strings -O2 -I$(SFML_INCLUDE) -I$(TGUI_INCLUDE)
LDFLAGS = -L$(SFML_LIB) -L$(TGUI_LIB)
#-std=c++14
#-std=c++98
#-D_GLIBCXX_USE_CXX11_ABI=0

ROBOTUI_OBJ = robotui.o client_memdisk.o sfml_gui.o

all: robotui

$(ROBOTUI_OBJ): %.o: %.cc
	$(CC) -c $(CXXFLAGS) $< -o $@

voxmap.o:
	gcc -c ../robotsoft/voxmap.c -O2 -Wall -o ./voxmap.o

voxmap_memdisk.o:
	gcc -c ../robotsoft/voxmap_memdisk.c -O2 -Wall -o ./voxmap_memdisk.o

robotui: $(ROBOTUI_OBJ) voxmap.o voxmap_memdisk.o
	$(LD) $(LDFLAGS) -o robotui $(ROBOTUI_OBJ) voxmap.o voxmap_memdisk.o -lm -ltgui -lsfml-network -lsfml-graphics -lsfml-window -lsfml-system -lGL -lGLEW -lz

e:
	gedit --new-window makefile makefile-win64 `echo "$(ROBOTUI_OBJ)" | sed s/"\.o"/"\.cc"/g` `echo "$(ROBOTUI_OBJ)" | sed s/"\.o"/"\.h"/g` vertex_shader.glsl fragment_shader.glsl &

# C++ libraries, such as SFML and TGUI, do not provide robust ABI, even within the same version numbers.
# For example, there are at least 3 different C++ string ABIs for linux during the last decade, and at least three different exception models for Windows;
# exact same compiler version is needed to build the libraries and the application. Hence, shared libraries in /usr/local/lib or similar
# do not work, except by luck, because shared libraries with exact same version numbers may not be binary compatible.
# (This is why C++ libraries are very rare, indeed, and most libraries are written in C.)
# Since we have a dependency to two C++ libraries, we have no choice but to provide a script to run the program
# with the exact libraries.

# Note: \044 is $. None of '$', "\$", or '\$" work in echo > file.


release: robotui
#	mkdir release-linux
#	rm ./release-linux/*
	cp robotui vertex_shader.glsl fragment_shader.glsl arial.ttf ./release-linux/
	cp $(SFML_LIB)/libsfml-network* $(SFML_LIB)/libsfml-graphics* $(SFML_LIB)/libsfml-window* $(SFML_LIB)/libsfml-system* ./release-linux/
	cp $(TGUI_LIB)/libtgui* ./release-linux/
	echo '#!/bin/bash' > ./release-linux/run_ui.sh
	echo 'export LD_LIBRARY_PATH=.:\044LD_LIBRARY_PATH' >> ./release-linux/run_ui.sh
	echo 'exec ./robotui \044*' >> ./release-linux/run_ui.sh
	chmod u+x ./release-linux/run_ui.sh
clean:
	rm *.o
	rm robotui
