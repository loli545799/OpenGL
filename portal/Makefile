FLAG_LIB		=	./glfw3.dll ./lib/libassimp.dll.a -lm  
FLAG_INCLUDE	=	-I ./include
SRC_UTIL		=	./src/glad.c ./src/utils.cpp ./src/model.cpp ./src/portal.cpp ./src/stb_image.cpp ./src/scene_zzk.cpp ./src/scene_xjj.cpp ./src/Body.cpp ./src/CollideFine.cpp ./src/Contacts.cpp ./src/mathematics.cpp
FLAGS			=	$(FLAG_LIB) $(FLAG_INCLUDE) $(SRC_UTIL) -O3

SRC_MAIN		=	./src/main.cpp

target: $(SRC_MAIN)
	g++ $< $(FLAGS)

static: $(SRC_MAIN)
	g++ $< $(FLAGS) -static

debug: $(SRC_MAIN)
	g++ $< $(FLAGS) -g


.PHONY:
run: a.exe
	./a.exe 

.PHONY:
clean: 
	del a.exe