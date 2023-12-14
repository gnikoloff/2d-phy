build:
	clang++ -Wall -std=c++17 ./src/*.cpp ./src/Physics/*.cpp -I/opt/homebrew/Cellar/sdl2/2.28.4/include -I/opt/homebrew/Cellar/sdl2_image/2.6.3_2/include -I/opt/homebrew/Cellar/sdl2_gfx/1.0.4/include -L/opt/homebrew/Cellar/sdl2/2.28.4/lib -L/opt/homebrew/Cellar/sdl2_image/2.6.3_2/lib -L/opt/homebrew/Cellar/sdl2_gfx/1.0.4/lib -lm -lSDL2 -lSDL2_image -lSDL2_gfx -o physics

run:
	./physics

clean:
	rm physics