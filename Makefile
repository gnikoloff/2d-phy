build:
	emcc -lembind -O3 -s TOTAL_MEMORY=83886080 -o quick_example.js ./src/EmscriptenBindings.cpp

# build:
# 	g++ -std=c++17 -Wall ./src/*.cpp -lm -o lib

run:
	./src

clean:
	rm src
