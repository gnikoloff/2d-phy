build:
	emcc -lembind -O3 -s TOTAL_MEMORY=83886080 -o quick_example.js ./src/EmscriptenBindings.cpp --embind-emit-tsd interface.d.ts

run:
	./src

clean:
	rm src
