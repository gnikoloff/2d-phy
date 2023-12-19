build:
	emcc -lembind -O3 -sALLOW_MEMORY_GROWTH -s MODULARIZE=1 -s EXPORT_ES6=1 -sENVIRONMENT="web" -o Phy2D.js ./src/EmscriptenBindings.cpp --embind-emit-tsd Phy2D.d.ts

run:
	./src

clean:
	rm src
