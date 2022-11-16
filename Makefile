.DEFAULT_GOAL := vasaro-web

RAYLIB_OBJs = \
	build/raylib/rcore.o build/raylib/rshapes.o build/raylib/rtextures.o build/raylib/rtext.o \
	build/raylib/rmodels.o build/raylib/utils.o build/raylib/raudio.o build/raylib/gui/raygui.o

prebuild:
	mkdir -p build/noises && mkdir -p build/raylib/gui

build/raylib/gui/raygui.o : ext/raygui.c
	emcc -c $< -o $@ -Os -DPLATFORM_WEB -Iext/raygui/src -Iext/raylib/src -w

build/raylib/%.o : ext/raylib/src/%.c
	emcc -c $< -o $@ -Os -DPLATFORM_WEB -DGRAPHICS_API_OPENGL_ES2 -w

build/libraylib.a: $(RAYLIB_OBJs)
	emar rcs build/libraylib.a $(RAYLIB_OBJs)

build/vasaro.o : src/vasaro.c src/rlights.h build/generator.o
	emcc -c $< -o $@ -Os -Wall -DPLATFORM_WEB -Iext/raylib/src/ -Iext/raygui/src/

build/hashmap.o : src/hashmap.c src/hashmap.h
	emcc -c $< -o $@ -Os -Wall -DPLATFORM_WEB -Iext/raylib/src/ -Iext/raygui/src/

build/opensimplexnoise.o: src/noises/opensimplexnoise.c src/noises/opensimplexnoise.h
	emcc -c $< -o $@ -Os -Wall -DPLATFORM_WEB

build/simplexnoise.o: src/noises/simplexnoise.c src/noises/simplexnoise.h
	emcc -c $< -o $@ -Os -Wall -DPLATFORM_WEB

build/generator.o: src/generator.c src/generator.h build/simplexnoise.o build/hashmap.o build/opensimplexnoise.o
	emcc -c $< -o $@ -Os -Wall -DPLATFORM_WEB


vasaro-web: prebuild build/generator.o build/libraylib.a build/vasaro.o
	emcc -o html/vasaro.html build/*.o build/libraylib.a -Os -Wall -I. -L. -sALLOW_MEMORY_GROWTH -s ASSERTIONS=1 -s USE_GLFW=3 -s TOTAL_MEMORY=67108864 -s FORCE_FILESYSTEM=1 -DPLATFORM_WEB -s EXPORTED_RUNTIME_METHODS=["cwrap"] -s EXPORTED_FUNCTIONS='["_main", "_resize"]' --preload-file resources@resources

clean:
	rm -rf build/*

