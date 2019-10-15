
all:
	@mkdir -p build; rm -f build/CMakeCache.txt
	cd build && cmake .. && $(MAKE) $(PARALLEL_JOBS)

install:
	cd build && $(MAKE) $(PARALLEL_JOBS) install

uninstall:
	cd build && $(MAKE) uninstall

clean:
	-cd build && $(MAKE) clean

