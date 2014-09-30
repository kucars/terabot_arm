

CFLAGS=-fPIC -g -I../include -Iinclude -I/usr/local/Aria/include

ifndef CXX
CXX=c++
endif

all: ./lib/libArTerabotArm.so examples/terabotArmDemo

dist-all: ./lib/libArTerabotArm.so examples/terabotArmDemo ./lib/libArTerabotArm.a docs

clean: FORCE
	-rm ./lib/libArTerabotArm.so
	-rm obj/ArTerabotArm.o
	-rm examples/terabotArmDemo
  

./lib/libArTerabotArm.so: obj/ArTerabotArm.o /usr/local/Aria/lib/libAria.so
	$(CXX) -shared -o $@ $< -L./lib -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt -lm -pthread

./lib/libArTerabotArm.a: obj/ArTerabotArm.o ./lib/libAria.a
	ar -cr $@ obj/ArTerabotArm.o
	ranlib $@

./lib/%: FORCE
	$(MAKE) -C .. lib/$*

#../lib/libAria.so: FORCE
#	$(MAKE) -C .. lib/libAria.so
#
#./lib/libAria.a: FORCE
#	$(MAKE) -C .. lib/libAria.a
#
#../lib/libArNetworking.so: FORCE
#	$(MAKE) -C .. lib/libArNetworking.so
#
#../lib/libArNetworking.a: FORCE
#	$(MAKE) -C .. lib/libArNetworking.a

obj/%.o: src/%.cpp 
	-mkdir -p obj
	$(CXX) -c $(CFLAGS) -o $@ $<

examples/%: examples/%.cpp ./lib/libArTerabotArm.so /usr/local/Aria/lib/libAria.so
	$(CXX) $(CFLAGS) -o $@ $< -L./lib -L/usr/local/Aria/lib -lArTerabotArm -lAria -ldl -lrt -lm -pthread 

examples/mogsAndArmServer: examples/mogsAndArmServer.cpp ./lib/libArTerabotArm.so /usr/local/Aria/lib/libAria.so ./lib/libArNetworking.so ../../Arnl/lib/libMogs.so ../../Arnl/lib/libBaseArnl.so
	$(CXX) $(CFLAGS) -I../ArNetworking/include -I../../Arnl/include -o $@ $< -L./lib -L../../Arnl/lib -lArTerabotArm -lMogs -lBaseArnl -lArNetworking -lAria -ldl -lrt -lm

examples/mogsAndArmServerStatic: examples/mogsAndArmServer.cpp ./lib/libArTerabotArm.a ./lib/libAria.a ./lib/libArNetworking.a ../../Arnl/lib/libMogs.a ../../Arnl/lib/libBaseArnl.a
	$(CXX) $(CFLAGS) -I../ArNetworking/include -I../../Arnl/include -o $@ $^ -L./lib -L../../Arnl/lib -ldl -lrt -lm -pthread

../../Arnl/%: FORCE
	$(MAKE) -C ../../Arnl $*

examples/%Static: examples/%.cpp ./lib/libArTerabotArm.a ./lib/libAria.a
	$(CXX) $(CFLAGS) -o $@ $^ -ldl -lrt -lm -pthread
	strip $@

include Makefile.dep

Makefile.dep: include/ArTerabotArm.h
	$(info Building Makefile.dep)
	$(CXX) $(CFLAGS) -MM src/ArTerabotArm.cpp | awk '$$1 ~ /:/{printf "obj/%s\n", $$0} $$1 !~ /:/' >Makefile.dep

ifndef INSTALL_DIR
INSTALL_DIR=/usr/local/Aria/ArTerabotArm
endif

install:
	dist/install.sh

	
docs/index.html: doxygen.conf include/ArTerabotArm.h src/ArTerabotArm.cpp examples/terabotArmDemo.cpp
	doxygen doxygen.conf

doc: docs/index.html

docs: doc

TIME=$(shell date +'%F %T')
doxygen.conf: doxygen.conf.in 
	sed -e "s/@VERSION@/dev $(TIME)/g" <$< >$@

FORCE:

.PHONY: all clean install
