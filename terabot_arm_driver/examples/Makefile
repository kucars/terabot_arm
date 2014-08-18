
all: $(patsubst %.cpp,%,$(shell ls *.cpp | grep -v proprietary))

clean: 
	-rm $(patsubst %.cpp,%,$(shell ls *.cpp | grep -v proprietary))
	-rm $(patsubst %.cpp,%Static,$(shell ls *.cpp | grep -v proprietary))

%Static: %.cpp
	$(MAKE) -C .. examples/$*Static

%: %.cpp
	$(MAKE) -C .. examples/$@

.PHONY: all clean
