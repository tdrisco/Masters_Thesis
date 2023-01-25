TARGETS:= phaseVariable_Calculator
OBJLIBS	= xspublic
INCLUDE=-I. -Ixspublic
CFLAGS=-g $(INCLUDE)
CXXFLAGS=-std=c++11 $(CFLAGS)
LFLAGS=-Lxspublic/xscontroller -Lxspublic/xscommon -Lxspublic/xstypes -lxscontroller -lxscommon -lxstypes -lpthread -lrt -ldl

.PHONY: $(OBJLIBS)
all : $(OBJLIBS) $(TARGETS)

xspublic :
	$(MAKE) -C xspublic $(MFLAGS)

phaseVariable_Calculator: phaseVariable_Calculator.o


$(TARGETS):
	$(CXX) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LFLAGS)

%.cpp.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $< -o $@

clean :
	-$(RM) $(OBJECTS) $(TARGETS)
	-$(RM) *.o
	-$(MAKE) -C xspublic/xscontroller $(MFLAGS) clean
	-$(MAKE) -C xspublic/xscommon $(MFLAGS) clean
	-$(MAKE) -C xspublic/xstypes $(MFLAGS) clean
