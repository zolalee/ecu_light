CC=arm-linux-gnueabihf-gcc
CXX=arm-linux-gnueabihf-g++

# 编译器在编译时的参数设置,包含头文件路径设置
CFLAGS:=
CFLAGS+=
CXXFLAGS:=-I . -I ./RosLibs
CXXFLAGS+=

# 库文件添加
LDFLAGS:=-lpthread -lstdc++
LDFLAGS+=

# 指定源程序存放位置
SRCDIRS:=.
SRCDIRS+=RosLibs

# 设置程序中使用文件类型
SRCEXTS:=.c .cpp

# 设置运行程序名
PROGRAM:=gingerlite-ecu

SOURCES=$(foreach d,$(SRCDIRS),$(wildcard $(addprefix $(d)/*,$(SRCEXTS))))
OBJS=$(foreach x,$(SRCEXTS),$(patsubst %$(x),%.o,$(filter %$(x),$(SOURCES))))

.PHONY: all clean distclean install

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<
	
%.o: %.cxx
	$(CXX) -c $(CXXFLAGS) -o $@ $<


$(PROGRAM): $(OBJS)
ifeq ($(strip $(SRCEXTS)),.c)
	$(CC) -o $(PROGRAM) $(OBJS) $(LDFLAGS)
else
	$(CXX) -o $(PROGRAM) $(OBJS) $(LDFLAGS)
endif


install:
	install -m 755 -D -p $(PROGRAM) ./bin

clean:
	rm -f $(shell find -name "*.o")
	rm -f $(PROGRAM)

distclean:
	rm -f $(shell find -name "*.o")
	rm -f $(shell find -name "*.d")
	rm -f $(PROGRAM)

all:
	@echo $(OBJS)



