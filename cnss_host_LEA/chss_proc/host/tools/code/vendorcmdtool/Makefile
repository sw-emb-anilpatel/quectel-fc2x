CC := $(ATH_CROSS_COMPILE_TYPE)gcc

all:
	$(CC) -g3 -Wall  \
		-I ./libnl-3.2.25/include \
		-I ./libxml2-2.9.8/include \
		-D __linux__ \
		nl_cmd_wrapper.c ven_cmd_tool.c libnl-3.a libnl-genl-3.a libxml2.a -o vendor_cmd_tool -static -lpthread

clean:
	rm -f vendor_cmd_tool
