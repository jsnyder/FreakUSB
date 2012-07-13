all: default


default:
	make -C demo/dfu_sim3

clean:
	make -C demo/dfu_sim3 clean
