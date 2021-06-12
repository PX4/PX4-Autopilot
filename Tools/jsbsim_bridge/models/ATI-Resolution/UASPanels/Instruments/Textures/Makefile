export PYTHONPATH=/home/curt/Projects/svginstr/lib:$$PYTHONPATH

all: glass-pfd-horiz.png glass-pfd-main.png glass-pfd-parts.png glass-pfd-tapes.png

%.svg:	%.py
	./$<

%.png: %.svg
	/home/curt/Projects/svginstr/bin/svg2png $<