name=easystar
jsbsim_root=../..

all: Datcom/datcom_aero.xml

clean:
	rm -rf *.sce *.log Datcom/fort.* Datcom/*.dat \
		Datcom/*.lfi Datcom/*.csv Datcom/*.ac Datcom/*.jiff Datcom/datcom.xml Datcom/*.out

dist-clean: clean
	rm -rf Datcom/datcom_aero.xml

Datcom/datcom_aero.xml: Datcom/datcom.dcm
	quilt pop -a; \
	cd Datcom; \
	datcom datcom.dcm; \
	cd ..; \
	quilt push -a 

Datcom/datcom.jiff: Datcom/datcom_aero.xml
	cd Datcom; \
	jiff datcom_aero.xml; 

plot: Datcom/datcom.jiff
	nautilus $^

install-deps:
	sudo apt-get install gnuplot

jsb-fg-test:
	${jsbsim_root}/scripts/fgrunExt.sh ${name}; \
    sleep 10; \
	jsbsim-cmd --root=${jsbsim_root} --realtime --script=aircraft/${name}/Scripts/cruise.xml --logdirectivefile=data_output/flightgear.xml

jsb-test:
	jsbsim-cmd --root=${jsbsim_root} --realtime --script=aircraft/${name}/Scripts/cruise.xml

manual-test:
	fgfs --heading=120 --airport=KSFO --aircraft=${name}

format:
	find . -type f -regex ".*\.xml" -exec xmlindent -w {} \;; \
	find . -type f -regex ".*\.xml~" -exec rm {} \;

.PHONY: plot clean dist-clean jsb-test manual-test jsb-fg-test format install-deps

