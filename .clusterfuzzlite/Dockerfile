FROM gcr.io/oss-fuzz-base/base-builder:v1
COPY . $SRC/PX4-Autopilot
RUN apt-get install -y libjpeg8-dev zlib1g-dev
RUN pip3 install --upgrade pip
RUN python3 -m pip install -r $SRC/PX4-Autopilot/Tools/setup/requirements.txt
WORKDIR $SRC/PX4-Autopilot
COPY ./.clusterfuzzlite/build.sh $SRC/
