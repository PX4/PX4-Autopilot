#FROM px4io/px4-dev-nuttx:latest
FROM slave_base
MAINTAINER Dario Roethlisberger <dario.roethlisberger@gmail.com>

ENV JAVA_TOOL_OPTIONS='-Dfile.encoding=UTF8 -Djava.awt.headless=false'
# ENV USER hitl_user
# ENV UUID 1000
# ENV LOCAL_USER_ID 1000

RUN apt-get install -y \
    ant \
    openjdk-8-jre \
    openjdk-8-jdk \
    vim \
    xvfb \ 
    python-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install pymavlink pyserial

COPY ./slave.jar .
COPY ./entrypoint.sh .
COPY ./hub-ctrl .

RUN mkdir -p /home/hil/

ENTRYPOINT ["./entrypoint.sh"]
