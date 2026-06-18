FROM python:3.12-slim
COPY deepsweep/server.py /server.py
WORKDIR /
RUN mkdir -p /jobs
ENV JOB_SERVER_DB=/jobs/jobs.db
ENTRYPOINT ["python", "/server.py"]
CMD ["--ip", "0.0.0.0", "--port", "13338"]
