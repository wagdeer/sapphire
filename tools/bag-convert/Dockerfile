FROM python:3.11-slim

ARG ROSBAGS_VERSION=0.11.9

RUN python -m pip install --no-cache-dir \
    "rosbags==${ROSBAGS_VERSION}"

ENTRYPOINT ["rosbags-convert"]
