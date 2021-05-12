FROM python:3.9

RUN apt-get -y update \
    && apt-get -y upgrade

WORKDIR /app

COPY ./ ./

RUN pip3 install -r requirements.txt

ENTRYPOINT ["/app/entrypoint.sh"]

CMD ["bash"]
