FROM px4io/px4-dev-simulation-bionic@sha256:3ccfb2c582ae06b212a50e738356b2063b7627087edd3a6567e4836a68e8c61a

# Install mavlink-router
RUN apt-get update && apt-get install -y python-future python3-future libtool autoconf
RUN git clone https://github.com/intel/mavlink-router.git && \
    cd mavlink-router && \
    git submodule update --init --recursive && \
    ./autogen.sh && ./configure CFLAGS='-g -O2' \
        --sysconfdir=/etc --localstatedir=/var --libdir=/usr/lib64 --disable-systemd \
        --prefix=/usr && \
    make && \
    make install

RUN apt-get update && apt-get install -y python-jinja2

WORKDIR /workspace
