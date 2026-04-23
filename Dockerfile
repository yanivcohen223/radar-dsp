FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libfftw3-dev \
    libncurses5-dev \
    libncursesw5-dev \
    libgtest-dev \
    python3 \
    python3-pip \
    python3-numpy \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir matplotlib

# Build GTest from source (Ubuntu packages don't ship pre-built libs)
RUN cd /usr/src/gtest && cmake . && make && \
    cp lib/*.a /usr/lib/ || \
    (cmake /usr/src/googletest -B/tmp/gtest-build && cmake --build /tmp/gtest-build && \
     cp /tmp/gtest-build/lib/*.a /usr/lib/)

WORKDIR /radar-dsp

COPY . .

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_PROFILING=ON && \
    cmake --build build --parallel $(nproc)

CMD ["./build/radar_dsp"]
