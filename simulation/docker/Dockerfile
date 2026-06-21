# ArduCopter SITL image
# Build with: docker compose build
# Run with:   docker compose up sitl
#
# Pin to the release tag that matches real hardware (Matek H743).
# Override at build time: docker compose build --build-arg ARDUPILOT_TAG=...
ARG ARDUPILOT_TAG=Copter-4.6.3

FROM ubuntu:22.04

ARG ARDUPILOT_TAG
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    ARDUPILOT_TAG=${ARDUPILOT_TAG}

# Bootstrap tools only — ArduPilot's prereqs script installs the rest
RUN apt-get update && apt-get install -y --no-install-recommends \
    git sudo wget curl ca-certificates python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ArduPilot's toolchain scripts require a non-root user with passwordless sudo
RUN useradd -m -s /bin/bash ardupilot \
    && echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ardupilot

# Pre-configure tzdata so its postinst never prompts, even when called via sudo
# (sudo strips DEBIAN_FRONTEND from the environment, so we must persist it to
# /etc/environment and also pre-answer debconf before the prereqs script runs)
RUN echo "tzdata tzdata/Areas select Etc" | debconf-set-selections \
    && echo "tzdata tzdata/Zones/Etc select UTC" | debconf-set-selections \
    && ln -snf /usr/share/zoneinfo/UTC /etc/localtime \
    && echo UTC > /etc/timezone \
    && echo "DEBIAN_FRONTEND=noninteractive" >> /etc/environment

USER ardupilot
# $USER must be set — ArduPilot's prereqs script runs `usermod -a -G dialout $USER`
# and Docker build doesn't populate $USER automatically for non-root users.
ENV USER=ardupilot
WORKDIR /home/ardupilot

# Clone at the specified tag/branch with shallow submodules to keep image smaller
RUN git clone \
        --branch "${ARDUPILOT_TAG}" \
        --depth 1 \
        https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git submodule update --init --recursive --depth 1

WORKDIR /home/ardupilot/ardupilot

# Install all build/runtime prerequisites via ArduPilot's official script
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# prereqs script installs some tools to ~/.local/bin
ENV PATH="/home/ardupilot/.local/bin:$PATH"

# Build SITL binary for ArduCopter
RUN ./waf configure --board sitl && ./waf copter

# SERIAL0 / "ELRS link" → TCP 5760
# SERIAL1 / "LTE link"  → TCP 5761
EXPOSE 5760/tcp
EXPOSE 5761/tcp

# Params directory is bind-mounted at runtime
VOLUME /params

COPY --chown=ardupilot:ardupilot entrypoint.sh /home/ardupilot/entrypoint.sh
RUN chmod +x /home/ardupilot/entrypoint.sh

ENTRYPOINT ["/home/ardupilot/entrypoint.sh"]
# Extra sim_vehicle.py flags can be appended via docker-compose command:
CMD []
