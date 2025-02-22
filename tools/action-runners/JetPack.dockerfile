# Base Image
FROM ghcr.io/missourimrdt/autonomy-jetpack:2024-11-27-21-06-51

# Install Variables
ARG L4T_MAJOR="36"
ARG L4T_MINOR="2"
ARG L4T_PATCH="0"
ARG L4T_BASE="l4t-jetpack"
ARG RUNNER_VERSION="2.321.0"
ENV DEBIAN_FRONTEND=noninteractive
ENV RUNNER_ALLOW_RUNASROOT=1

# Set Labels
LABEL authors="Missouri S&T Mars Rover Design Team"
LABEL maintainer="Mars Rover Design Team <marsrover@mst.edu>"
LABEL org.opencontainers.image.source=https://github.com/missourimrdt/autonomy_software
LABEL org.opencontainers.image.licenses=GPL-3.0-only
LABEL org.opencontainers.image.version="v24.5.0"
LABEL org.opencontainers.image.description="Docker Image for ${L4T_BASE} ${L4T_MAJOR}.${L4T_MINOR}.${L4T_PATCH} with Runner ${RUNNER_VERSION}."

# Uupdate the base packages + add a non-sudo user
RUN apt-get update -y && apt-get upgrade -y && useradd -m docker

# Set Working Directory
WORKDIR /home/docker

# Install the packages and dependencies along with jq so we can parse JSON (add additional packages as necessary)
RUN apt-get install -y --no-install-recommends \
    curl gnutls-bin openssl nodejs wget unzip vim git jq build-essential libssl-dev libffi-dev python3 python3-venv python3-dev python3-pip docker.io \
    libjpeg-dev libjpeg-turbo8-dev libturbojpeg git-lfs ccache

# Change into the user directory, download and unzip the github actions runner
RUN cd /home/docker && mkdir actions-runner && cd actions-runner \
    && curl -O -L https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/actions-runner-linux-arm64-${RUNNER_VERSION}.tar.gz \
    && tar xzf ./actions-runner-linux-arm64-${RUNNER_VERSION}.tar.gz

# Set Working Directory
WORKDIR /home/docker/actions-runner

# Install some additional dependencies
RUN chown -R docker ~docker && /home/docker/actions-runner/bin/installdependencies.sh

# Prevent SSH repo timout for an hour.
RUN mkdir -p /root/.ssh && echo "Host *\n  ServerAliveInterval 60\n  ServerAliveCountMax 60" >> ~/.ssh/config

# Remove Autonomy Software
RUN rm -rf /opt/Autonomy_Software

# Set Ownership of Autonomy Software
RUN chown docker -R /usr/local/zed
RUN chown docker -R /usr/local/cuda

# Add over the start.sh script
COPY scripts/start.sh start.sh

# Make the script executable
RUN chmod +x start.sh

# Set the entrypoint to the start.sh script
ENTRYPOINT ["./start.sh"]
