# Production Dockerfile. Should only be built in Travis after code is built.
FROM uasatucla/px4-simulator-1.10-dev
COPY . /workspace
