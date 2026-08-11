# syntax=docker/dockerfile:1
#
# Prebuilt build environment for panda-py wheels: a manylinux image with
# libfranka and its dependencies already installed.
#
# Building these dependencies from source takes ~7 minutes for libfranka >= 0.14
# (Boost and Pinocchio dominate) and repeating that on every CI run, for every
# matrix row, is the bulk of the pipeline's wall clock. Building an image per
# libfranka version instead and consuming it via cibuildwheel's
# manylinux-x86_64-image setting reduces a wheel build to about a minute.
#
# The build instructions live in bin/before_install_centos.sh so that the
# script and this image cannot drift apart; each stage below is one cached layer.
#
# Build a legacy image (libfranka < 0.14, Poco and Eigen only):
#   docker build -t panda-py-build:0.9.2 \
#     --build-arg LIBFRANKA_VER=0.9.2 .
#
# Build a modern image (libfranka >= 0.14, adds the Pinocchio chain):
#   docker build -t panda-py-build:0.21.3 \
#     --build-arg LIBFRANKA_VER=0.21.3 --build-arg DEPS=modern .
#
# Then build wheels against it:
#   CIBW_MANYLINUX_X86_64_IMAGE=panda-py-build:0.21.3 \
#   CIBW_BEFORE_ALL= \
#   CIBW_ENVIRONMENT=LIBFRANKA_VER=0.21.3 \
#     python -m cibuildwheel --platform linux

# manylinux_2_28 (AlmaLinux 8, glibc 2.28) rather than manylinux2014 (CentOS 7,
# EOL since June 2024): live repos, GCC 14 instead of 10.2, and it matches the
# floor numpy itself ships for, which panda-py depends on. manylinux_2_34 has
# more of these dependencies packaged but requires glibc 2.34, which would drop
# Ubuntu 20.04 and put us ahead of numpy's own floor.
ARG MANYLINUX_IMAGE=quay.io/pypa/manylinux_2_28_x86_64
# "legacy" for libfranka < 0.14.0, "modern" for >= 0.14.0 (adds Pinocchio).
ARG DEPS=legacy

# --------------------------------------------------------------------------
# System packages. Shared by every image, so this layer is built once.
# --------------------------------------------------------------------------
FROM ${MANYLINUX_IMAGE} AS system

# LIBFRANKA_VER is asserted by the script even for stages that do not use it.
# The real value is supplied to the libfranka stage below; using a placeholder
# here keeps these shared layers independent of it, so they stay cached across
# every libfranka version in the matrix.
ENV LIBFRANKA_VER=0.0.0
ENV WORK_DIR=/tmp/build
RUN mkdir -p ${WORK_DIR}

COPY bin/before_install_centos.sh /opt/panda-py/bin/
COPY bin/patches /opt/panda-py/bin/patches
COPY bin/cmake /opt/panda-py/bin/cmake
RUN /opt/panda-py/bin/before_install_centos.sh system \
 && yum clean all && rm -rf /var/cache/yum

# --------------------------------------------------------------------------
# Poco. Identical for every libfranka version, so it is shared too.
# --------------------------------------------------------------------------
FROM system AS poco
RUN /opt/panda-py/bin/before_install_centos.sh poco && rm -rf ${WORK_DIR:?}/*

# --------------------------------------------------------------------------
# Dependency flavours. "modern" adds Boost, TinyXML2, console_bridge, urdfdom,
# Assimp, Pinocchio and fmt. Because this layer does not depend on the exact
# libfranka version, it is built once and reused by every >= 0.14 row.
# --------------------------------------------------------------------------
FROM poco AS deps-legacy

FROM poco AS deps-modern
RUN /opt/panda-py/bin/before_install_centos.sh pinocchio && rm -rf ${WORK_DIR:?}/*

# --------------------------------------------------------------------------
# libfranka. The only layer that varies per matrix row.
# --------------------------------------------------------------------------
FROM deps-${DEPS} AS final
ARG LIBFRANKA_VER
ENV LIBFRANKA_VER=${LIBFRANKA_VER}
RUN test "${LIBFRANKA_VER}" != "0.0.0" \
    || { echo "--build-arg LIBFRANKA_VER=<version> is required" >&2; exit 1; }
RUN /opt/panda-py/bin/before_install_centos.sh libfranka && rm -rf ${WORK_DIR:?}/*

LABEL org.opencontainers.image.source="https://github.com/JeanElsner/panda-py"
LABEL org.opencontainers.image.description="panda-py build environment with libfranka ${LIBFRANKA_VER}"
