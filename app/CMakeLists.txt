#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(gateway_lower LANGUAGES C)

# Note that here, we're adding CMAKE_SOURCE_DIR to the include path for nanopb.
# This is needed because the below call to nanopb_generate_cpp() is using
# 'RELPATH .'
set(NANOPB_OPTIONS "-I${CMAKE_SOURCE_DIR}")
nanopb_generate_cpp(proto_sources proto_headers RELPATH .
    protobuf/time.proto
)
# we need to be able to include generated header files
zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})

target_sources(app PRIVATE ${proto_sources})


# Include files that are common for all modules
add_subdirectory(src/common/dev_id)
add_subdirectory(src/common/message_channel)

# add_subdirectory_ifdef(CONFIG_UPLINK src/modules/uplink)

# Include mandatory module source folders
add_subdirectory(src/modules/core)
add_subdirectory(src/modules/uplink)
add_subdirectory(src/modules/downlink) # convert to optional later (see below)
add_subdirectory(src/modules/systime)
add_subdirectory(src/modules/error)

# Optional modules
# add_subdirectory_ifdef(CONFIG_NOT_EDGE_DEVICE src/modules/dnlink)
