#
# Copyright (c) 2023 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# Include dev_id.h
target_include_directories(app PRIVATE .)

target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dev_id.c)