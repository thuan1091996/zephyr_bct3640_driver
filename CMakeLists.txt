# SPDX-License-Identifier: Apache-2.0

# Point to this project's root directory.
set(BCT3640_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "BCT3640 module root directory")

# Subdirectories
add_subdirectory(drivers)

# Include headers
zephyr_include_directories(drivers)