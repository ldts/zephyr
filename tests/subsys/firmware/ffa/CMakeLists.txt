# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ffa_core)

target_sources(app PRIVATE src/main.c)
# Reach the subsystem's internal header for white-box testing.
target_include_directories(app PRIVATE
  ${ZEPHYR_BASE}/subsys/firmware/ffa)
