cmake_minimum_required(VERSION 3.13.1)
include(../../../../nrf/cmake/boilerplate.cmake NO_POLICY_SCOPE)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(bh1749)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
