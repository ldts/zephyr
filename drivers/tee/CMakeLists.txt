# SPDX-License-Identifier: Apache-2.0

if(CONFIG_OPTEE OR CONFIG_OPTEE_FFA)
    add_subdirectory(optee)
endif()

zephyr_library()
zephyr_library_sources(tee.c)
