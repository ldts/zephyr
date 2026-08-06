# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(ffa_core.c ffa_msg.c)
zephyr_library_sources_ifdef(CONFIG_ARM_FFA_MEM_SHARE ffa_mem.c)
zephyr_include_directories(.)
