# Copyright (c) 2023 EPAM Systems
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_OPTEE      optee.c)
zephyr_library_sources_ifdef(CONFIG_OPTEE_FFA  optee_ffa.c)
