# ##############################################################################
# boards/arm/imxrt/ddcnc-ec300/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS imxrt_boot.c imxrt_flexspi_nor_boot.c imxrt_flexspi_nor_flash.c
         imxrt_bringup.c)

if(CONFIG_IMXRT_ENET)
  list(APPEND SRCS imxrt_ethernet.c)
endif()

if(CONFIG_DDCNC_EC300_STEPGEN)
  list(APPEND SRCS imxrt_stepgen.c)

  # The interrupt runs from RAM: keep loops from becoming memcpy() calls
  # into the flash

  set_source_files_properties(
    imxrt_stepgen.c PROPERTIES COMPILE_OPTIONS
                               -fno-tree-loop-distribute-patterns)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS imxrt_reset.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS imxrt_autoleds.c)
else()
  list(APPEND SRCS imxrt_userleds.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_ETC_ROMFS)
  nuttx_add_romfs(
    NAME
    etc
    MOUNTPOINT
    etc
    RCSRCS
    etc/init.d/rc.sysinit
    etc/init.d/rcS
    PATH
    ${CMAKE_CURRENT_BINARY_DIR}/etc)

  target_link_libraries(board PRIVATE romfs_etc)
endif()

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/flash-ocram.ld")
