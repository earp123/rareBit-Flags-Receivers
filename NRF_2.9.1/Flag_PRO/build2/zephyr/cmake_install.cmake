# Install script for directory: C:/ncs/v2.9.1/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/Zephyr-Kernel")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/ncs/toolchains/b620d30767/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/arch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/lib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/boards/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/subsys/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/drivers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/nrf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/mcuboot/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/trusted-firmware-m/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/cjson/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/azure-sdk-for-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/cirrus-logic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/suit-processor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/memfault-firmware-sdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/canopennode/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/chre/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/lz4/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/nanopb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/zscilib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/cmsis/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/cmsis-dsp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/cmsis-nn/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/hal_nordic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/hal_st/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/hal_wurthelektronik/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/hostap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/libmetal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/liblc3/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/littlefs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/loramac-node/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/lvgl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/mipi-sys-t/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/nrf_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/open-amp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/picolibc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/segger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/tinycrypt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/uoscore-uedhoc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/zcbor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/nrfxlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/nrf_hw_models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/modules/connectedhomeip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/kernel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/cmake/flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/cmake/usage/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/ralls/GitHub/rareBit-Flags-Receivers/NRF_2.9.1/Flag_PRO/build2/zephyr/cmake/reports/cmake_install.cmake")
endif()

