# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/esp/esp-idf/frameworks/esp-idf-v5.0/components/bootloader/subproject"
  "C:/inverter_esp32/build/bootloader"
  "C:/inverter_esp32/build/bootloader-prefix"
  "C:/inverter_esp32/build/bootloader-prefix/tmp"
  "C:/inverter_esp32/build/bootloader-prefix/src/bootloader-stamp"
  "C:/inverter_esp32/build/bootloader-prefix/src"
  "C:/inverter_esp32/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/inverter_esp32/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/inverter_esp32/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
