# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/nds/esp/esp-idf/components/bootloader/subproject"
  "/home/nds/Programs/c-workspace/esp32-gy291-i2c/build/bootloader"
  "/home/nds/Programs/c-workspace/esp32-gy291-i2c/build/bootloader-prefix"
  "/home/nds/Programs/c-workspace/esp32-gy291-i2c/build/bootloader-prefix/tmp"
  "/home/nds/Programs/c-workspace/esp32-gy291-i2c/build/bootloader-prefix/src/bootloader-stamp"
  "/home/nds/Programs/c-workspace/esp32-gy291-i2c/build/bootloader-prefix/src"
  "/home/nds/Programs/c-workspace/esp32-gy291-i2c/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/nds/Programs/c-workspace/esp32-gy291-i2c/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/nds/Programs/c-workspace/esp32-gy291-i2c/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
