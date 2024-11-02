# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles\\flight_control_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\flight_control_autogen.dir\\ParseCache.txt"
  "flight_control_autogen"
  )
endif()
