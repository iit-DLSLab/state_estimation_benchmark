# Monitoring-only guarantee: the package must not contain any interface able to
# command the robot. Fails if a command message type or Unitree command topic
# appears in the sources, launch or configuration files.
file(GLOB_RECURSE files
  ${SRC_DIR}/src/*.cpp ${SRC_DIR}/src/*.hpp ${SRC_DIR}/include/*.hpp
  ${SRC_DIR}/launch/*.py ${SRC_DIR}/config/*.yaml)
set(forbidden "LowCmd" "low_cmd" "/lowcmd" "MotorCmd" "motor_cmd" "SportModeCmd" "/api/sport/request")
set(failed FALSE)
foreach(f ${files})
  file(READ ${f} content)
  foreach(pattern ${forbidden})
    string(FIND "${content}" "${pattern}" pos)
    if (NOT pos EQUAL -1)
      message(SEND_ERROR "Actuation interface '${pattern}' found in ${f}")
      set(failed TRUE)
    endif()
  endforeach()
endforeach()
list(LENGTH files n)
if (n EQUAL 0)
  message(FATAL_ERROR "no files checked (wrong SRC_DIR?)")
endif()
if (failed)
  message(FATAL_ERROR "monitoring-only check failed")
endif()
message(STATUS "monitoring-only check passed (${n} files)")
