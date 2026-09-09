if(NOT DEFINED PROGRAM)
  message(FATAL_ERROR "PROGRAM was not provided")
endif()

execute_process(
  COMMAND "${PROGRAM}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE stdout
  ERROR_VARIABLE stderr
)

if(result EQUAL 0)
  message(FATAL_ERROR "retired rinbo_sin_sweep unexpectedly returned success")
endif()

string(CONCAT output "${stdout}" "${stderr}")
if(NOT output MATCHES "rinbo_sin_sweep is retired" OR
   NOT output MATCHES "No ROS node or motor-command publisher was created")
  message(FATAL_ERROR "retired rinbo_sin_sweep message was incomplete: ${output}")
endif()
