# Runs popf on DOMAIN and PROBLEM and checks the outcome against EXPECT (plan|unsolvable).
# popf returns 0 in both cases, so the result is read from its output.

execute_process(
  COMMAND ${POPF} ${DOMAIN} ${PROBLEM}
  OUTPUT_VARIABLE output
  ERROR_VARIABLE output
  RESULT_VARIABLE result
  TIMEOUT 60)

if(NOT result EQUAL 0)
  message(FATAL_ERROR "popf failed (${result}):\n${output}")
endif()

string(FIND "${output}" "Solution Found" found)
if(EXPECT STREQUAL "plan" AND found EQUAL -1)
  message(FATAL_ERROR "popf found no plan:\n${output}")
endif()
if(EXPECT STREQUAL "unsolvable" AND NOT found EQUAL -1)
  message(FATAL_ERROR "popf found a plan for an unsolvable problem:\n${output}")
endif()
