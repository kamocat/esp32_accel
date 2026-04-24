# Usage: cmake -DINPUT=<file> -DOUTPUT=<file> -P gzip_file.cmake
execute_process(
    COMMAND gzip -9 -c "${INPUT}"
    OUTPUT_FILE "${OUTPUT}"
    RESULT_VARIABLE result
)
if(result)
    message(FATAL_ERROR "gzip failed with exit code ${result}")
endif()
