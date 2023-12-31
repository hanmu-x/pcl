# 检查达梦数据库的一些信息
if("" STREQUAL "$ENV{DM_HOME}")
    message(STATUS "DM_HOME NOT FOUND.")
else()
    MESSAGE(STATUS "FOUND DM_HOME: $ENV{DM_HOME}")
    set(DM8_DPI_ROOT  $ENV{DM_HOME}/drivers/dpi)
    MESSAGE(STATUS "FOUND ${DM8_DPI_ROOT}")
    if(EXISTS "${DM8_DPI_ROOT}")
        set(DM8_DPI_INCLUDE ${DM8_DPI_ROOT}/include)
        set(DM8_DPI_LIBRARY "${DM8_DPI_ROOT}/dmdpi.${STATIC_LIB_SUFFIX}")
        MESSAGE(STATUS "USE DM DPI INCLUDE: ${DM8_DPI_INCLUDE}")
        MESSAGE(STATUS "USE DM DPI LIB: ${DM8_DPI_LIBRARY}")
        include_directories(${DM8_DPI_INCLUDE})
        LINK_LIBRARIES(${DM8_DPI_LIBRARY})
        # file(COPY  ${DM8_DPI_LIBRARY}/*.dll DESTINATION ${EXECUTABLE_OUTPUT_PATH})
        file(GLOB DM8_DYNAMIC_LIBRARIES "${DM8_DPI_ROOT}/*.${DYNAMIC_LIB_SUFFIX}" )
        if(NOT EXISTS ${EXECUTABLE_OUTPUT_PATH})
            file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
        endif()
        foreach(dlf ${DM8_DYNAMIC_LIBRARIES})
            # file(COPY ${dlf} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
            execute_process(COMMAND ${CMAKE_COMMAND} -E  copy "${dlf}" "${EXECUTABLE_OUTPUT_PATH}")
        endforeach()
        add_definitions("-DDM8_ENABLED")
    else()
        message(STATUS "NOT FOUND DM DRIVER DIR.")
    endif()
endif()