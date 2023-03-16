cmake_minimum_required(VERSION 3.1)

function(SET_CXX_STANDARD TARGET)
    target_compile_features(${TARGET} PUBLIC cxx_std_14)
    set_target_properties(${TARGET} PROPERTIES CXX_EXTENSIONS OFF)
endfunction()

# For vcproj grouping in visual studio.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

project(vectorizer)

# Silence MSVC security warning (for all projects).
if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# External projects/Dependencies:
# tinyobjloader
add_subdirectory(deps/tinyobjloader)
set_target_properties(tinyobjloader PROPERTIES FOLDER deps)

###########################################################################
# vectorizer
set(VECTORIZER_SRC
    src/api.h
    src/allocator.h
    src/assertion.h
    src/blockAllocator.h
    src/blockAllocator.cpp
	src/cast.h
    src/dynamicArray.h
    src/diff.h
    src/diff.cpp
    src/maths.h
    src/poolArray.h
    src/convexHull.h
    src/convexHull.cpp
    src/polygonShared.h
    src/polygonShared.cpp
    src/polygon.h
    src/polygon.cpp
    src/polygonGrad.h
    src/polygonGrad.cpp
    src/vbvhShared.h
    src/vbvhShared.cpp
	src/vbvh.h
	src/vbvh.cpp
    src/vbvhGrad.h
    src/vbvhGrad.cpp
    src/spatialGrid.h
    src/spatialGrid.cpp
    src/vhiz.h
    src/vhiz.cpp
    src/vectorizer.h
    src/vectorizer.cpp
    src/util.h
)

add_library(vectorizer STATIC
    ${VECTORIZER_SRC}
)

add_library(vectorizerShared SHARED
	${VECTORIZER_SRC}
)

SET_CXX_STANDARD(vectorizer)
SET_CXX_STANDARD(vectorizerShared)

target_include_directories(vectorizer PRIVATE ${CMAKE_CURRENT_LIST_DIR}/deps)
target_include_directories(vectorizerShared PRIVATE ${CMAKE_CURRENT_LIST_DIR}/deps)

# Enable more warnings (compiler-specific)
target_compile_options(vectorizer PRIVATE
                        $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>: -Wall -Wextra>
                        $<$<CXX_COMPILER_ID:MSVC>: /W4>)
target_compile_options(vectorizerShared PRIVATE
                        $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>: -Wall -Wextra>
                        $<$<CXX_COMPILER_ID:MSVC>: /W4>)
if(MSVC)
set_target_properties(vectorizer PROPERTIES COMPILE_DEFINITIONS VECTORIZER_STATIC)
set_target_properties(vectorizerShared PROPERTIES COMPILE_DEFINITIONS VECTORIZER_EXPORTS)
endif()

# Stupid: Copy the latest headers and binaries to mitsuba dependency directory...
# Must create the directories first or this command will fail...
# Headers (Only copy "public interface" files)
add_custom_command(TARGET vectorizerShared POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_LIST_DIR}/src/api.h
    ${CMAKE_CURRENT_LIST_DIR}/src/assertion.h
	${CMAKE_CURRENT_LIST_DIR}/src/maths.h
	${CMAKE_CURRENT_LIST_DIR}/src/cast.h
    ${CMAKE_CURRENT_LIST_DIR}/src/diff.h
	${CMAKE_CURRENT_LIST_DIR}/src/vectorizer.h
	${CMAKE_CURRENT_LIST_DIR}/tool/mitsuba-fork/dependencies/include/vectorizer
)
# Binaries
add_custom_command(TARGET vectorizerShared POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
	${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>/vectorizerShared.lib
	${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>/vectorizerShared.dll
	${CMAKE_CURRENT_LIST_DIR}/tool/mitsuba-fork/dependencies/lib/vectorizer/$<CONFIGURATION>
)

###########################################################################
# vectorizerTest
set(VECTORIZER_TEST_SRC
    test/main.cpp
    test/utilTest.cpp
    test/convConvTest.cpp
    test/convHullTest.cpp
	test/convSplitTest.cpp
    test/vbvhTest.cpp
    test/vhizTest.cpp
    test/mtsTest.cpp
    test/diffTest.cpp
    test/vhizTest.cpp
	test/modelLoader.h
	test/modelLoader.cpp
)

add_executable(vectorizerTest
    ${VECTORIZER_TEST_SRC}
)

SET_CXX_STANDARD(vectorizerTest)
target_include_directories(vectorizerTest PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
target_include_directories(vectorizerTest PRIVATE ${CMAKE_CURRENT_LIST_DIR}/deps)
target_link_libraries(vectorizerTest PRIVATE vectorizer)
target_link_libraries(vectorizerTest PRIVATE tinyobjloader)
if(MSVC)
set_target_properties(vectorizerTest PROPERTIES COMPILE_DEFINITIONS VECTORIZER_STATIC)
endif()

add_custom_command(TARGET vectorizerTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/test/assets/ ${CMAKE_CURRENT_BINARY_DIR}/assets
)