project(ntcore)

cmake_minimum_required(VERSION 3.4.1)

file(GLOB_RECURSE SRC_FILES src/main/cpp/*.cpp)
include_directories(src/main/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fcxx-exceptions -std=c++1y -Wformat=2 -Wall -Wextra -Werror -pedantic -Wno-unused-parameter")

# Main JNI library
add_library( ntcore
             SHARED
             ${SRC_FILES} )

# import NDK log library
find_library( log-lib
              log )

# link everything
target_link_libraries( ntcore
                       ${log-lib} )