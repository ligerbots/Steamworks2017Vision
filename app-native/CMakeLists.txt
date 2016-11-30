project(app-native)

cmake_minimum_required(VERSION 3.4.1)

file(GLOB_RECURSE SRC_FILES src/main/cpp/*.cpp)
include_directories(src/main/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fcxx-exceptions -std=c++1y -Wformat=2 -Wall -Wextra -Werror -pedantic -Wno-unused-parameter")

# Main JNI library
add_library( app-native
             SHARED
             ${SRC_FILES} )

# import NDK log library
find_library( log-lib
              log )

# add opencv
add_library( libopencv_java3
             SHARED
             IMPORTED )

set_target_properties( libopencv_java3
                       PROPERTIES IMPORTED_LOCATION
                       ${CMAKE_SOURCE_DIR}/../app/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so )

# add ffmpeg
add_library( libx264
             SHARED
             IMPORTED )

set_target_properties( libx264
                       PROPERTIES IMPORTED_LOCATION
                       ${CMAKE_SOURCE_DIR}/../app/src/main/jniLibs/${ANDROID_ABI}/libx264.so )

add_library( libavutil
             SHARED
             IMPORTED )

set_target_properties( libavutil
                       PROPERTIES IMPORTED_LOCATION
                       ${CMAKE_SOURCE_DIR}/../app/src/main/jniLibs/${ANDROID_ABI}/libavutil.so )

add_library( libavformat
             SHARED
             IMPORTED )

set_target_properties( libavformat
                       PROPERTIES IMPORTED_LOCATION
                       ${CMAKE_SOURCE_DIR}/../app/src/main/jniLibs/${ANDROID_ABI}/libavformat.so )

add_library( libavcodec
             SHARED
             IMPORTED )

set_target_properties( libavcodec
                       PROPERTIES IMPORTED_LOCATION
                       ${CMAKE_SOURCE_DIR}/../app/src/main/jniLibs/${ANDROID_ABI}/libavcodec.so )

# link everything
target_link_libraries( app-native
                       ${log-lib} libopencv_java3 libx264 libavutil libavformat libavcodec )
