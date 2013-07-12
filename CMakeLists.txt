cmake_minimum_required (VERSION 2.8)
project (freespace-examples C)

set (PROJECT_VERSION_MAJOR 0)
set (PROJECT_VERSION_MINOR 7)
set (PROJECT_VERSION_PATCH 0)


add_subdirectory(../libfreespace libfreespace)

# This hides the LIB_TYPE selector. We do this because the examples only
# support the default lib type. If you want to select your lib type you
# must build libfreespace seperate from the examples.
set(LIBFREESPACE_LIB_TYPE "${LIBFREESPACE_LIB_TYPE_DEFAULT}" CACHE INTERNAL "")

if (UNIX)
	set(LIBFREESPACE_LIBRARIES ${LIBFREESPACE_LIBRARIES} m)
	set(pthread_lib_name pthread)
elseif (WIN32)
	link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
	set(pthread_lib_name pthreadVSE2)
	# Copy any dlls to the build tree to keep things clean
	FILE(GLOB DLL_LIST "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.dll")
	SET(DLL_DIR "${CMAKE_CURRENT_BINARY_DIR}/dll/")
	FILE(COPY ${DLL_LIST} DESTINATION ${DLL_DIR})
endif()

include_directories(${LIBFREESPACE_INCLUDE_DIRS})

set(APP_CTRL_SRCS src/appControlHandler.c src/appControlHandler.h)
set(MATH_SRCS src/math/quaternion.c src/math/quaternion.h src/math/vec3.h)

add_executable(motion_example src/motion_example.c ${APP_CTRL_SRCS})
target_link_libraries(motion_example ${LIBFREESPACE_LIBRARIES})

add_executable(motion_example_async src/motion_example_async.c ${APP_CTRL_SRCS})
target_link_libraries(motion_example_async ${LIBFREESPACE_LIBRARIES})

add_executable(communication_example src/communication_example.c ${APP_CTRL_SRCS})
target_link_libraries(communication_example ${LIBFREESPACE_LIBRARIES})

add_executable(game3d_example src/game3d_example.c ${APP_CTRL_SRCS} ${MATH_SRCS})
target_link_libraries(game3d_example ${LIBFREESPACE_LIBRARIES} ${pthread_lib_name})
# Add this post-build command so that the exe can find the dll
add_custom_command(TARGET game3d_example
				POST_BUILD
				COMMAND copy $(ProjectDir)dll\\pthreadVSE2.dll $(TargetDir)
				COMMENT "copying pthreadVSE2.dll to $(TargetDir)")


add_executable(firmware_version_example src/firmware_version_example.c ${APP_CTRL_SRCS})
target_link_libraries(firmware_version_example ${LIBFREESPACE_LIBRARIES})

add_executable(enable_mouse_example src/enable_mouse_example.c ${APP_CTRL_SRCS})
target_link_libraries(enable_mouse_example ${LIBFREESPACE_LIBRARIES})

if (CMAKE_COMPILER_IS_GNUCC)
	add_definitions(-Wall) # TODO  should really set cflags
endif()

