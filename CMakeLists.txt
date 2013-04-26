cmake_minimum_required (VERSION 2.6)
project (freespace-examples C)

set (PROJECT_VERSION_MAJOR 0)
set (PROJECT_VERSION_MINOR 7)
set (PROJECT_VERSION_PATCH 0)


add_subdirectory(../libfreespace libfreespace)

if (UNIX)
	set(LIBFREESPACE_LIBRARIES ${LIBFREESPACE_LIBRARIES} m)
	set(pthread_lib_name pthread)
elseif (WIN32)
	link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
	set(pthread_lib_name pthreadVSE2)
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

add_executable(multiple_device_example src/multiple_device_example.c ${APP_CTRL_SRCS})
target_link_libraries(multiple_device_example ${LIBFREESPACE_LIBRARIES})

add_executable(gameloop_example src/gameloop_example.c ${APP_CTRL_SRCS})
target_link_libraries(gameloop_example ${LIBFREESPACE_LIBRARIES} ${pthread_lib_name})

add_executable(gameloop_example2 src/gameloop_example2.c ${APP_CTRL_SRCS})
target_link_libraries(gameloop_example2 ${LIBFREESPACE_LIBRARIES})

add_executable(game3d_example src/game3d_example.c ${APP_CTRL_SRCS} ${MATH_SRCS})
target_link_libraries(game3d_example ${LIBFREESPACE_LIBRARIES} ${pthread_lib_name})

add_executable(firmware_version_example src/firmware_version_example.c ${APP_CTRL_SRCS})
target_link_libraries(firmware_version_example ${LIBFREESPACE_LIBRARIES})

if (CMAKE_COMPILER_IS_GNUCC) 
	add_definitions(-Wall) # TODO  should really set cflags
endif()

