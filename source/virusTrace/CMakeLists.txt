cmake_minimum_required(VERSION 3.10)

project(virusTrace)

add_executable(virusTrace)

target_sources(virusTrace PRIVATE virusTrace.cpp)

target_link_libraries(virusTrace PUBLIC virusConsoleLib)

if(UNIX AND NOT APPLE)
	target_link_libraries(virusTrace PUBLIC -static-libgcc -static-libstdc++)
endif()

set_property(TARGET virusTrace PROPERTY FOLDER "Virus")
