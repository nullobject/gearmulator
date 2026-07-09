cmake_minimum_required(VERSION 3.10)

project(virusReverbProbe)

add_executable(virusReverbProbe)

target_sources(virusReverbProbe PRIVATE virusReverbProbe.cpp)

target_link_libraries(virusReverbProbe PUBLIC virusConsoleLib)

if(UNIX AND NOT APPLE)
	target_link_libraries(virusReverbProbe PUBLIC -static-libgcc -static-libstdc++)
endif()

set_property(TARGET virusReverbProbe PROPERTY FOLDER "Virus")
