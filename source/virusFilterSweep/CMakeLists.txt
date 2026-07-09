cmake_minimum_required(VERSION 3.10)

project(virusFilterSweep)

add_executable(virusFilterSweep)

target_sources(virusFilterSweep PRIVATE virusFilterSweep.cpp)

target_link_libraries(virusFilterSweep PUBLIC virusConsoleLib)

if(UNIX AND NOT APPLE)
	target_link_libraries(virusFilterSweep PUBLIC -static-libgcc -static-libstdc++)
endif()

set_property(TARGET virusFilterSweep PROPERTY FOLDER "Virus")
