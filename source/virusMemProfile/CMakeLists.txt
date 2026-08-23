cmake_minimum_required(VERSION 3.10)

project(virusMemProfile)

add_executable(virusMemProfile)

target_sources(virusMemProfile PRIVATE virusMemProfile.cpp)

target_link_libraries(virusMemProfile PUBLIC virusConsoleLib)

if(UNIX AND NOT APPLE)
	target_link_libraries(virusMemProfile PUBLIC -static-libgcc -static-libstdc++)
endif()

set_property(TARGET virusMemProfile PROPERTY FOLDER "Virus")
