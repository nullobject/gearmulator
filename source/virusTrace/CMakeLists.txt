cmake_minimum_required(VERSION 3.10)

project(virusTrace)

# Regenerated on every build so the commit stamp in a trace describes the
# binary that wrote it. version.cmake only rewrites the header when the commit
# actually changes, so this does not force a rebuild each time.
set(VIRUSTRACE_VERSION_HEADER "${CMAKE_CURRENT_BINARY_DIR}/virusTraceVersion.h")

add_custom_target(virusTraceVersion
	COMMAND ${CMAKE_COMMAND}
		-DGEARMULATOR_DIR=${CMAKE_SOURCE_DIR}
		-DOUTPUT_FILE=${VIRUSTRACE_VERSION_HEADER}
		-P ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake
	BYPRODUCTS ${VIRUSTRACE_VERSION_HEADER}
	COMMENT "Stamping virusTrace with the current commit"
	VERBATIM)

add_executable(virusTrace)

target_sources(virusTrace PRIVATE virusTrace.cpp)

target_include_directories(virusTrace PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(virusTrace PUBLIC virusConsoleLib)

add_dependencies(virusTrace virusTraceVersion)

if(UNIX AND NOT APPLE)
	target_link_libraries(virusTrace PUBLIC -static-libgcc -static-libstdc++)
endif()

set_property(TARGET virusTrace PROPERTY FOLDER "Virus")
