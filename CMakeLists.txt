project(BagelMARS)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "test plugin to load and execute a behavior graph.")
include(FindPkgConfig)

find_package(lib_manager)
lib_defaults()
define_module_info()

cmake_minimum_required(VERSION 2.6)

pkg_check_modules(PKGCONFIG
                  lib_manager
                  data_broker
                  cpp_bagel_wrapper
                  c_bagel
                  mars_interfaces
                  mars_utils
                  main_gui
                  configmaps
                  REQUIRED
)
include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CFLAGS_OTHER})  #flags excluding the ones with -I

include_directories(
	src
)

set(SOURCES 
	src/BagelMARS.cpp
)

set(HEADERS
	src/BagelMARS.hpp
)



add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
                      ${PKGCONFIG_LIBRARIES}
)

if(WIN32)
  set(LIB_INSTALL_DIR bin) # .dll are in PATH, like executables
else(WIN32)
  set(LIB_INSTALL_DIR lib)
endif(WIN32)


set(_INSTALL_DESTINATIONS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION lib
)


# Install the library into the lib folder
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${HEADERS} DESTINATION include/mars/plugins/${PROJECT_NAME})

# Prepare and install necessary files to support finding of the library 
# using pkg-config
configure_file(${PROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)


