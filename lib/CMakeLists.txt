# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(
    ${Boost_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBUSB_INCLUDE_DIR}
)
link_directories(${Boost_LIBRARY_DIRS} ${LIBUSB_LIBRARY_DIRS})


#set_target_properties(ssrptx_shared PROPERTIES DEFINE_SYMBOL "ssrptx_EXPORTS")
#set_target_properties(ssrptx_shared PROPERTIES OUTPUT_NAME ssrptx)
#set_target_properties(ssrptx_shared PROPERTIES SOVERSION 0 VERSION 0.0.0)


list(APPEND ssrptx_sources
    ssrptx_impl.cc
    fusb_linux.cc
    fusb.cc
    sdrtx_prims.cc
    sdrtx_standard.cc
    sdrtx_basic.cc
)

if (WIN32)
	list(APPEND ssrptx_sources
		fusb_generic.cc
		fusb_sysconfig_generic.cc
	)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	list(APPEND ssrptx_sources
		fusb_darwin.cc
		fusb_sysconfig_generic.cc
	)
elseif (NOT WIN32)
	list(APPEND ssrptx_sources
		fusb_linux.cc
		fusb_sysconfig_linux.cc
  	)
endif()


add_library(gnuradio-ssrptx SHARED ${ssrptx_sources})
target_link_libraries(gnuradio-ssrptx ${Boost_LIBRARIES} ${GRUEL_LIBRARIES} ${GNURADIO_CORE_LIBRARIES} ${LIBUSB_LIBRARIES})
set_target_properties(gnuradio-ssrptx PROPERTIES DEFINE_SYMBOL "gnuradio_ssrptx_EXPORTS")

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-ssrptx
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_ssrptx_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ssrptx.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_ssrptx.cc
)

add_executable(test-ssrptx ${test_ssrptx_sources})

target_link_libraries(
  test-ssrptx
  ${GNURADIO_CORE_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  gnuradio-ssrptx
)

GR_ADD_TEST(test_ssrptx test-ssrptx)
