cmake_minimum_required(VERSION 2.8.3)
project(rviz_satellite)

find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
if(NOT OGRE_OV_FOUND)
  pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif()

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rviz
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS nav_msgs sensor_msgs)

if (rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
	include(${QT_USE_FILE})
else()
	find_package(Qt5 COMPONENTS Core Gui Network Concurrent REQUIRED)
endif()

set(${PROJECT_NAME}_SOURCES
  src/aerialmap_display.cpp
  src/tileloader.cpp
)

set(${PROJECT_NAME}_HEADERS
  src/aerialmap_display.h
  src/tileloader.h
)

# invoke MOC and UI/ include Qt headers/ link Qt libraries
if (rviz_QT_VERSION VERSION_LESS "5")
	qt4_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})
	include_directories(${Qt4_INCLUDE_DIR})
	link_libraries(
  		${QT_LIBRARIES}
		)
else()
	qt5_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})
	include_directories(
		${Qt5Core_INCLUDE_DIRS}
		${Qt5Gui_INCLUDE_DIRS}
		${Qt5Network_INCLUDE_DIRS}
		${Qt5Concurrent_INCLUDE_DIRS}
		)
	link_libraries(
		${Qt5Core_LIBRARIES}
		${Qt5Gui_LIBRARIES}
		${Qt5Network_LIBRARIES}
		${Qt5Concurrent_LIBRARIES}
		)
endif()

# Other includes
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${OGRE_OV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  src
)

# Other libraries
link_libraries(
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

# set compiler options and inject version of software (see tileloader.cpp)
add_definitions("-Wall -Wunused -std=c++11 -DRVIZ_SATELLITE_VERSION=\"\\\"${rviz_satellite_VERSION}\\\"\"")

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOCSrcs}
)

add_library(${PROJECT_NAME}
  ${PROJECT_SOURCE_FILES}
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


