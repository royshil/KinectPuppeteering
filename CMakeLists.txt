cmake_minimum_required (VERSION 2.6) 
project (KinectPuppeteering) 

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")

include(cotire)

find_package(OpenGL REQUIRED)
find_package(Qt4 REQUIRED QtGui QtOpenGL QtXml)
find_package(OpenMP REQUIRED)
find_package(boost REQUIRED)

find_package(Eigen3 QUIET)
IF(NOT Eigen3_FOUND)
	include(FindPkgConfig)
	IF(PKG_CONFIG_FOUND)
		pkg_check_modules(EIGEN3 REQUIRED eigen3 )
		include_directories(${EIGEN3_INCLUDE_DIRS})
	ENDIF()
ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_subdirectory(Pinocchio)
add_subdirectory(PinocchioUI)

file(GLOB MYSOURCES	
	*.cpp *.c)
file(GLOB MYHEADERS	
	*.h *.hpp)
file(GLOB MYUIS	
	*.ui)

######## Qt stuff ########
ADD_DEFINITIONS(${QT_DEFINITIONS})
include_directories(${QT_INCLUDES})
include(${QT_USE_FILE}) 

QT4_WRAP_UI( MY_UI_HDRS ${MYUIS} )
QT4_WRAP_CPP( MY_MOC_SRCS "KinectPuppeteeringViewer.h" )

INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR} ) #qt wrapping creates some sources
#########################

include_directories(Pinocchio PinocchioUI)

IF(APPLE)
   include_directories( /usr/local/include/ )
   INCLUDE_DIRECTORIES ( /System/Library/Frameworks )
   FIND_LIBRARY(COCOA_LIBRARY Cocoa)
   FIND_LIBRARY(GLUT_LIBRARY GLUT )
   FIND_LIBRARY(OpenGL_LIBRARY OpenGL )
   MARK_AS_ADVANCED (COCOA_LIBRARY
                     GLUT_LIBRARY
                     OpenGL_LIBRARY)
   SET(OpenGL_LIBS ${COCOA_LIBRARY} ${GLUT_LIBRARY} ${OpenGL_LIBRARY})
ENDIF (APPLE)

add_executable(KinectPuppeteering ${MYSOURCES} ${MYHEADERS} ${MY_UI_HDRS} ${MY_MOC_SRCS})

target_link_libraries(KinectPuppeteering 
	Pinocchio 
	PinocchioUI
	${QT_LIBRARIES} 
	${OpenGL_LIBS}
	QGLViewer
	)

set_target_properties(KinectPuppeteering PROPERTIES COTIRE_CXX_PREFIX_HEADER_INIT "std.h")
	
cotire(KinectPuppeteering)