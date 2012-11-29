
file(GLOB PinocUI_SOURCES	
	*.cpp *.c)
file(GLOB PinocUI_HEADERS	
	*.h *.hpp)

message(STATUS "PinocUI_SOURCES ${PinocUI_SOURCES}")

add_library(PinocchioUI ${PinocUI_SOURCES} ${PinocUI_HEADERS})

