
file(GLOB MYSOURCES	
	*.cpp *.c)
file(GLOB MYHEADERS	
	*.h *.hpp)

list(REMOVE_ITEM MYSOURCES main.cpp)


add_library(Pinocchio ${MYSOURCES} ${MYHEADERS})

