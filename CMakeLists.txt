

project(adaptiveSubdivision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



  
include(cmake/OpenGL.cmake)
include(cmake/GLUT.cmake)
# include(cmake/CGAL.cmake)
find_package(CGAL REQUIRED)
include(cmake/Eigen3.cmake)




file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.h")

message(VTK_DIR="${VTK_DIR}")
message(CGAL_DIR="${CGAL_DIR}")
message(LIBRARIES="${LIBRARIES}")

add_executable(adaptiveSubdivision ${SOURCES} ${HEADERS})
target_link_libraries(adaptiveSubdivision PRIVATE ${LIBRARIES} CGAL::CGAL)




