cmake_minimum_required(VERSION 3.7)
project(boxCollisions)

set(CMAKE_CXX_STANDARD 14)

set(SOURCE_FILES
        main.cpp
        src/box.cpp
        lib/src/billengine.cpp
        lib/src/billGLfunctions.cpp
        lib/src/billmaterialpoint.cpp
        lib/src/billwindow.cpp
        lib/src/billRBengine.cpp
        lib/src/billrigidbody.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

SET(EXTRA_LIBS GL X11 GLU glut)

target_link_libraries(${PROJECT_NAME} ${EXTRA_LIBS})