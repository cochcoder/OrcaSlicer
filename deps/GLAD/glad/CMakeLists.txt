cmake_minimum_required(VERSION 3.0)
project(GLAD)

find_package(OpenGL REQUIRED)

add_library(GLAD src/gl.c)
target_include_directories(GLAD PUBLIC include/)
target_link_libraries(GLAD PUBLIC OpenGL::GL ${CMAKE_DL_LIBS})

if (NOT BUILD_SHARED_LIBS)
    target_compile_definitions(GLAD PUBLIC GLAD_STATIC)
endif ()

include(GNUInstallDirs)

install( 
    FILES
        ${PROJECT_SOURCE_DIR}/include/glad/gl.h
    DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}/glad
)

install( 
    FILES
        ${PROJECT_SOURCE_DIR}/include/KHR/khrplatform.h
    DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}/KHR
)

install(TARGETS GLAD
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} 
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
