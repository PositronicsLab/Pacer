find_package(SDL2 QUIET)
find_package(curses QUIET)

IF(CURSES_FOUND)
option(USE_CURSES "Build Pacer with ncurses display support?" OFF)
ENDIF(CURSES_FOUND)

IF(SDL2_FOUND)
IF(CURSES_FOUND AND USE_CURSES)
add_definitions( -DUSE_CURSES )
ENDIF()
add_library(gamepad MODULE gamepad.cpp)
target_link_libraries(gamepad ${REQLIBS} ${SDL2_LIBRARIES} ${CURSES_LIBRARIES})
ENDIF(SDL2_FOUND)
