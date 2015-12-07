file(GLOB PLUGIN_SOURCES
"*.cpp"
)

# build each source file as its own plugin (simple plugins)
FOREACH(i ${PLUGIN_SOURCES})
  GET_FILENAME_COMPONENT(PLUGINNAME ${i} NAME_WE)
  add_library(${PLUGINNAME} MODULE ${i})
  target_link_libraries(${PLUGINNAME} ${REQLIBS})
ENDFOREACH(i)

file(GLOB sub-dir "*")
foreach(dir ${sub-dir})
  if(IS_DIRECTORY ${dir})
    if(EXISTS ${dir}/CMakeLists.txt)
      add_subdirectory( ${dir} )
    endif()
  endif()
endforeach()

option(USE_CURSES "Build Pacer with ncurses display support?" OFF)
if(USE_CURSES)
  add_definitions(-DUSE_CURSES)
endif(USE_CURSES)
