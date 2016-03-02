add_library(inverse-dynamics MODULE plugin.cpp)
target_link_libraries(inverse-dynamics ${REQLIBS})


# Moby Interface
add_library(TestPlugin MODULE test.cpp)
target_link_libraries (TestPlugin ${REQLIBS})
