add_library(mrbear INTERFACE)
target_sources(mrbear INTERFACE ${CMAKE_CURRENT_LIST_DIR}/mrbear.c)
target_include_directories(mrbear INTERFACE ${CMAKE_CURRENT_LIST_DIR})
