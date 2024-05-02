project(lx16lib)

aux_source_directory(. LX16_SRC_LIST)

add_library(lx16lib STATIC ${LX16_SRC_LIST})

