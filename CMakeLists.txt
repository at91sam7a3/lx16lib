project(lx16)
cmake_minimum_required(VERSION 2.8)

aux_source_directory(. LX16_SRC_LIST)

add_library(lx16 ${LX16_SRC_LIST})
