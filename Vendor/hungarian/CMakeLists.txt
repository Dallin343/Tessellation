cmake_minimum_required(VERSION 3.3.0)
project(HungarianAlgorithm)

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD 11)

add_library(Hungarian STATIC Hungarian.h Hungarian.cpp)