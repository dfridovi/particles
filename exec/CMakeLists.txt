# Loop over the executables and create the targets.
particles_subdir_list(executables ${CMAKE_SOURCE_DIR}/exec)
foreach(executable ${executables})
  message("Including binary \"${BoldMagenta}${executable}${ColorReset}\".")
  file(GLOB_RECURSE local_src ${CMAKE_SOURCE_DIR}/exec/${executable}/*.cpp)
  add_executable(${executable} ${local_src})
  target_link_libraries(${executable} particles ${particles_LIBRARIES})
  particles_set_runtime_directory(${executable} ${EXECUTABLE_OUTPUT_PATH})
endforeach()

# Define a variable storing the path to executable directory.
add_definitions(-DPARTICLES_EXEC_DIR="${CMAKE_SOURCE_DIR}/exec")
