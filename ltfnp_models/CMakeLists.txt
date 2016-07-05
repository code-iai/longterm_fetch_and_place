cmake_minimum_required(VERSION 2.8.3)
project(ltfnp_models)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package()

# Xacro files
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/models/*/*.urdf.xml)

foreach(it ${xacro_files})
  # Remove .xml extension
  string(REGEX MATCH "(.*)[.]xml$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})
  # Create a rule to generate ${output_filename} from ${it}
  xacro_add_xacro_file(${it} ${output_filename})
  
  list(APPEND model_files ${output_filename})
endforeach(it)

# Add an abstract target to actually trigger model builds
add_custom_target(shopping_model_media_files ALL DEPENDS ${model_files})
