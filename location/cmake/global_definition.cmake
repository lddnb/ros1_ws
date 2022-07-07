set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
configure_file (
  ${PROJECT_SOURCE_DIR}/include/global_definition/global_definition.h.in
  ${PROJECT_BINARY_DIR}/include/global_definition/global_definition.h)
