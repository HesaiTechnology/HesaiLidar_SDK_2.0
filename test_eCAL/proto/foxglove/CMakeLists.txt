get_filename_component(DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(TARGET_NAME "pb_${DIR_NAME}")

add_library(${TARGET_NAME} STATIC)

file(GLOB PROTO_FILES_CPP
    ${CMAKE_CURRENT_SOURCE_DIR}/*.proto
    ${BUILD_ROOT_DIR}/util/types.proto
)

file(GLOB PROTO_FILES_PY ${CMAKE_CURRENT_SOURCE_DIR}/*.proto)

target_include_directories(${TARGET_NAME}
    PRIVATE
         ${Protobuf_INCLUDE_DIR}
)

# Run protocol buffer compiler to generate C++ and Python files
PROTOBUF_TARGET_CPP(${TARGET_NAME} ${BUILD_ROOT_DIR} ${PROTO_FILES_CPP})
foreach(pbfile ${PROTO_FILES_PY})
  add_custom_command(
      TARGET ${TARGET_NAME} POST_BUILD
      COMMAND ${Protobuf_PROTOC_EXECUTABLE} --proto_path=${CMAKE_CURRENT_SOURCE_DIR} --python_out=$<TARGET_FILE_DIR:${TARGET_NAME}> ${pbfile} -I=${BUILD_ROOT_DIR}
      DEPENDS ${PROTO_FILES_PY}
  )
endforeach()

install(
  DIRECTORY $<TARGET_FILE_DIR:${TARGET_NAME}>/
  COMPONENT c_${PROJECT_NAME}
  DESTINATION scripts/${DIR_NAME}
  FILES_MATCHING
  PATTERN *.py
)

install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/protobuf/${DIR_NAME}
    COMPONENT c_${PROJECT_NAME}
    DESTINATION include
    FILES_MATCHING
    PATTERN *.h
)

install(
    TARGETS ${TARGET_NAME}
    COMPONENT c_${PROJECT_NAME}
    DESTINATION lib
)

install(
    FILES $<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.pdb
    COMPONENT c_${PROJECT_NAME}
    DESTINATION lib
    OPTIONAL
)
