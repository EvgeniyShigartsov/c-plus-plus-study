set(HM20_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")

idf_component_register(
    SRCS
    "main.cpp"
    "${HM20_ROOT}/src/drone/DroneNode.cpp"
    "${HM20_ROOT}/src/sim/DronePhysics.cpp"
    "${HM20_ROOT}/src/mavlink/Codec.cpp"
    INCLUDE_DIRS
    "${HM20_ROOT}/include")

target_include_directories(${COMPONENT_LIB} SYSTEM
                           PUBLIC "${HM20_ROOT}/include/third_party/c_library_v2")
