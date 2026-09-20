set(HM20_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")

idf_component_register(
    SRCS
    "main.cpp"
    "EspUartLink.cpp"
    "${HM20_ROOT}/src/drone/DroneNode.cpp"
    "${HM20_ROOT}/src/sim/DronePhysics.cpp"
    "${HM20_ROOT}/src/mavlink/Codec.cpp"
    "${HM20_ROOT}/src/mavlink/Endpoint.cpp"
    INCLUDE_DIRS
    "${HM20_ROOT}/include"
    REQUIRES
    esp_driver_uart)

target_include_directories(${COMPONENT_LIB} SYSTEM
                           PUBLIC "${HM20_ROOT}/include/third_party/c_library_v2")

# Який конфіг дрона (1..10, як номер тесту) вшити в прошивку: idf.py build -DDRONE_CONFIG=N. Змінити тест = перезібрати й перепрошити
set(DRONE_CONFIG 1 CACHE STRING "Номер вшитого конфігу дрона (1..10)")
target_compile_definitions(${COMPONENT_LIB} PRIVATE DRONE_CONFIG=${DRONE_CONFIG})
