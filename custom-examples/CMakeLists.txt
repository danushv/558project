# Register the executable for basic-network
add_executable(basic-network basic-network.cc)

# Link the basic-network executable to NS-3 libraries with exact names
target_link_libraries(basic-network 
    ${CMAKE_SOURCE_DIR}/build/lib/libns3-dev-core-default.dylib
    ${CMAKE_SOURCE_DIR}/build/lib/libns3-dev-network-default.dylib
    ${CMAKE_SOURCE_DIR}/build/lib/libns3-dev-wifi-default.dylib
    ${CMAKE_SOURCE_DIR}/build/lib/libns3-dev-mobility-default.dylib
    ${CMAKE_SOURCE_DIR}/build/lib/libns3-dev-internet-default.dylib
    ${CMAKE_SOURCE_DIR}/build/lib/libns3-dev-energy-default.dylib
)
