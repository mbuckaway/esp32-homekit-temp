idf_component_register(SRCS ./bsec_integration.c ./bme68x.c ./app_main.c ./bsec_iaq.c)


target_link_libraries(${COMPONENT_TARGET} "-L${CMAKE_CURRENT_LIST_DIR}/lib")
target_link_libraries(${COMPONENT_TARGET} algobsec)