add_library(CatapultSystem SHARED CatapultSystem.cpp)
target_link_libraries(CatapultSystem PRIVATE ${GZ_SIM_TARGET} ${GZ_TRANSPORT_TARGET} ${GZ_PLUGIN_TARGET})

px4_add_unit_gtest(SRC CatapultTest.cpp LINKLIBS ${GZ_SIM_TARGET} ${GZ_TRANSPORT_TARGET})

if(BUILD_TESTING)
    add_dependencies(unit-Catapult CatapultSystem)
    target_compile_definitions(unit-Catapult PRIVATE CATAPULT_PLUGIN_PATH="$<TARGET_FILE:CatapultSystem>")
endif()
