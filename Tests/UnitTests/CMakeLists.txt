#======================== Unit tests configuration ============================#
# Unit tests of Iwdg module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(IWDG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${IWDG_DIR}/.." ABSOLUTE)

UnitTesting_Add_Test(
    NAME            Iwdg
    TEST_SOURCE     Test_Iwdg.c
    UUT_SOURCES     ${IWDG_DIR}/Iwdg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
    INCLUDE_DIRS    ${IWDG_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
    LINK_LIBS       Ral_Lib
)
