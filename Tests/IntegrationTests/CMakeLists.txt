#===================== Integration tests configuration ========================#
# Integration tests of Iwdg module on target (Nucleo board). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).

IntegrationTesting_Add_Test(
    NAME            Iwdg
    TEST_SOURCE     ItTest_Iwdg.c
    LINK_LIBS       Iwdg_Lib
                    Rcc_Lib             # reset source of the MCU
                    Tim_Lib             # reference time base (TIM2)
    TIMEOUT         60
)
