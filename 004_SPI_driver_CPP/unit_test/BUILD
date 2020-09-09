#cc_binary(
cc_test(
    name = "ut_gpio",
    srcs = ["ut_gpio.cpp"],
    defines = ["GOOGLE_UNIT_TEST"], # compiler switch activate
    deps = [
        ":gpio_driver",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "gpio_driver",
    srcs = ["stm32f446re_gpio_driver.cpp", "unit_test_def.cpp"],
    hdrs = ["core_cm4.h", "stm32f4xx.h", "stm32f446re_gpio_driver.h", "unit_test_def.h"],
    defines = ["GOOGLE_UNIT_TEST"], # compiler switch activate
    visibility = ["//visibility:public"],
)
