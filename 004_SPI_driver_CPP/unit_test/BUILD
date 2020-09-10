cc_test(
    name = "ut_gpio",
    srcs = ["ut_gpio.cpp"],
    deps = [
        ":gpio_driver",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "ut_spi",
    srcs = ["ut_spi.cpp"],
    deps = [
        ":spi_driver",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "spi_driver",
    srcs = ["stm32f446re_spi_driver.cpp"],
    hdrs = ["stm32f446re_spi_driver.h"],
    deps = [
        ":gpio_driver"
    ]
)

cc_library(
    name = "gpio_driver",
    srcs = ["stm32f446re_gpio_driver.cpp"],
    hdrs = ["stm32f446re_gpio_driver.h"],
    deps = [
        ":ut_def"
    ]
)

cc_library(
    name = "ut_def",
    srcs = ["unit_test_def.cpp"],
    hdrs = ["unit_test_def.h", "core_cm4.h", "stm32f4xx.h"],
    defines = ["GOOGLE_UNIT_TEST"], # compiler switch activate
    visibility = ["//visibility:public"],
)