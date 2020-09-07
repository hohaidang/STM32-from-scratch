cc_binary(
    name = "ut_gpio",
    srcs = ["ut_gpio.cpp"],
    deps = [
        "//driver:gpio_driver",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "unit_test_def",
    srcs = ["unit_test_def.cpp"],
    hdrs = ["unit_test_def.h"],
    defines = ["GOOGLE_UNIT_TEST"],
    deps = ["//driver:core_cm4", "//driver:stm32f4xx"],
    visibility = ["//visibility:public"],
)