load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "ut_gpio",
    srcs = ["ut_gpio.cpp"],
    deps = [
        "//driver:gpio_driver",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_hello",
    srcs = ["cpp_lib_test.cpp"],
    deps = [
        "//driver/lib:cpp_lib",
        "@googletest//:gtest_main",
    ],
)