load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "gpio_driver",
    srcs = ["stm32f446re_gpio_driver.cpp"],
    hdrs = glob(["*.h"]), # include all header files in current library
    
    visibility = ["//visibility:public"],
)