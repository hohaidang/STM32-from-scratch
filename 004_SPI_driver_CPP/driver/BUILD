cc_library(
    name = "gpio_driver",
    srcs = ["stm32f446re_gpio_driver.cpp"],
    hdrs = ["stm32f446re_gpio_driver.h"],
    defines = ["GOOGLE_UNIT_TEST"], # compiler switch activate
    deps = ["core_cm4", "stm32f4xx", "//unit_test:unit_test_def"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "core_cm4",
    hdrs = ["core_cm4.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "stm32f4xx",
    hdrs = ["stm32f4xx.h"],
    visibility = ["//visibility:public"],
)