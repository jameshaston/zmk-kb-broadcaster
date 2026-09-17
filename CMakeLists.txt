if (CONFIG_ZMK_KB_BROADCASTER)
  zephyr_library()

  zephyr_library_sources(src/broadcaster.c)

  zephyr_include_directories(include)
endif()