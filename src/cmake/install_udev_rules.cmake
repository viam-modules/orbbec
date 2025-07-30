add_custom_command(install_udev_rules
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_BINARY_DIR}/_deps/orbbec-sdk-release-src/shared/99-obsensor-libusb.rules"
            "/etc/udev/rules.d/"
    COMMAND udevadm control --reload-rules
    COMMAND udevadm trigger
)
