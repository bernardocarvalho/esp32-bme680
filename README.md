# esp32-bme680

How to build PlatformIO based project
=====================================

1. `Install PlatformIO Core <http://docs.platformio.org/page/core.html>`_
2. Run these commands:

.. code-block:: bash

    # Clone this repository
    > clone https://github.com/bernardocarvalho/esp32-bme680 

    # Change directory to example
    > cd devkit

    # Build project
    > platformio run

    # Upload firmware
    > platformio run --target upload

    # Clean build files
    > platformio run --target clean

