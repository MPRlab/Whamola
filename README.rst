
Whamola Anderson README
==========================================
Whamola Anderson is a robotic one-stringed instrument that uses dynamic string tensioning to achieve pitch. 
The DSP, communications, and controls are handled by the `mbed <https://os.mbed.com/>`_ OS running on an STM32 MCU.
PlatformIO is the chosen IDE for this project.

How to build PlatformIO-based mbed project
------------------------------------------

.. code-block:: bash

    # Change current working directory to project (if not already)

    # Build project
    > platformio run

    # Upload firmware
    > platformio run --target upload

    # Build specific environment
    > platformio run -e nucleo_f746zg

    # Upload firmware for the specific environment
    > platformio run -e nucleo_f746zg --target upload

    # Clean build files
    > platformio run --target clean
