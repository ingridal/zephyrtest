# nRF Connect SDK with MS8607 sensor

This is the app to obtain the pressure, temperature & humidity from an I2C MikroE MS8607 sensor connected to pins 015 (SDA) and pins 013 (SCL) via TWI. 

MS8607 sensor [PHT Click] (https://www.mikroe.com/pht-click) is a self-contained pressure, humidity, and temperature sensor that is fully calibrated during manufacture. The sensor can operate from 1.5V to 3.6V. The MS8607 is ideal for weather station applications embedded into compact devices and any applications in which pressure, humidity, and temperature monitoring is required.

Application structure is adapted following this example: https://github.com/nrfconnect/ncs-example-application/tree/main. 

## Hardware 
* MikroE PHT Click sensor MS8607 data sheet:https://eu.mouser.com/datasheet/2/418/5/NG_DS_MS8607-02BA01_B3-1134999.pdf .
* nRF52840 Product specification is available [HERE](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fkeyfeatures_html5.html)

## Getting started

Before getting started, make sure you have a proper nRF Connect SDK development environment.
Follow the official
[Getting started guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started.html).

### Initialization

The first step is to initialize the workspace folder where
the ``ms8607`` and all nRF Connect SDK modules will be cloned. Run the following
command:

```shell
# initialize your workspace for the ms8607 package
west init -l ms8607
# update nRF Connect SDK modules
west update
```

### Building and running

To build the application, run the following command:

```shell
west build -b $BOARD app
```

where `$BOARD` is the target board such as ``nrf52840dongle_nrf52840`` or ``nrf52840dk_nrf52840``

A sample debug configuration is also provided. To apply it, run the following
command:

```shell
west build -b $BOARD app -- -DOVERLAY_CONFIG=debug.conf
```

You can also use it together with the `rtt.conf` file if using Segger RTT (Real Time Transfer). Once
you have built the application, run the following command to flash it:

```shell
west flash
```
