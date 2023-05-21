.. Tilburg Hand documentation master file, created by
   sphinx-quickstart on Sun Jul 10 10:56:25 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Installation and basic usage
============================

.. image:: images/logo.png
   :width: 200
   :align: center

|

The tilburg-hand library contains the basic Python interface to communicate with the Tilburg Hand via USB, together with examples and a handy motor control GUI application (tilburg_hand_motorgui).


.. _installation:

Installation
------------

The library can be installed via PyPI

.. code-block:: console

   $ pip install tilburg-hand

or directly from source (`GitHub <https://github.com/TilburgRobotics/tilburg-hand>`_), running the following from inside the main directory:

.. code-block:: console

   $ pip install -e .

The most critical dependency of the library is the `dynamixel-sdk` Python wrapper, available from PyPi or from `Dynamixel <https://github.com/ROBOTIS-GIT/DynamixelSDK>`_ .
On Linux, the following enables access to USB devices without need of root privileges:

.. code-block:: console

    $ sudo usermod -a -G dialout $USER


.. _usage:

Usage
-----

Using the library is fairly simple. You need to instantiate a Tilburg HandMotorInterface() object, and connect() to the motors.

Before using the Tilburg Hand, you should generate a configuration file. A default configuration file (including the range of each joints and their zero position) is generated automatically using the included motor control GUI (see below).

The first time the GUI is opened, you will be asked to configure either a Left or Right Tilburg Hand. The default configuration file will be saved in your user folder. For example, on Linux it will be saved as  $HOME/tilburg_hand/calibration.json .
This is the directory that the Tilburg Hand library will look for the configuration file in, by default.

The motor GUI, like the Tilburg Hand library, rely on a second config.json configuration file (included by default within the installed Python library, in the subfolder `tilburg_hand/motorgui/config.json`). The config.json file includes default names for the USB port to use and/or the VID/PID of the U2D2 interface board (for automatic detection of the USB port). 

.. code-block:: python

    from tilburg_hand import TilburgHandMotorInterface, Unit

    motors = TilburgHandMotorInterface()
    ret = motors.connect()

    pos_normalized = [0.9, 0.7, 0.2, 0.5, 0.0, 0, 0, 0.9, 0.0, 0, 0, 0.1, 0.9, 0.85, 0.85, 0, 0, 0]
    motors.set_pos_vector(pos_normalized, unit=Unit.NORMALIZED)
    sleep(3)

    motors.goto_zero_position()
    sleep(1)

    motors.disconnect()



Example code is included with the Python package under the `examples` subfolder, and at the accompanying repository `tilburg-hand-contrib <https://github.com/TilburgRobotics/tilburg-hand-contrib>`_ .





.. _motor_gui:

Motor GUI
---------

A simple motor-control GUI is installed together with the tilburg_hand library, and can be run as:

.. code-block:: console

   $ tilburg_hand_motorgui

.. image:: images/motorgui.svg
   :width: 700
   :align: center


.. image:: images/motor_names.png
   :width: 400
   :align: center



.. _motors_settings:

Customizing Motors Settings
---------------------------

Motors settings like the individual PID gains can be set using the `Dynamixel Wizard 2 application <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2>`_ . Set the baudrate to 4000000 and the protocol version to 2.0.



.. usb_latency:

USB Latency
-----------

In order to reduce delays between the computer and the Tilburg Hand, you should enable low-latency settings for USB on your computer.
This is done automatically by the TilburgHandMotorInterface() object on **Linux**. For Windows, please follow the instructions at `usb_latency_setting <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting>`_ .



