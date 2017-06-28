.. _redbear_duo_board:

REDBEAR DUO
###########

Overview
********

TBC

Hardware
********

TBC

Programming and Debugging
*************************

Flashing
========

Redbear Duo card must be programmed with the Rblink kit. This
interface is supported by the openocd version included in Zephyr SDK.

Flashing an application to STM32F4DISCOVERY
-------------------------------------------

The sample application :ref:`blinky` is being used in this tutorial:

.. code-block:: console

   $<zephyr_root_path>/samples/basic/blinky

To build the Zephyr kernel and application, enter:

.. code-block:: console

   $ cd <zephyr_root_path>
   $ source zephyr-env.sh
   $ cd $ZEPHYR_BASE/samples/basic/blinky/
   $ make BOARD=redbear_duo

Connect the RBlink to your host computer using the USB port.
Then, enter the following command:

.. code-block:: console

   $ make BOARD=redbear_duo flash

You should see the red LED blinking.


Debugging
=========

Access gdb with the following make command **does not work**:

.. code-block:: console

   $ make BOARD=redbear_duo debug
