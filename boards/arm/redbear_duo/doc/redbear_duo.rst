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

Flashing an application to REDBEAR Duo
--------------------------------------


The sample application :ref:`hello_world` is being used in this tutorial:

.. code-block:: console

   $<zephyr_root_path>/samples/hello_world

To build the Zephyr kernel and application, enter:

.. code-block:: console

   $ cd <zephyr_root_path>
   $ source zephyr-env.sh
   $ cd $ZEPHYR_BASE/samples/hello_world/
   $ make BOARD=stm32f3_disco

Connect the RBlink to your host computer using the USB port. Then,
enter the following command:

.. code-block:: console

   $ make BOARD=stm32f3_disco flash

Run a serial host program to connect with your board at `115200 8N1`.

.. code-block:: console

   $ minicom -D /dev/ttyACM0

Or simplier:

.. code-block:: console

   $ screen /dev/ttyACM0 115200,cs8,-parenb,-cstopb,-hupcl

You should see the following message:

.. code-block:: console

   Hello World! arm


Debugging
=========

Access gdb with the following make commands work:

.. code-block:: console

   $ make BOARD=redbear_duo serverdebug
   $ make BOARD=redbear_duo debug
