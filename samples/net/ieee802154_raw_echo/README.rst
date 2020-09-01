.. _ieee802154_raw_echo_sample:

IEEE802.15.4 raw radio_api echo
###############################

Overview
********

The sample consists of two devices - dev_tx, dev_rx, which exchange data save
in test_data file.

The source code for this sample application can be found at:
:zephyr_file:`samples/net/ieee802154_raw_echo`.

Requirements
************

- `BabbleSim`_

Building and Running
********************

The applications were made to test native_posix radio emulation that uses
BabbleSim and Ieee 802.15.4 radio_api implementation for this peripheral.

Build both of the the applications separately using following command:

.. zephyr-app-commands::
   :zephyr-app: samples/net/ieee802154_raw_echo
   :board: native_posix
   :conf: prj.conf
   :goals: build
   :compact:

Setting up the environment
==========================

To run the sample, BabbleSim installed on the machine is required.
It can be built by following the `manual`_. Both dev-tx and dev-rx
applications must be built separately in their directiories.

Simulation can be started by running both applications::

    $ .dev-tx/build/zephyr.elf -bsim -d=0 -s=echo -p=2G4 &
    $ .dev-rx/build/zephyr.elf -bsim -d=1 -s=echo -p=2G4 &

The phy must also be started::

    $ .{BSIM_OUT_PATH}/bin/bs_2G4_phy_v1 -s=echo -D=2 -sim_length=30e6 &

.. _BabbleSim:
  https://babblesim.github.io

.. _manual:
  https://babblesim.github.io/building.html
