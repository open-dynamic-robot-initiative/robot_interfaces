:orphan:

*********************************************
Build and Install the PREEMPT_RT Linux Kernel
*********************************************

To get good real-time performance on Linux, we recommend using a Linux kernel
with the `PREEMPT_RT patch`_.  Unfortunately, this requires one to build the kernel
from source (at least on Ubuntu).  We provide some help for this in the
following.

Note that Nvidia drivers do officially not support the PREEMPT_RT patch.  It may still
work (see :ref:`nvidia_preempt_rt` below) but you are at your own risk.

An alternative may be the "lowlatency" kernel.  It is easier to install and
works with Nvidia drivers but has weaker real-time capabilities (see
:ref:`lowlatency_kernel`).


Install PREEMPT_RT Kernel
=========================

To install the patched kernel on Ubuntu, you may use our script
``install_rt_preempt_kernel.sh``.  We provide adapted versions for different Ubuntu
versions, which can be found `here <install_rt_preempt_kernel.sh_>`__.

The ``VERSION_`` variables at the top of the script refer to the kernel version that
will be installed.  Usually you can leave the default values, but you can change it here
if you want a different version.  See preempt_rt_versions_ for available versions.

Then simply execute the script in a terminal.  Internally, sudo is used in some
steps, so the user executing it needs to have sudo-permission.

.. note::

    In the beginning (after downloading some things) you will be asked to manually
    adjust some configuration settings.  Before entering the menu, the script prints
    instructions like the following::

        Please apply the following configurations in the next step:

        [...]

        General setup [Enter]
          Preemption Model (Voluntary Kernel Preemption (Desktop)) [Enter]
            Fully Preemptible Kernel (RT) [Enter] #Select

    However, depending on the kernel version the "Preemption Model" setting may be
    found in the "Processor type and features" menu instead.

The script will automatically download, build and install the kernel.  This will
take a while.

When finished, go back to :doc:`real_time` and follow the further steps to configure
your system for real-time usage.


.. _nvidia_preempt_rt:

Using Nvidia Drivers with the PREEMPT_RT Kernel
===============================================

Officially, Nvidia drivers do not support the PREEMPT_RT kernel.  However, at least with
more recent versions, it seems to work in practice.  However, you need set the
environment variable ``IGNORE_PREEMPT_RT_PRESENCE=1`` when installing it with apt.
Complete steps:

1. First uninstall any Nvidia drivers
2. Install the PREEMPT_RT kernel (see above)
3. Install the drivers with the following command (adjust driver version to the desired
   one)::

      sudo IGNORE_PREEMPT_RT_PRESENCE=1 apt install nvidia-driver-530

Please note that this variable also needs to be set when upgrading packages, so you may
want to set it in a global place like `/etc/environment` and disable unattended upgrades
for the driver.


.. _PREEMPT_RT patch: https://wiki.linuxfoundation.org/realtime/start
.. _install_rt_preempt_kernel.sh: https://github.com/machines-in-motion/ubuntu_installation_scripts/tree/master/rt-preempt
.. _preempt_rt_versions: https://wiki.linuxfoundation.org/realtime/preempt_rt_versions
