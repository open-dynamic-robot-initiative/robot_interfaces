***************
Real Time Setup
***************

Since a constant update rate of the :cpp:class:`~robot_interfaces::RobotBackend`
is important for stable control of the robot, a real-time capable operating
system should be used.

We are using Ubuntu with a PREEMPT_RT kernel patch.  Below are instructions on
how to install the patched kernel and how to configure the system to use it.


Install PREEMPT_RT Kernel
=========================

See :doc:`preempt_rt_kernel`.


.. _lowlatency_kernel:

Alternative: The Low Latency Kernel
===================================

A potential alternative to the PREEMPT_RT patch is using the "lowlatency"
kernel.  It is easier to install and does not conflict with Nvidia drivers.  On
the other hand, the real-time capabilities it provides are weaker than those of
the PREEMPT_RT kernel.  Whether it is sufficient for you depends on your
application (e.g. how much computational load you put on the system while the
robot is running).  We still recommend using the PREEMPT_RT kernel but you may
test with the lowlatency kernel first.

To install on Ubuntu 20.04 (on other versions of Ubuntu adjust the version at
the end)::

    sudo apt install linux-lowlatency-hwe-20.04

Configuration to use this kernel by default works the same as for the PREEMPT_RT
kernel, see :ref:`boot_rt_kernel`.


System Configuration
====================

Normally root permission is needed to run real-time processes.  To allow
non-root users to run real-time processes, apply the configuration described in
this section.


Create the "realtime" Group and Add Users
-----------------------------------------

If you used our install script to install the PREEMPT_RT kernel, the "realtime"
group is already created automatically.  Otherwise, you can create it manually
with this command:

.. code-block:: sh

   sudo groupadd realtime

Then add users that should be able to run real-time applications to the group:

.. code-block:: sh

   sudo usermod -aG realtime <username>


Set rtprio and memlock limits
-----------------------------

**Note:** This step can be skipped if you used our install script to install the
PREEMPT_RT kernel.

Create a file ``/etc/security/limits.d/99-realtime.conf`` with the following
content to increase the rtprio and memlock limit for users in the realtime
group::

    @realtime   -   rtprio  99
    @realtime   -   memlock unlimited



Set Permissions of /dev/cpu_dma_latency
---------------------------------------

The following command needs to be run after each reboot to allow non-root users
to set the CPU DMA latency::

    sudo chmod 0666 /dev/cpu_dma_latency


Alternatively you can set up a udev rule to do this automatically.  For this,
simply create a file ``/etc/udev/rules.d/55-cpu_dma_latency.rules`` with the
following content:

.. code-block::

    KERNEL=="cpu_dma_latency", NAME="cpu_dma_latency", MODE="0666"


Building robot_interfaces for the Real-Time System
==================================================

To be able to run the robot backend in a real-time thread, some options need to
be set at build time.  This is automatically done when building on a system that
is running the PREEMPT_RT or lowlatency kernel.  However, if you previously
built while running a different kernel, you'll need to rebuild.

You can also explicitly build with real-time settings on any kernel with the
following command::

    colcon build --cmake-args -DOS_VERSION=preempt-rt


.. note::

    If you see the following output during initialisation of the robot, this
    means you are running a non-real-time build.

    .. code-block:: text

        Warning this thread is not going to be real time.

