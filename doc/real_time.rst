***************
Real Time Setup
***************

Since a constant update rate of the :cpp:class:`~robot_interfaces::RobotBackend`
is important for stable control of the robot, a real-time capable operating
system should be used.

For this we are using a standard Linux distribution (Ubuntu in our case) but install a
kernel with better real-time capabilities.  There are two options: the "low-latency"
kernel which is easy to install but doesn't give full real-time guarantees, and the
preempt_rt kernel, which has better real-time performance but is more tedious to
install.

Below are instructions for both of them and on how to configure the system to use them.


.. _lowlatency_kernel:

Installation: Low-latency vs PREEMPT_RT Kernel
==============================================

We recommend one of the following Linux kernel variants:

**low-latency:**

- Fewer latency guarantees as with PREEMPT_RT but often good enough.  Latency can
  increase if the system load is high, though.  It can be useful to assign separate CPU
  cores for the real-time critical processes.
- Can be used with Nvidia drivers.
- Easy to install.  On Ubuntu it's just::

    sudo apt install linux-lowlatency


**PREEMPT_RT patch:**

- Better real-time guarantees.
- Officially not supported by Nvidia drivers (but may still kind of work, see
  :ref:`nvidia_preempt_rt`).
- Difficult to install as it has to be built from source.  See :doc:`preempt_rt_kernel`.


Historically we used the PREEMPT_RT kernel (and still do on many of our robots) but
lately also used the low-latency kernel in some projects without problems (you have to
be a bit more careful to not overload the CPU, though).
Since the low-latency Kernel is much easier to install, you may want to try with that
one first and only move to the PREEMPT_RT version if you notice timing-related issues.

The further system configuration described in the following is the same in both cases.


.. _boot_rt_kernel:

Boot with the real-time Kernel
==============================

Once the real-time kernel is installed, you need to reboot and select the corresponding
kernel ("preempt-rt" or "low-latency") kernel in the GRUB menu (go to "Advanced options
for Ubuntu", it should be listed there).

When the system is running, you can check which kernel is running with
``uname -a``.  It should print something containing "PREEMPT_RT"/"lowlatency".


Select real-time Kernel by Default
----------------------------------

You can configure GRUB to automatically select a specific kernel when booting the
computer.  We provide a script to semi-automatically update the configuration (option
1), but you may also edit it manually (option 2).

Option 1: Semi-automatic
~~~~~~~~~~~~~~~~~~~~~~~~

We provide a script grub_select_default_kernel.py_ for this.  Download it and execute
with

.. code-block:: bash

   sudo python3 grub_select_default_kernel.py

It will automatically parse the GRUB configuration and list the available boot options.
Select the desired kernel and press enter.  It will then show a diff of the change it
intends to apply to ``/etc/default/grub``.  **Review the diff carefully and only confirm
if it looks good!**  It should only modify the line starting with ``GRUB_DEFAULT``, i.e.
like this:

.. code-block:: diff

    --- current
    +++ new
    @@ -3,7 +3,7 @@
     # For full documentation of the options in this file, see:
     #   info -f grub -n 'Simple configuration'

    -GRUB_DEFAULT=0
    +GRUB_DEFAULT="gnulinux-advanced-cc71b1fb-b530-4694-a839-aecf600f1f49>gnulinux-5.15.0-112-generic-advanced-cc71b1fb-b530-4694-a839-aecf600f1f49"
     GRUB_TIMEOUT_STYLE=hidden
     GRUB_TIMEOUT=0
     GRUB_DISTRIBUTOR=`lsb_release -i -s 2> /dev/null || echo Debian`

If all is good and you confirmed, you still need to run

.. code-block:: bash

    sudo update-grub

to apply the change.  Then reboot and verify that the correct kernel is used.


Option 2: Manually
~~~~~~~~~~~~~~~~~~

If you don't want to use the script mentioned above, you may also edit the GRUB
configuration manually.

For this, first the identifier of the kernel needs to be determined.  Open a terminal and run

.. code-block:: bash

   cat /boot/grub/grub.cfg | grep -w -e menuentry -e submenu

It should print something like this::

    menuentry 'Ubuntu' --class ubuntu --class gnu-linux --class gnu --class os $menuentry_id_option 'gnulinux-simple-1a26991b-b045-48dd-bb12-064a2725b80b' {
    submenu 'Advanced options for Ubuntu' $menuentry_id_option 'gnulinux-advanced-1a26991b-b045-48dd-bb12-064a2725b80b' {
        menuentry 'Ubuntu, with Linux 5.4.93-rt51-preempt-rt' --class ubuntu --class gnu-linux --class gnu --class os $menuentry_id_option 'gnulinux-5.4.93-rt51-preempt-rt-advanced-1a26991b-b045-48dd-bb12-064a2725b80b' {
        menuentry 'Ubuntu, with Linux 5.4.93-rt51-preempt-rt (recovery mode)' --class ubuntu --class gnu-linux --class gnu --class os $menuentry_id_option 'gnulinux-5.4.93-rt51-preempt-rt-recovery-1a26991b-b045-48dd-bb12-064a2725b80b' {
        menuentry 'Ubuntu, with Linux 5.4.0-65-generic' --class ubuntu --class gnu-linux --class gnu --class os $menuentry_id_option 'gnulinux-5.4.0-65-generic-advanced-1a26991b-b045-48dd-bb12-064a2725b80b' {
        menuentry 'Ubuntu, with Linux 5.4.0-65-generic (recovery mode)' --class ubuntu --class gnu-linux --class gnu --class os $menuentry_id_option 'gnulinux-5.4.0-65-generic-recovery-1a26991b-b045-48dd-bb12-064a2725b80b' {


For ``GRUB_DEFAULT`` the path full submenu/menuentry is needed, using the id of
each step (the last part in the line, "gnulinux-..."), separated by ">".  In
this specific case, the setting for starting the rt-kernel would be::

    GRUB_DEFAULT = "gnulinux-advanced-1a26991b-b045-48dd-bb12-064a2725b80b>gnulinux-5.4.93-rt51-preempt-rt-advanced-471e9718-013f-4cbb-91a7-d22635173b70"

After saving the changes in ``/etc/default/grub`` you need to run the following
command for the changes to become active::

    sudo update-grub

Then reboot and verify that the correct kernel is used.



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


.. _grub_select_default_kernel.py: https://github.com/machines-in-motion/ubuntu_installation_scripts/blob/master/rt-preempt/grub_select_default_kernel.py
