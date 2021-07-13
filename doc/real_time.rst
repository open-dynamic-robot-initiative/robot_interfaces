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

To install the patched kernel on Ubuntu, you may use our script
install_rt_preempt_kernel.sh_.  It is tested for Ubuntu 18.04 but may also work
with other recent versions.

Before running the script, you will likly need to modify it a bit, though:

- The ``VERSION_`` variables at the top refer to the kernel version that will be
  installed.  Usually you can leave the default values.  See reempt_rt_versions_
  for available versions in case you want to change it.
- ``DEFAULT_CONFIG`` needs to be modified to match the kernel version you are
  currently using (see ``uname -r``).

Then simply execute the script in a terminal.  Internally, sudo is used in some
steps, so the user executing it needs to have sudo-permission.

In the beginning (after downloading some things) you will be asked to manually
adjust some configuration settings.  Before entering the menu, the script prints
the following instructions::

    Please apply the following configurations in the next step:

    General setup
      Local version - append to kernel release: [Enter] Add '-preempt-rt'

    Processor type and features ---> [Enter]
      Preemption Model (Voluntary Kernel Preemption (Desktop)) [Enter]
        Fully Preemptible Kernel (RT) [Enter] #Select

However, I observed that depending on the kernel version the "Preemption Model"
setting may be found in the "General setup" menu, not in "Processor type and
features".


The script will automatically download, build and install the kernel.  This will
take a while.


Configuration to use the PREEMPT_RT Kernel
==========================================

Boot with the PREEMPT_RT Kernel
-------------------------------

Once the PREEMPT_RT kernel is installed, you need to reboot and select the
"preempt-rt" kernel in the GRUB menu (go to "Advanced options for Ubuntu", it
should be listed there).

When the system is running, you can check which kernel is running with
``uname -a``.  It should print something containing "PREEMPT_RT".


Select PREEMPT_RT Kernel by Default
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can configure GRUB to automatically select this kernel by setting
``GRUB_DEFAULT`` in ``/etc/default/grub``.  For this, first the identifier of
the kernel needs to be determined.  Open a terminal and run

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


Add Users to the realtime Group
-------------------------------

To be able to run real-time threads, the user needs to be added to the
"realtime" group:

.. code-block:: sh

   sudo usermod -aG realtime ${USER}


Set Permissions of /dev/cpu_dma_latency
---------------------------------------

The following command needs to be run after each reboot::

    sudo chmod 0666 /dev/cpu_dma_latency

Alternatively you can set up a udev rule to do this automatically.  For this,
simply create a file ``/etc/udev/rules.d/55-cpu_dma_latency.rules`` with the
following content:

.. code-block::

    KERNEL=="cpu_dma_latency", NAME="cpu_dma_latency", MODE="0666"


Building robot_interfaces for the Real-Time System
==================================================

To be able to run real-time threads, the robot_interfaces software needs to be
build with the PREEMPT_RT kernel running.  This means if you built the workspace
previously running a normal kernel, you need to do a clean rebuild.

Alternatively, you can build with the following option to explicitly request a
real-time build (e.g. useful when cross-compiling)::

    colcon build --cmake-args -DOS_VERSION=preempt-rt


.. note::

    If you see the following output during initialisation of the robot, this
    means you are running a non-real-time build.

    .. code-block:: text

        Warning this thread is not going to be real time.


.. _install_rt_preempt_kernel.sh: https://github.com/machines-in-motion/ubuntu_installation_scripts/blob/master/rt-preempt/ubuntu18.04/install_rt_preempt_kernel.sh
.. _preempt_rt_versions: https://wiki.linuxfoundation.org/realtime/preempt_rt_versions
