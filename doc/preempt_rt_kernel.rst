***********************
PREEMPT_RT Linux Kernel
***********************

To get good real-time performance on Linux, we recommend using a Linux kernel
with the `PREEMPT_RT patch`_.  Unfortunately, this requires one to build the kernel
from source (at least on Ubuntu).  We provide some help for this in the
following.

Note that this page only covers installation of the kernel, for further
real-time-related configuration of the system see :doc:`real_time`.

Unfortunately, Nvidia does not support the PREEMPT_RT patch, so you may not be
able to use Nvidia drivers (and thus also no CUDA) when using this kernel.

An alternative may be the "lowlatency" kernel.  It is easier to install and
works with Nvidia drivers but has weaker real-time capabilities (see
:ref:`lowlatency_kernel`).


Install PREEMPT_RT Kernel
=========================

To install the patched kernel on Ubuntu, you may use our script
install_rt_preempt_kernel.sh_.  It is tested for Ubuntu 18.04 but may also work
with other recent versions.

Before running the script, you may want to modify it a bit, though:

- The ``VERSION_`` variables at the top refer to the kernel version that will be
  installed.  Usually you can leave the default values.  See reempt_rt_versions_
  for available versions in case you want to change it.

Then simply execute the script in a terminal.  Internally, sudo is used in some
steps, so the user executing it needs to have sudo-permission.

In the beginning (after downloading some things) you will be asked to manually
adjust some configuration settings.  Before entering the menu, the script prints
the following instructions::

    Please apply the following configurations in the next step:

    General setup [Enter]
      Local version - append to kernel release: [Enter] Add '-preempt-rt'

    General setup [Enter]
      Preemption Model (Voluntary Kernel Preemption (Desktop)) [Enter]
        Fully Preemptible Kernel (RT) [Enter] #Select

However, depending on the kernel version the "Preemption Model" setting may be
found in the "Processor type and features" menu instead.

The script will automatically download, build and install the kernel.  This will
take a while.


.. _boot_rt_kernel:

Boot with the PREEMPT_RT Kernel
===============================

Once the PREEMPT_RT kernel is installed, you need to reboot and select the
"preempt-rt" kernel in the GRUB menu (go to "Advanced options for Ubuntu", it
should be listed there).

When the system is running, you can check which kernel is running with
``uname -a``.  It should print something containing "PREEMPT_RT".


Select PREEMPT_RT Kernel by Default
-----------------------------------

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


.. _PREEMPT_RT patch: https://wiki.linuxfoundation.org/realtime/start
.. _install_rt_preempt_kernel.sh: https://github.com/machines-in-motion/ubuntu_installation_scripts/blob/master/rt-preempt/ubuntu18.04/install_rt_preempt_kernel.sh
.. _preempt_rt_versions: https://wiki.linuxfoundation.org/realtime/preempt_rt_versions
