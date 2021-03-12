===============
OSSAT QEMU FORK
===============

Welcome to the OSSAT QEMU Fork that has been created specifically for
OSSAT `<https://opensourcesatellite.org>`_.

We are trying to develop a spacecraft platform including all design
artefacts (software, hardware, mechanical). We are pledging to
issue this design freely and openly on the internet 1 year from the
first spacecraft launch. If you want to get involved... Get on over 
to http://www.opensourcesatellite.org/register. 

As part of this development, we are engaging developers to create
on board software, in order to support this development, we figured
that we would need a emulator for our preferred main processor
(STM32H753ZI) and one that is open source... QEMU is a good fit!

Whilst QEMU emulates lots of ARM processors, support for Cortex-M7
is since release 5 (2019) and there are only a few boards supported.
Therefore, we decided to fork version 5.2.0 and then modify it to add
explicit support for the ST Nucleo-H753ZI board. This is a basic
evaluation board that includes the ST-Link debugging hardware. So,
you can take code built within the STM32Cube environment, add some
small modifications to get it running under QEMU on your PC,
with no actual STM hardware :)
This also includes support for GDB debugging :) :)

Getting hold of the code
========================

Since the code is branched from v5.2.0 of QEMU, you'll need to clone
as follows:

.. code-block:: shell

   git clone --branch OSSAT_STM32_MODS https://github.com/Open-Source-Satellite/qemu.git

This is the development branch, I have also been releasing code too,
the OSSAT releases are available from GitHub, the release tags are 
formatted as:

.. code-block:: shell

  OSSAT_STM32_VX.X.X

NOTE: I am attempting to add the Windows and Linux executables to each
release, so if you just want to use the executable without building it,
just download the executable from the latest release

NOTE: On Linux: QEMU is shipped with many Linux distributions. Therefore,
you need to replace the qemu-system-arm executable within the /usr/bin
directory in order to use this OSSAT version.

   
Building
========

QEMU is multi-platform software intended to be buildable on all modern
Linux platforms, OS-X, Win32 (via the Mingw64 toolchain) and a variety
of other UNIX targets. The simple steps to build QEMU are:


.. code-block:: shell

  mkdir build
  cd build
  ../configure
  make

Additional information can also be found online via the QEMU website:

* `<https://qemu.org/Hosts/Linux>`_
* `<https://qemu.org/Hosts/Mac>`_
* `<https://qemu.org/Hosts/W32>`_


QEMU is a generic and open source machine & userspace emulator and
virtualizer.

QEMU is capable of emulating a complete machine in software without any
need for hardware virtualization support. By using dynamic translation,
it achieves very good performance. QEMU can also integrate with the Xen
and KVM hypervisors to provide emulated hardware while allowing the
hypervisor to manage the CPU. With hypervisor support, QEMU can achieve
near native performance for CPUs. When QEMU emulates CPUs directly it is
capable of running operating systems made for one machine (e.g. an ARMv7
board) on a different machine (e.g. an x86_64 PC board).

QEMU is also capable of providing userspace API virtualization for Linux
and BSD kernel interfaces. This allows binaries compiled against one
architecture ABI (e.g. the Linux PPC64 ABI) to be run on a host using a
different architecture ABI (e.g. the Linux x86_64 ABI). This does not
involve any hardware emulation, simply CPU and syscall emulation.

QEMU aims to fit into a variety of use cases. It can be invoked directly
by users wishing to have full control over its behaviour and settings.
It also aims to facilitate integration into higher level management
layers, by providing a stable command line interface and monitor API.
It is commonly invoked indirectly via the libvirt library when using
open source applications such as oVirt, OpenStack and virt-manager.

QEMU as a whole is released under the GNU General Public License,
version 2. For full licensing details, consult the LICENSE file.

Using QEMU for OSSAT
====================

In order to use the OSSAT fork of QEMU, you'll need to:

* Ensure that the executable qemu-system-arm from the release is in the
  /usr/bin directory
* Run the following command line

.. code-block:: shell
  
  qemu-system-arm -kernel <name_of_elf_file>.elf -M stm32h753-nucleo -nographic -d guest_errors -D ./log_file.txt

explaining this command line:

* qemu-system-arm: This is the QEMU executable for ARM emulation
* -kernel: This specifies the elf file that will be loaded into the emulated
  processors memory before resetting and running the virtual target processor.
  (see the "Related Repos" section later on for a Repo containing STM32Cube
  code that can build for the real and virtual STM32 target).
* -M: This specifies the board that QEMU is emulating. Note: this is where
  our customisation is evident. We have customised it to emulate the
  STM32H753ZI Nucleo board.
* -nographic since this is an embedded target with no display, there are
  no graphics and all serial output (that is routed through USART3 on the
  real target) is routed to the terminal running QEMU.
* -d guest_errors sets what errors get put into the QEMU log file.
* -D this is the path to a log file that qemu generates as it performs its
  emulation.

* -s -S: these are optional, allowing for gdb debugging. They basically
  tell the emulator to halt on the first instruction and wait for a GDB
  connection.


Related Repos
=============

There is a Unit Test Template project that can be used to build code for the
STM32H753ZI processor and run the code on either a real (Nucleo) target OR
the QEMU target.

Contributing
============

See the GitHub Issues for a list of enhancements... If you want to contribute
Please review the enhancements and contact pmadle@kispe.co.uk

Bug reporting
=============

Please use the GitHub Issues to log any issues you find.

Contact
=======

To register to collaborate on OSSAT, go to https://opensourcesatellite.org/register
To contact/hurl abuse at the main author of this fork, please email pmadle@kispe.co.uk.
