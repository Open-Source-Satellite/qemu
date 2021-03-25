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

QEMU as a whole is released under the GNU General Public License,
version 2. For full licensing details, consult the LICENSE file.
   
Building
========

QEMU is multi-platform software intended to be buildable on all modern
Linux platforms, OS-X, Win32 (via the Mingw64 toolchain) and a variety
of other UNIX targets. The simple steps to build QEMU are:

We support building and running qemu from Windows 10 and Ubuntu Linux 
although Linux is def the least pain!

Building On Linux (Debian, Ubuntu, Mint)
========================================

Install the following dependencies (these are also required to run QEMU):

.. code-block:: shell

sudo apt-get install git libglib2.0-dev libfdt-dev libpixman-1-dev zlib1g-dev

Then, QEMU also advises the following additional dependencies:

.. code-block:: shell

sudo apt-get install git-email
sudo apt-get install libaio-dev libbluetooth-dev libbrlapi-dev libbz2-dev
sudo apt-get install libcap-dev libcap-ng-dev libcurl4-gnutls-dev libgtk-3-dev
sudo apt-get install libibverbs-dev libjpeg8-dev libncurses5-dev libnuma-dev
sudo apt-get install librbd-dev librdmacm-dev
sudo apt-get install libsasl2-dev libsdl1.2-dev libseccomp-dev libsnappy-dev libssh2-1-dev
sudo apt-get install libvde-dev libvdeplug-dev libvte-2.90-dev libxen-dev liblzo2-dev
sudo apt-get install valgrind xfslibs-dev

And, for newer versions of Debian/Ubuntu

.. code-block:: shell

sudo apt-get install libnfs-dev libiscsi-dev

Then, build the code by navigating qemu/ directory where you cloned the code from Git and then run the following commands: 

.. code-block:: shell

  mkdir build
  cd build
  ../configure --target-list=arm-softmmu
  make

Building On Windows 10
======================

In order to build QEMU for Windows, you'll need to install a few Linux emulation environments (MINGW64 is also needed to run QEMU on Windows)
Follow these steps:

MSYS2 provides a convenient environment to produce native builds for W64.
* Download and run the MSYS2 installer from msys2.org.
* Run the MSYS2 console (Click start and search for MSYS)
* As per the MSYS2 documentation, download the latest repository updates with:

.. code-block:: shell

pacman -Syu

* If required, restart the MSYS2 console. Then update the remaining packages with:

.. code-block:: shell

pacman -Su

* Next install the basic set of developer tools:

.. code-block:: shell

pacman -S base-devel mingw-w64-x86_64-toolchain git python ninja

* Then install any required QEMU-specific packages. For a basic setup you can use:

.. code-block:: shell

pacman -S mingw-w64-x86_64-glib2 mingw64/mingw-w64-x86_64-gtk3 mingw64/mingw-w64-x86_64-SDL2 python-setuptools

* Close the MSYS2 console.
* Start mingw64.exe. (It should be in the MSYS install directory)

.. code-block:: shell

cd /mingw64/bin
cp x86_64-w64-mingw32-gcc-ar.exe x86_64-w64-mingw32-ar.exe
cp x86_64-w64-mingw32-gcvc-ranlib.exe x86_64-w64-mingw32-ranlib.exe
cp windres.exe x86_64-w64-mingw32-windres.exe
cp nm.exe x86_64-w64-mingw32-nm.exe
cp objcopy.exe x86_64-w64-mingw32-objcopy.exe
cd ~


- Finally build QEMU with:

.. code-block:: shell

cd qemu
./configure --cross-prefix=x86_64-w64-mingw32- --enable-gtk --enable-sdl --target-list=arm-softmmu
make

Running QEMU for OSSAT
======================

In order to use the OSSAT fork of QEMU, you'll need to:

* Ensure that the executable qemu-system-arm from the release is in the
  /usr/bin directory (if running over Linux)
* Under Windows, you need to run QEMU from the MINGW64 console andyou'll 
  find the qemu-system-arm executable under the qemu/build directory
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
