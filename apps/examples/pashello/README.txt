README
^^^^^^

hello.pas

  This is a sample "Hello, World!" Pascal Program

hello.pex

  This is the compiled, linked P-Code executable that results
  when hello.pas is compiled.

hello.h

  This file defines an initialized C array holds a copy of
  hello.pex.  This file as created by:

    xxd -i hello.pex >hello.h

mkhello.sh

  This is a scripts that can be used to rebuild both hello.pex
  and hello.h.

device.c

  The hello.pex file must be provided to the interpreter as a file
  in the file system.  Normally this would be done using real storage
  medium.  In this example, we will use device.c:

  device.c implements a simple device driver.  Reads from this device
  will access the in-memory copy of hello.pex  This device driver is
  registered as /dev/pashello in the pseudo filesystem.

