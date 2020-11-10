# EmYModem
Implementation of XMODEM/YMODEM protocol, designed for embedded devices with RTOS.

## What is this?
The goal of the project is to create a set of classes and interfaces for complete implementation of XMODEM and YMODEM protocols, suitable for memory-constrained embedded devices. It has zero hardware or software dependencies, so would be easily adopted into virtually any project.

## What is ready?
For now the sending part of the project is complete and proven to be usable. The receiving part is coming soon I hope.

## How to use it
### To send a file from embedded device
First of all, take a look at _src/interfaces._ The host (i.e. your application) have to implement _iSystem_ and _iSerialIO_ interfaces to provide timers, synchronization objects and access to serial port.

Then you have to create an instance of _EmYSend_ class, and call it's _init()_ method, providing pointers to _iSystem_ and _iSerialIO_ implementations.
After initialization the sending of file is quite straightforward:

```
/**
 * In this example assumed that "filesys" object provides access to a previously opened file
 * and "console" object gives the way to report messages to user's terminal.
 */
FileSendStatus_t fsend = emYSend.start(filename, filesys.getFileSize());

size_t sent = 0;
if (fsend == FSS_ACTIVE) {
  while (fsend == FSS_ACTIVE) {
    size_t bytes = filesys.readData(dataReadBuffer, EMYMODEM_MAX_XFER_BLOCK_SIZE, sent);
    if (bytes == 0) {
      break;
    }
    fsend = emYSend.send(dataReadBuffer, bytes);
    sent += bytes;
  }
}

if (fsend != FSS_ABORTED) {
  fsend = emYSend.finish();
} else {
  fsend = emYSend.abort();
}

filesys.closeFile();

if (fsend == FSS_ABORTED) {
  console.sendReplyLn("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  console.sendReplyLn("ERROR: transmission aborted.\r\n");
} else if (fsend == FSS_SUCCESS) {
  console.sendReplyFmt("\r\nOK. Sent %d bytes successfully\r\n", sent);
} else if (fsend == FSS_ERROR) {
  console.sendReplyLn("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  console.sendReplyLn("ERROR. Transmission error\r\n");
} else {
  Console.sendReplyLn("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Console.sendReplyLn("ERROR. Transmission failed\r\n");
}
```
