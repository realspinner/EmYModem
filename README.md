# EmYModem
Implementation of XMODEM/YMODEM protocol, designed for embedded devices with RTOS.

## What is this?
The goal of the project is to create a set of classes and interfaces for complete implementation of XMODEM and YMODEM protocols, suitable for memory-constrained embedded devices. It has zero hardware or software dependencies, so would be easily adopted into virtually any project.

## What is ready?
For now the sending part of the project is complete and proven to be usable. The receiving part is coming soon I hope.

## How to use it
### To send a file from embedded device
First of all, take a look at _src/interfaces._ The host (i.e. your application) have to implement _iSystem_ and _iSerialIO_ interfaces to provide timers, synchronization objects and access to serial port.

The _iSystem_ interface implementation is basically a small pool of mutexes and binary semaphores. EmYSend class needs one mutex and couple of semaphores to interact with incoming bytes. The timer and delay should be obviously a wrappers of corresponding RTOS function.

The _iSerialIO_ interface implementation must have separate incoming and outgoing threads, running independently from the main thread. Somewhere in the incoming thread the status of _EmYSend_ instance must be checked with it's _isInputNeeded()_ method. If it is true, the incoming thread must feed the read bytes one by one into _EmYSend's_ _readRemote()_ method.

Then you have to create an instance of _EmYSend_ class, and call it's _init()_ method, providing pointers to _iSystem_ and _iSerialIO_ implementations.
After initialization the sending of file is quite straightforward, see an example, taken from one of my projects:

```c++
/**
 * In this example the "filesys" object provides access to a previously opened file
 * and "console" object gives the way to report messages to user's terminal.
 */
 
// At this point we already have the filename and the file is opened for reading cuccessfully. 
 
console.sendReplyFmt("\r\nSending file %s size %d bytes.\r\n", filename, filesys.getFileSize());
console.sendReplyLn("Please start the YMODEM receiver.\r\n"); 
 
// Initiate the transmission 
FileSendStatus_t fsend = emYSend.start(filename, filesys.getFileSize());

size_t sent = 0;
// If receiver has confirmed the reception of file
if (fsend == FSS_ACTIVE) {
  // While the transmission is going on
  while (fsend == FSS_ACTIVE) {
    // Read bytes from file
    size_t bytes = filesys.readData(dataReadBuffer, EMYMODEM_MAX_XFER_BLOCK_SIZE, sent);
    
    // If no more bytes were read, stop.
    if (bytes == 0) {
      break;
    }
    
    // Send the read bytes to receiver
    fsend = emYSend.send(dataReadBuffer, bytes);
    sent += bytes;
  }
}

// Check the result of last send operation
if (fsend != FSS_ABORTED) {
  // If the transmission wasn't aborted, try to gracefully finish it.
  fsend = emYSend.finish();
} else {
  // Perform abort of transmission
  fsend = emYSend.abort();
}

// Close the file
filesys.closeFile();

// Report the results to user
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
