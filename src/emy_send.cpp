/*
 * Embedded YMODEM file sender realization, designed to use in multithreaded
 * RTOS-based embedded system.
 * Copyright (C) 2020  Oleg Kochetov ok@noiselab.ru
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/** 
 * Includes section
 */
#include "emy_send.h"
#include "ymodem_defs.h"
#include "crc16.h"
#include <cstring>

/**
 * Static variables section
 */                                

// Symbols for convertion of int to decimal notation
static const char decimals[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

/**
 * 
 * EMYSEND CLASS IMPLEMENTATION
 * 
 */

void EmYSend::init ( iSystem* system, iSerialIO* port ) {
    // Set interfaces
    this->system = system;
    this->port = port;

    // Reset statemashines
    mode = EMYMODE_YMODEM_G;
    sendState = EMYSEND_NONE;
    stage = EMYSTAGE_IDLE;
}


void EmYSend::reset() {
    // Return if is not initialized or already clean
    if ((stage == EMYSTAGE_UNKNOWN) ||
        (stage == EMYSTAGE_IDLE)) {
        return;
    }

    // Set statemashine to clearing stage
    stage = EMYSTAGE_CLEARING;

    // Drain the input queue
    while(inputSize > 0) {
        peekCode(0);
    }

    // Delete incoming binary semaphore
    if (incomingHandle > 0) {
        system->freeSemaphoreHandle(incomingHandle);
        incomingHandle = -1;
    }

    // Delete incoming free binary semaphore
    if (incomingFreeHandle > 0) {
        system->freeSemaphoreHandle(incomingFreeHandle);
    }

    // Delete incoming queue mutex
    if ( incomingMutexHandle > 0) {
        if (system->lockMutex(incomingMutexHandle, system->infiniteTime())) {
            system->unlockMutex(incomingMutexHandle);
            incomingMutexHandle = -1;
        }
    }

    // Reset statemachines
    mode = EMYMODE_YMODEM_G;
    sendState = EMYSEND_NONE;
    stage = EMYSTAGE_IDLE;
    
    // Set safe timeout value
    xferTimeout = EMYMODEM_SEND_GENERAL_TIMEOUT * 1000;
}


FileSendStatus_t EmYSend::start(const char* filename, size_t size) {
    // Check if it is clean to start
    if (stage != EMYSTAGE_IDLE) {
        return FSS_ERROR;
    }

    // Nothing to send, abort the transfer
    if (size == 0) {
        stage = EMYSTAGE_ABORT;
        return FSS_ABORTED;
    }

    // Request synchronization objects from system
    // Get mutex handle fot incoming bytes queue
    incomingMutexHandle = system->getMutexHandle();
    if ( incomingMutexHandle < 0) {
        stage = EMYSTAGE_INTERNAL_ERROR;
        return FSS_ERROR;
    }

    // Get binary semaphore to get signal on incoming byte
    incomingHandle = system->getSemaphoreHandle();
    if ( incomingHandle < 0) {
        stage = EMYSTAGE_INTERNAL_ERROR;
        return FSS_ERROR;
    }

    // Get binary semaphore to send signal when incoming
    // bytes queue has free space
    incomingFreeHandle = system->getSemaphoreHandle();
    if (incomingHandle < 0) {
        stage = EMYSTAGE_INTERNAL_ERROR;
        return FSS_ERROR;
    }

    // Initialize transfer counter
    bytesToSend = size;
    
    // Reset cancellation and abort counters
    cancelBytes = 0;
    retries = 0;
    
    // Set statemachine to initial wait stage
    stage = EMYSTAGE_WAIT_START;

    // Wait for the receiver to initiate transfer
    // Remeber start time for timeout calculation
    stageTime = system->getMillis();
    
    // Storage var for incoming code
    YmodemCode_t code = YM_NONE;
    
    // Cycling over incoming bytes
    do {
        
        // Wait for NAK/C/G byte from the receiver
        code = peekCode(1000);
        switch(code) {
            // Start YMODEM transfer
            case YmodemCode_t::YM_C :
                mode = EMYMODE_YMODEM;
                stage = EMYSTAGE_HEADER;
                break;

            // Start streaming YMODEM transfer
            case YmodemCode_t::YM_G :
                mode = EMYMODE_YMODEM_G;
                stage = EMYSTAGE_HEADER;
                break;

            // Start good old plain XMODEM transfer
            case YmodemCode_t::YM_NAK:
                mode = EMYMODE_XMODEM;
                stage = EMYSTAGE_BODY;
                frame.sequence = 1;
                frame.invSeq = ~frame.sequence;
                break;

            // receiver wants to cancel
            case YmodemCode_t::YM_CAN:
                // Increase cancellation counter
                ++cancelBytes;
                if (cancelBytes >= EMYMODEM_CANCEL_SEQUENCE) {
                    // Abort when sure to cancel
                    stage = EMYSTAGE_ABORT;
                }
                break;

            // just ignore all the rest incoming garbage
            default:
                break;
        }

        // Calc timeout
        xferTimeout = system->getMillis() - stageTime;

    // Repeat until timeout or statemachine change
    } while ((xferTimeout < EMYMODEM_SEND_GENERAL_TIMEOUT * 1000) &&
             (stage == EMYSTAGE_WAIT_START));

    // Check the stage
    if (stage == EMYSTAGE_WAIT_START) {
        // Transfer didn't start, wait finished by timeout
        stage = EMYSTAGE_ABORT;
    }

    // Return abort if aborted
    if (stage == EMYSTAGE_ABORT) {
        return FSS_ABORTED;
    }

    // Go on to transfer body of the file
    if (stage == EMYSTAGE_BODY) {
        // Yes we're in good old XMODEM mode
        return FSS_ACTIVE;
    }

    //**************************
    // YMODEM-specific section
    //**************************
    
    // Here is the transfer of zero block
    // Setup frame
    frame.sequence = 0;
    frame.invSeq = ~frame.sequence;
    
    // Clear frame data
    memset(frame.dataBlock, 0x00, EMYMODEM_MAX_XFER_BLOCK_SIZE);

    // Choose the size of block
    size_t flen = strlen(filename);
    bufferPosition = (flen > 100) ? 1024 : 128;

    // Set the frame header
    frame.blockType = static_cast<uint8_t>(
        (bufferPosition == 128) ? YmodemCode_t::YM_SOH : YmodemCode_t::YM_STX
    );

    // Copy the filename
    strcpy(reinterpret_cast<char*>(frame.dataBlock), filename);

    // Lowercase the filename
    uint8_t* ptr = frame.dataBlock;
    while (*ptr != 0x00) {
        char in = *ptr;
        if (in <= 'Z' && in >= 'A') {
            in -= ('Z' - 'z');
            *ptr = in;
        }
        ++ptr;
    }

    // Place file size field into data block
    // It is really not very wise to send gigabytes via x/ymodem
    if (size > 1000000000) {
        // So we refuse to do it here.
        // Actually we're limiting the length of
        // file size field by this...
        stage = EMYSTAGE_INTERNAL_ERROR;
        return FSS_ERROR;
    }

    // determine file size field length
    size_t nlen = 1;
    size_t fsize = size;
    
    // let's count how many decimal positions it has
    do {
        ++nlen;
        fsize /= 10;
    } while (fsize > 0);

    // render the file size field
    ptr = frame.dataBlock + flen + nlen - 1;
    fsize = size;

    // render decimals one by one
    do {
        *ptr = static_cast<uint8_t>(decimals[fsize % 10]);
        --ptr;
        fsize /= 10;
    } while (fsize > 0);

    // Transfer the zero block and return it's result
    return sendBlock(true);
}


YmodemCode_t EmYSend::peekCode(uint32_t timeout) {
    // Storage var for code
    YmodemCode_t code = YM_NONE;

    // get the timestamp
    uint32_t waitStart = system->getMillis();

    // Wait for valid incoming byte
    do {
        // Check if there are any incoming byte already
        if (inputSize == 0) {
            // Wait for incoming byte signal
            system->waitSemaphore(incomingHandle, timeout);
        }

        // Check again
        if (inputSize > 0) {
            // Lock the mutex
            if (system->lockMutex(incomingMutexHandle, system->infiniteTime())) {
                // Get the oldest code
                code = readQueue[0];
                
                // Index variable
                size_t idx = 0;
                
                // Decrease the queue length
                --inputSize;
                
                // Move rest of the incoming bytes to the begin of queue
                while (idx < inputSize) {
                    readQueue[idx] = readQueue[idx + 1];
                    ++idx;
                }
                
                // Unlock the mutex
                system->unlockMutex(incomingMutexHandle);
                
                // Send signal to receiving thread, there's at least one 
                // free cell in the queue.
                system->giveSemaphore(incomingFreeHandle);
            }
        }
    // Repeat until any valid code arrived or time's up.        
    } while ((code == YM_NONE) && (system->getMillis() - waitStart < timeout));

    // return the code
    return code;
}


void EmYSend::readRemote ( uint8_t byte ) {
    // Check if input is not acceptable
    if (!isInputNeeded()) {
        return;
    }

    
    // Check if there's no free cells in the input queue
    while (inputSize >= EMYMODEM_SEND_INPUT_QUEUE) {
        // Wait for some room to store input
        system->waitSemaphore(incomingFreeHandle, system->infiniteTime());
    }

    // Store the input. Lock the mutex first
    if (system->lockMutex(incomingMutexHandle, system->infiniteTime())) {
        // Validate the byte
        switch(byte) {
            case static_cast<uint8_t>(YM_SOH):
            case static_cast<uint8_t>(YM_STX):
            case static_cast<uint8_t>(YM_EOT):
            case static_cast<uint8_t>(YM_ACK):
            case static_cast<uint8_t>(YM_NAK):
            case static_cast<uint8_t>(YM_CAN):
            case static_cast<uint8_t>(YM_EOF):
            case static_cast<uint8_t>(YM_C):
            case static_cast<uint8_t>(YM_K):
            case static_cast<uint8_t>(YM_G):
            case static_cast<uint8_t>(YM_A1):
            case static_cast<uint8_t>(YM_A2):
                // The byte is from X-YMODEM set
                readQueue[inputSize++] = static_cast<YmodemCode_t>(byte);
                break;
            default:
                // Just substitute the garbage with NONE
                readQueue[inputSize++] = YmodemCode_t::YM_NONE;
                break;
        }
        
        // Unlock the mutex
        system->unlockMutex(incomingMutexHandle);
        
        // Send the signal there's an incoming byte
        system->giveSemaphore(incomingHandle);
    }
}


void EmYSend::setCRC() {
    // Pointer to the start of data block
    uint8_t* ptr = frame.dataBlock;
    // Pointer to the CRC16 position
    uint8_t* crcPtr;
    // Size of the data block
    size_t bytes;

    // Detect the size of data block and set pointer to CRC16
    if (static_cast<YmodemCode_t>(frame.blockType) == YM_SOH) {
        // Short block
        bytes = 128;
        crcPtr = ptr + bytes;
    } else {
        // 1K block
        bytes =  1024;
        crcPtr = frame.crc16;
    }

    // Get CRC16 for the data
    uint16_t crc = CalculateCRC16(0, ptr, bytes);

    // Store CRC16 into frame
    *(crcPtr) = crc >> 8;
    *(crcPtr + 1) = crc & 0x00FF;
}


void EmYSend::setChecksum() {
    // Checksum store var
    uint8_t checksum = 0;
    // Pointer to data block
    uint8_t* ptr = frame.dataBlock;
    // Checksum can be applied only for short block    
    size_t bytes = 128;

    // Calculate the sum
    while (bytes-- > 0) {
        checksum += *(ptr++);
    }

    // Store the sum
    *ptr = checksum;
}


void EmYSend::transmitFrame() {
    // Header is 3 bytes long
    size_t bytes = 3;

    // Detect data blcok size
    if (static_cast<YmodemCode_t>(frame.blockType) == YM_SOH) {
        bytes += 128;
        // Checksum or CRC16
        bytes += (mode == EMYMODE_XMODEM) ? 1 : 2;
    } else {
        // 1K packet + CRC16
        bytes += 1026;
    }
    
    // Transmit the data via iSerialIO interface
    port->sendData(reinterpret_cast<uint8_t const*>(&frame), bytes);
}


void EmYSend::getBuffer ( uint8_t ** bufPtr, size_t* size ) {
    // Write buffer pointer
    *bufPtr = frame.dataBlock;
    
    // Write buffer size
    *size = EMYMODEM_MAX_XFER_BLOCK_SIZE;
}


FileSendStatus_t EmYSend::send ( uint8_t* buffer, size_t size ) {
    // Check the cancellation signal
    if (inputSize != 0) {
        YmodemCode_t code = YM_NONE;
        cancelBytes = 0;
        // Iterate over input bytes
        do {
            // Try to get incoming code without delay 
            code = peekCode(0);
            
            // Check it against CAN 
            switch (code) {
            case YM_CAN:
                ++cancelBytes;
                break;

            default:
                cancelBytes = 0;
                break;
            }

        // Repeat until there are no valid codes or 
        } while ((code != YM_NONE) && (cancelBytes < EMYMODEM_CANCEL_SEQUENCE));

        // Check if there was a complete cancel sequence
        if (cancelBytes >= EMYMODEM_CANCEL_SEQUENCE) {
            // Abort transfer
            stage = EMYSTAGE_ABORT;
            return FSS_ABORTED;
        }
    }

    //Check if data is already in buffer
    //(user is working directly on frame.dataBlock)
    if ((buffer == frame.dataBlock) || (buffer == nullptr)) {
        // it is up to user to optimize the block size,
        bufferPosition = size;
        // just transmit what we've got
        return sendBlock(false);
    }

    // User provided his own buffer
    FileSendStatus_t status = FSS_ACTIVE;
    // Index variable
    size_t idx = 0;

    // Iterate over provided buffer data
    do {
        // Get free bytes in data block
        size_t free = EMYMODEM_MAX_XFER_BLOCK_SIZE - bufferPosition;
        
        // Detect how many bytes must be copied from user buffer to data block
        size_t toCopy = (size > free) ? free : size;

        //Block size optimization
        if (toCopy > 128) {
            toCopy = (toCopy < (1024 - 128)) ? 128 : toCopy;
        }

        // Check if mode is plain good old XMODEM
        if (mode == EMYMODE_XMODEM) {
            toCopy = 128;
        }

        // Copy data to the frame
        memcpy(frame.dataBlock, buffer + idx, toCopy);

        // Advance byte counters
        idx += toCopy;
        size -= toCopy;
        bufferPosition += toCopy;

        // transmit the block
        status = sendBlock(false);
    // Repeat until we have bytes in buffer or abort signal
    } while ((size > 0) && (status == FSS_ACTIVE));

    // Return last received status
    return status;
}


FileSendStatus_t EmYSend::sendBlock(bool header) {
    //Check if the buffer is not complete
    if ((bufferPosition != 128) && (bufferPosition != 1024)) {
        // If the block is not the last one - return signal to continue
        if ((bytesToSend - bufferPosition) != 0) {
            // Ask user to provide a new portion of data
            return FSS_ACTIVE;
        }
    }

    // Here the block MUST be transmitted, so it is safe to decrease the main file size counter
    if (!header) {
        bytesToSend -= bufferPosition;
    }

    // Block size and padding detection
    size_t pad = 0;

    // If data con be fitted into small block
    if (bufferPosition <= 128) {
        // Set SOH type of the block
        frame.blockType = static_cast<uint8_t>(YmodemCode_t::YM_SOH);
        // Calculate padding
        pad = 128 - bufferPosition;
    } else {
        // Set STX type of the block
        frame.blockType = static_cast<uint8_t>(YmodemCode_t::YM_STX);
        // Calculate padding
        pad = 1024 - bufferPosition;
    }

    // Do padding
    if (pad > 0) {
        memset(frame.dataBlock + bufferPosition, static_cast<uint8_t>(YmodemCode_t::YM_EOF), pad);
        bufferPosition += pad;
    }

    // Set the CRC or checksum
    if (mode == EMYMODE_XMODEM) {
        // Set checksum
        setChecksum();
    } else {
        // Set CRC
        setCRC();
    }

    // Reset abort counters
    retries = 0;

    // Do the transfer
    // Incoming code storage variable
    YmodemCode_t code = YmodemCode_t::YM_NONE;
    
    // Try to send the block 
    do {
        // Do transmit block to receiver
        transmitFrame();
        cancelBytes = 0;

        // Skip waiting for ACK if we're in streaming mode
        if (mode == EMYMODE_YMODEM_G) {
            break;
        }

        // Wait for the answer
        // Set the statemachine
        sendState = EMYSEND_WAIT;        
        // Get timer for timeout detection
        stageTime = system->getMillis();

        // iterate over incoming bytes
        do {
            // Try to get the code
            code = peekCode(1000);
            switch(code) {
                // Block accepted
                case YmodemCode_t::YM_ACK :                    
                    sendState = EMYSEND_ACK;
                    break;
                    
                // Receiver asked to repeat the transmission
                case YmodemCode_t::YM_NAK :
                    sendState = EMYSEND_NAK;
                    break;

                // Reciever wants to cancel transmission
                case YmodemCode_t::YM_CAN :
                    ++cancelBytes;
                    if (cancelBytes >= EMYMODEM_CANCEL_SEQUENCE) {
                        sendState = EMYSEND_ABORT;
                    }
                    break;

                // Meh, skip that garbage
                default:
                    cancelBytes = 0;
                    break;
            }

            // Calculate timeout
            xferTimeout = system->getMillis() - stageTime;

        // Repeat until timeout or statemachine change
        } while ((xferTimeout < EMYMODEM_SEND_ACK_TIMEOUT * 1000) &&
                (sendState == EMYSEND_WAIT));

        // Check sending statemachine
        switch (sendState) {
            case EMYSEND_WAIT:
                // Timeout is here
                sendState = EMYSEND_TIMEOUT;
                /* no break */

            case EMYSEND_TIMEOUT:
            case EMYSEND_NAK:
                // Let's try another one
                ++retries;
                if (retries >= 10) {
                    // No more retries, abort the transmission
                    stage = EMYSTAGE_ABORT;
                }
                break;

            case EMYSEND_ACK:                
                // The block has been accepted 
                if (stage == EMYSTAGE_HEADER) {
                    // switch main statemachine from HEADER to BODY stage
                    stage = EMYSTAGE_BODY;
                }
                break;

            case EMYSEND_ABORT:
                // Switch main statemachine to ABORT stage
                stage = EMYSTAGE_ABORT;
                break;

            default:
                // Just skip the garbage
                break;
        }

    // Repeat until retry is requested
    } while ((sendState == EMYSEND_NAK) || (sendState == EMYSEND_TIMEOUT));

    // If it is OK to continue, return FSS_ACTIVE or FSS_FINAL signal
    if ((sendState == EMYSEND_ACK) && ((stage == EMYSTAGE_BODY) || (stage == EMYSTAGE_FINAL))) {
        // Increment the frame sequence number
        ++frame.sequence;
        // Invert the sequence
        frame.invSeq = ~frame.sequence;
        // Reset data block write position
        bufferPosition = 0;
        
        // Check if there are more bytes to send
        if (bytesToSend > 0)  {
            return FSS_ACTIVE;
        } else {
            // It's time to finish the transfer
            // Transition from BODY to STOP_BATCH stage
            if (stage == EMYSTAGE_BODY) {
                stage = EMYSTAGE_STOP_BATCH;
            }
            return FSS_FINAL;
        }
    }

    // In all other cases return abort signal
    return FSS_ABORTED;
}


FileSendStatus_t EmYSend::finish() {
    // Reset retry counter
    retries = 10;
    // Incoming code storage variable
    YmodemCode_t code = YM_NONE;

    // Finalizing the transfer
    // When ABORT is asked, do it
    if (stage == EMYSTAGE_ABORT) {
        return abort();
    }

    // Forcefully set the STOP_BATCH stage
    stage = EMYSTAGE_STOP_BATCH;

    // Sending EOTs
    // Set code to frame space
    frame.dataBlock[0] = static_cast<uint8_t>(YmodemCode_t::YM_EOT);
    do {
        // Send the EOT code
        port->sendData(frame.dataBlock, 1);
        // Switch sending statemachine to WAIT
        sendState = EMYSEND_WAIT;
        // Get timer for timeout calculation
        stageTime = system->getMillis();

        // Iterate over incoming bytes
        do {
            // Wait for incoming byte
            code = peekCode(1000);

            // Check the code
            switch(code) {                
                case YM_ACK:
                    // receiver agreed to stop the transfer of the file
                    sendState = EMYSEND_ACK;
                    break;
                    
                case YM_NAK:
                    // receiver is not sure about what we've asked
                    sendState = EMYSEND_NAK;
                    break;
                    
                default:
                    // Skip all other replies
                    break;
            }

            // Calculate timeout
            xferTimeout = system->getMillis() - stageTime;
            
        // Repeat until timeout or sending statemachine change
        } while ((sendState == EMYSEND_WAIT) && (xferTimeout < EMYMODEM_SEND_GENERAL_TIMEOUT));

        // On timeout or NAK, repeat the EOT
        if ((sendState == EMYSEND_WAIT) ||
            (sendState == EMYSEND_NAK)) {
            --retries;
        }
        
    // Repeat until we have retries or receiver accepted EOT
    } while ((sendState != EMYSEND_ACK) && (retries > 0));

    // If receiver gracefully accepted the EOT and we're 
    // doing batch transmission
    if ((retries > 0) && (mode == EMYMODE_XMODEM)) {
        // Wait for next file request 'C'
        // Reset retries counter
        retries = 10;
        
        // Iterate over input bytes
        while (retries > 0) {
            // read the code
            code = peekCode(1000);

            // Receiver asked for the next file
            if (code == YmodemCode_t::YM_C) {
                // Go transmit the NULL zero-frame
                stage = EMYSTAGE_FINAL;
                break;
            }

            // Input timeout - retry
            if (inputSize == 0) {
                --retries;
            }
        }

        // Clear to finish the transmission with NULL zero-frame
        if (stage == EMYSTAGE_FINAL) {
            // Set frame type
            frame.blockType = YmodemCode_t::YM_SOH;
            // Zero sequence
            frame.sequence = 0;
            frame.invSeq = ~frame.sequence;
            // Zero out the data block
            memset(frame.dataBlock, 0x00, 128);
            bufferPosition = 128;
            // Send the block
            sendBlock(true);
        }
    }

    // Last EOT, just for luck
    frame.dataBlock[0] = static_cast<uint8_t>(YmodemCode_t::YM_EOT);
    port->sendData(frame.dataBlock, 1);
    system->sleepMillis(100);

    // Check the main statemachine
    FileSendStatus_t  ret = (stage == EMYSTAGE_FINAL) ? FSS_SUCCESS : FSS_ABORTED;

    // Clear everything
    reset();

    // Report result
    return ret;
}


FileSendStatus_t EmYSend::abort() {
    // Send a couple of EOTs and reset everything
    frame.dataBlock[0] = static_cast<uint8_t>(YmodemCode_t::YM_EOT);
    
    // First EOT
    port->sendData(frame.dataBlock, 1);
    // Wait a little
    system->sleepMillis(100);
    // Seconf EOT
    port->sendData(frame.dataBlock, 1);
    
    // Clear everything
    reset();
    
    // Confirm the transmission has been aborted
    return FSS_ABORTED;
}

