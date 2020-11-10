/*
 * Embedded YMODEM file transfer implementation, desined to use in multithreaded
 * RTOS-based system.
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

#ifndef EMYMODEM_H
#define EMYMODEM_H

/**
 * Includes section
 */
#include "interfaces/iFileSend.h"
#include "interfaces/iSerialIO.h"
#include "interfaces/iSystem.h"
#include "emymodem_config.h"
#include "ymodem_defs.h"


/**
 * Defines secton
 */
#ifdef EMYMODEM_USE_1K
#define EMYMODEM_MAX_XFER_BLOCK_SIZE    1024
#else
#define EMYMODEM_MAX_XFER_BLOCK_SIZE    128
#endif


/**
 * Types section
 */

/**
 * Stages of main statemachine
 */
typedef enum _EmYSendState_t {
    EMYSTAGE_UNKNOWN,
    EMYSTAGE_IDLE,
    EMYSTAGE_WAIT_START,
    EMYSTAGE_HEADER,
    EMYSTAGE_BODY,
    EMYSTAGE_STOP_BATCH,
    EMYSTAGE_FINAL,
    EMYSTAGE_ABORT,
    EMYSTAGE_INTERNAL_ERROR,
    EMYSTAGE_CLEARING,
} EmYSendStage_t;


/**
 * Supported modes of transfer (autodetected by reading receiver input)
 */
typedef enum _EmYSendMode_t {
    EMYMODE_XMODEM,
    EMYMODE_XMODEM_1K,
    EMYMODE_YMODEM,
    EMYMODE_YMODEM_G
} EmYSendMode_t;


/**
 * Block sender statemachine
 */
typedef enum _EmYSendBlockState_t {
    EMYSEND_NONE,
    EMYSEND_CLEAR,
    EMYSEND_WAIT,
    EMYSEND_ACK,
    EMYSEND_NAK,
    EMYSEND_ABORT,
    EMYSEND_TIMEOUT
} EmYSendBlockState_t;


/**
 * Data block frame structure
 */
typedef struct _EmYFrame {
    uint8_t blockType;
    uint8_t sequence;
    uint8_t invSeq;
    uint8_t dataBlock[EMYMODEM_MAX_XFER_BLOCK_SIZE];
    uint8_t crc16[2];
} __attribute__((__packed__)) EmYFrame;


/**
 * This is the implementation of iFileSend interface for X- and YMODEM protocols.
 * User must supply the instance with objects providing interfaces iSerialIO for 
 * serial write operations and iSystem for system resources like create/destroy/lock/release
 * synchronization objects (mutex and binary semaphores) and timers/sleep with millisecond
 * resolution.
 */
class EmYSend : public iFileSend {
private:
    // iSystem interface
    iSystem* system = nullptr;
    
    // iSerialIO interface
    iSerialIO* port = nullptr;

    // Work buffer
    EmYFrame frame = {};
    
    // Byte counters
    size_t bytesToSend = 0;
    size_t bufferPosition = 0;


    // Statemachines vars
    EmYSendMode_t mode = EMYMODE_YMODEM_G;
    EmYSendStage_t stage = EMYSTAGE_UNKNOWN;
    EmYSendBlockState_t sendState = EMYSEND_NONE;

    // Timeouts 
    uint32_t stageTime = 0;
    uint32_t xferTimeout = 0;
    
    // Handles of synchronization objects provided by iSystem
    int incomingHandle = -1;
    int incomingFreeHandle = -1;
    int incomingMutexHandle = -1;

    // Cancellation and abort counters
    size_t cancelBytes = 0;
    uint8_t retries = 0;

    
    // Input processing
    YmodemCode_t readQueue[EMYMODEM_SEND_INPUT_QUEUE] = {};
    size_t inputSize = 0;
    
    /**
     * Wait for incoming byte with the given timeout or return immediately if
     * a byte is available already.
     * @return one of (X)YMODEM codes
     */
    YmodemCode_t peekCode(uint32_t timeout);

    /**
     * Calculate CRC16 and set it to appropriate position. Detection of 128 and 1024 data blocks made by SOH/STX byte in header
     */
    void setCRC();
    
    /**
     * Calculate and set checksum. Used only for plain XMODEM mode.
     */
    void setChecksum();

    /**
     * Actual transmitting of the frame with iSerialIO interface.
     */
    void transmitFrame ();
    
    /**
     * Send one frame of data.
     * @return - result code of operation
     */
    FileSendStatus_t sendBlock(bool header);

public:
    // Constructor
    EmYSend () = default;
    
    // Destructor
    virtual ~ EmYSend () = default;

    /**
     * Initialization.
     * @param system - pointer to object with iSystem interface
     * @param port - pointer to object with iSerialIO interface
     */
    void init(iSystem* system, iSerialIO* port);

    /**
     * Getter for current main statemachine stage.
     * @return - current stage of the statemachine
     */
    EmYSendStage_t getStage() { return stage; }

    /**
     * Check if the interface is ready to start file transfer, or is busy with transferring a file.
     * @return true if it is clear to start transfer.
     */
    bool isIdle() override { return stage == EMYSTAGE_IDLE; }

    /**
     * Check if receiver data must be fed here.
     * @return true if the sender waits for receiver input.
     */
    bool isInputNeeded () override { return (stage == EMYSTAGE_WAIT_START) ||
                                            (stage == EMYSTAGE_HEADER)     ||
                                            (stage == EMYSTAGE_BODY)       ||
                                            (stage == EMYSTAGE_STOP_BATCH) ||
                                            (stage == EMYSTAGE_FINAL); }

    /**
     * Reset the state machine of the interface.
     */
    void reset() override;

    /**
     * Initiate a new file send transfer. If return value is FSS_ACTIVE, host have to actually read the
     * file in any convenient portions and feed here via send() method.
     * On FSS_ERROR the finish() method must be called.
     * On FSS_ABORT the abort() method must be called.
     * @param filename - name of file to send.
     * @param size - size of file in bytes.
     * @return Status of operation
     */
    FileSendStatus_t start(char const* filename, size_t size) override;

    /**
     * The host may ask for an input buffer to read the file to. My be useful in memory-constrained
     * systems. If such a buffer used, the host have to provide nullptr as the buffer pointer in
     * send() method.
     * @param bufPtr - pointer to pointer to the buffer. Will be written with pointer to the buffer
     * @param size - pointer to buffer size variable.  Will be written with maximum available size
     */
    void getBuffer(uint8_t** bufPtr, size_t* size) override;

    /**
     * Main working horse for sending. The host have to read the file in any convenient buffer, then
     * feed here. If the internal buffer is used, the pointer must be set as nullptr.
     * If the return value is FXS_ACTIVE, host must provide the next portion of data,
     * with next call of the method.
     * On FSS_FINAL host must call the finish() method.
     * On FSS_ABORT host must call the abort() method.
     * The calling thread will be blocked until
     * @param buffer - pointer to the data buffer
     * @param size - size of data to send
     * @return Status of operation
     */
    FileSendStatus_t send (uint8_t* buffer, size_t size) override;

    /**
     * Finish the transfer. Must be called on receiving status FSS_FINAL.
     * @return FSS_SUCCESS on a successfull transfer and FSS_ABORTED on non-successfull.
     */
    FileSendStatus_t finish() override;


    /**
     * Abort the transfer. Must be called on receiving FSS_ABORTED status.
     * @return always returns FSS_ABORTED 
     */
    FileSendStatus_t abort() override;

    /**
     * Callback for reading the remote data. The serial interface reading thread must
     * check the status of the iFileXfer object with isInputNeeded() method and direct all input data here
     * if it is true.
     */
    void readRemote(uint8_t byte) override;

};

#endif // EMYMODEM_H
