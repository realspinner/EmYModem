/**
 * The interface declaration for a file transfer interface, designed to work in
 * multithreaded (e.g. RTOS-based) embedded system.
 */

#ifndef __IBLOCKXFER_H__
#define __IBLOCKXFER_H__


#include <cstdint>
#include <cstddef>


typedef enum _FileSendStatus_t {
    FSS_IDLE, FSS_ACTIVE, FSS_ERROR, FSS_FINAL, FSS_ABORTED, FSS_SUCCESS
} FileSendStatus_t;


class iFileSend {
public:
    virtual ~iFileSend() = default;

    /**
     * Check if the interface is ready to start file transfer, or is busy with transferring a file.
     * @return true if it is clear to start transfer.
     */
    virtual bool isIdle() = 0;

    /**
     * Check if receiver data must be fed here.
     * @return true if the sender waits for receiver input.
     */
    virtual bool isInputNeeded() = 0;

    /**
     * Reset the state machine of the interface.
     */
    virtual void reset() = 0;

    /**
     * Initiate a new file send transfer. If return value is FSS_ACTIVE, host have to actually read the
     * file in any convenient portions and feed here via send() method.
     * On FSS_ERROR the finish() method must be called.
     * On FSS_ABORT the abort() method must be called.
     * @param filename - name of file to send.
     * @param size - size of file in bytes.
     * @return Status of operation
     */
    virtual FileSendStatus_t start(char const* filename, size_t size) = 0;

    /**
     * The host may ask for an input buffer to read the file to. My be useful in memory-constrained
     * systems. If such a buffer used, the host have to provide nullptr as the buffer pointer in
     * send() method.
     * @param bufPtr - pointer to pointer to the buffer. Will be written with pointer to the buffer
     * @param size - pointer to buffer size variable.  Will be written with maximum available size
     */
    virtual void getBuffer(uint8_t** bufPtr, size_t* size) = 0;

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
    virtual FileSendStatus_t send (uint8_t* buffer, size_t size) = 0;

    /**
     * Finish the transfer. Must be called on receiving status FSS_FINAL.
     * @return FSS_SUCCESS on a successfull transfer and FSS_ABORTED on non-successfull.
     */
    virtual FileSendStatus_t finish() = 0;


    /**
     * Abort the transfer. Must be called on receiving FSS_ABORTED status.
     * @return always returns FSS_ABORTED 
     */
    virtual FileSendStatus_t abort() = 0;

    /**
     * Callback for reading the remote data. The serial interface reading thread must
     * check the status of the iFileXfer object with isInputNeeded() method and direct all input data here
     * if it is true.
     */
    virtual void readRemote(uint8_t byte) = 0;
};


#endif /* __IBLOCKXFER_H__ */
