/*
 * i2c_driver.c -- Project 6
 *
 * TM4C123 I2C0 -- PB2 (SCL), PB3 (SDA), 400 kHz Fast Mode
 *
 * Project 6 change: fully interrupt driven.
 * Polling loop i2cWaitForBus() is replaced by a binary semaphore.
 * The calling task blocks on xSemaphoreTake() and is unblocked
 * by the ISR via xSemaphoreGiveFromISR() when the transaction ends.
 *
 * Public API is identical to Project 5 -- no other files change.
 */

#include "i2c_driver.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"

#include "FreeRTOS.h"
#include "semphr.h"

/*---------------------------------------------------------------------------
 * State machine states
 *
 * The ISR uses this to know where it is in the transaction and what
 * to do next. Think of it as a position marker in the I2C conversation.
 *
 * IDLE        -- no transaction in progress
 * WRITE_DATA  -- write transaction: just sent reg addr, now send data byte
 * READ_SWITCH -- read transaction: write phase done, switch to read mode
 * READ_CONT   -- burst read: receiving middle bytes
 * READ_LAST   -- burst read: receiving the final byte
 * DONE        -- transaction complete, give semaphore
 * ERROR       -- NACK or bus error detected, give semaphore with error flag
 *--------------------------------------------------------------------------*/
typedef enum
{
    I2C_STATE_IDLE = 0,
    I2C_STATE_WRITE_DATA,
    I2C_STATE_READ_SWITCH,
    I2C_STATE_READ_SINGLE,
    I2C_STATE_READ_CONT,
    I2C_STATE_READ_LAST,
    I2C_STATE_DONE,
    I2C_STATE_ERROR
} I2C_State_t;

/*---------------------------------------------------------------------------
 * Transaction context struct
 *
 * Shared between the public API functions and the ISR.
 * Public functions fill this before arming the hardware.
 * ISR reads and updates it on every interrupt.
 *
 * PITFALL: this struct is accessed from both task context and ISR context.
 * We avoid a race condition by ensuring the public function never touches
 * the struct while the ISR is running -- once i2cRunTransaction() arms
 * the hardware and calls xSemaphoreTake(), the public function is blocked
 * and the ISR owns the struct exclusively until it gives the semaphore.
 *--------------------------------------------------------------------------*/
typedef struct
{
    uint8_t         slaveAddr;      /* 7-bit I2C address of target device  */
    uint8_t         regAddr;        /* register address to read/write       */
    uint8_t        *pBuffer;        /* pointer to receive buffer            */
    uint8_t         bytesTotal;     /* total bytes to receive (burst read)  */
    uint8_t         bytesReceived;  /* bytes received so far                */
    uint8_t         writeData;      /* data byte for write transactions     */
    bool            isRead;         /* true = read, false = write           */
    bool            error;          /* set true by ISR if NACK detected     */
    I2C_State_t     state;          /* current state machine position       */
    SemaphoreHandle_t xSemaphore;   /* given by ISR when transaction ends   */
} I2CContext_t;

/* Single global context instance -- only one I2C transaction at a time.
 * This is safe because all callers go through i2cRunTransaction() which
 * blocks until the current transaction completes before returning. */
static I2CContext_t gContext;

/*---------------------------------------------------------------------------
 * i2cRunTransaction (private)
 *
 * Arms the hardware to start the transaction described in gContext,
 * then blocks the calling task until the ISR signals completion.
 *
 * This function is the bridge between task context and ISR context.
 * Everything above this line runs in task context.
 * Everything in the ISR runs in interrupt context.
 * The semaphore is the handoff point between them.
 *--------------------------------------------------------------------------*/
static bool i2cRunTransaction(void)
{
    /* Clear error flag before each transaction */
    gContext.error = false;
    gContext.bytesReceived = 0;

    if (gContext.isRead)
    {
        /* Read transaction -- Phase 1 is always a write of the register
         * address. Set write mode, load register address, fire START.
         * ISR will handle the switch to read mode after this phase. */
        gContext.state = I2C_STATE_READ_SWITCH;

        MAP_I2CMasterSlaveAddrSet(I2C0_BASE, gContext.slaveAddr, false);
        MAP_I2CMasterDataPut(I2C0_BASE, gContext.regAddr);
        MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    }
    else
    {
        /* Write transaction -- send register address first.
         * ISR will send the data byte after this phase. */
        gContext.state = I2C_STATE_WRITE_DATA;

        MAP_I2CMasterSlaveAddrSet(I2C0_BASE, gContext.slaveAddr, false);
        MAP_I2CMasterDataPut(I2C0_BASE, gContext.regAddr);
        MAP_I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    }

    /* Block here until ISR gives the semaphore.
     *
     * portMAX_DELAY -- wait forever. The ISR will always give the
     * semaphore eventually -- either on success or on error/timeout.
     *
     * PITFALL: This is why the scheduler must be running before any
     * I2C transaction. xSemaphoreTake with portMAX_DELAY requires the
     * FreeRTOS scheduler to be active to block the calling task and
     * allow other tasks to run while we wait. Without the scheduler
     * this call hangs forever. */
    xSemaphoreTake(gContext.xSemaphore, portMAX_DELAY);

    /* We are back -- ISR has completed the transaction.
     * Return true if no error was flagged by the ISR. */
    return !gContext.error;
}

/*---------------------------------------------------------------------------
 * I2C_init
 *
 * Same peripheral setup as Project 5 with two additions:
 *   1. Binary semaphore created for task/ISR synchronization
 *   2. I2C0 interrupt enabled in NVIC with correct priority
 *--------------------------------------------------------------------------*/
void I2C_init(void)
{
    /* Create the binary semaphore used to synchronize task and ISR.
     *
     * Binary semaphore starts empty (count = 0).
     * ISR gives it when transaction completes.
     * Calling task takes it -- blocks until ISR gives it.
     *
     * PITFALL: Must be created before the I2C interrupt is enabled.
     * If the interrupt fires before the semaphore exists, the ISR
     * will call xSemaphoreGiveFromISR on a NULL handle -- hard fault. */
    gContext.xSemaphore = xSemaphoreCreateBinary();
    configASSERT(gContext.xSemaphore != NULL);

    gContext.state = I2C_STATE_IDLE;

    /* Peripheral clock enables */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    SysCtlDelay(1000);
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    /* Pin routing and electrical configuration */
    MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    MAP_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    /* Clock speed -- 400 kHz Fast Mode */
    MAP_I2CMasterInitExpClk(I2C0_BASE, MAP_SysCtlClockGet(), true);

    /* Enable I2C master interrupt in the I2C peripheral.
     * This arms the I2C0 hardware to generate an interrupt signal
     * to the NVIC when a transaction phase completes. */
    MAP_I2CMasterIntEnable(I2C0_BASE);

    /* Set I2C0 interrupt priority in the NVIC.
     *
     * PITFALL -- critical priority rule:
     * Any ISR that calls FreeRTOS ISR-safe APIs (xSemaphoreGiveFromISR)
     * must have a priority numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY.
     * In our FreeRTOSConfig.h:
     *   configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
     *   configMAX_SYSCALL_INTERRUPT_PRIORITY = 5 << 5 = 0xA0
     *
     * We set I2C0 to exactly 0xA0 -- the boundary value.
     * Setting it lower numerically (e.g. 0x80 = priority 4) would
     * mean the ISR runs at a priority above the FreeRTOS kernel and
     * calling xSemaphoreGiveFromISR would corrupt the kernel state.
     *
     * Higher numerically (0xC0, 0xE0) is also safe but gives the
     * ISR lower urgency. 0xA0 is the highest safe priority. */
    MAP_IntPrioritySet(INT_I2C0, 0xA0);

    /* Enable I2C0 interrupt in the NVIC */
    MAP_IntEnable(INT_I2C0);

    /* Global interrupt enable -- required for any interrupt to fire */
    MAP_IntMasterEnable();
}

/*---------------------------------------------------------------------------
 * I2C_writeByte
 *
 * Public API -- identical signature to Project 5.
 * Internally uses interrupt driven transaction instead of polling.
 *--------------------------------------------------------------------------*/
bool I2C_writeByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    gContext.slaveAddr = slaveAddr;
    gContext.regAddr   = regAddr;
    gContext.writeData = data;
    gContext.isRead    = false;
    gContext.pBuffer   = NULL;
    gContext.bytesTotal = 0;

    return i2cRunTransaction();
}

/*---------------------------------------------------------------------------
 * I2C_readByte
 *
 * Public API -- identical signature to Project 5.
 *--------------------------------------------------------------------------*/
bool I2C_readByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *dest)
{
    gContext.slaveAddr  = slaveAddr;
    gContext.regAddr    = regAddr;
    gContext.pBuffer    = dest;
    gContext.bytesTotal = 1;
    gContext.isRead     = true;

    return i2cRunTransaction();
}

/*---------------------------------------------------------------------------
 * I2C_readBurst
 *
 * Public API -- identical signature to Project 5.
 *--------------------------------------------------------------------------*/
bool I2C_readBurst(uint8_t slaveAddr, uint8_t regAddr,
                   uint8_t *dest, uint8_t length)
{
    if (length == 0) return false;

    gContext.slaveAddr  = slaveAddr;
    gContext.regAddr    = regAddr;
    gContext.pBuffer    = dest;
    gContext.bytesTotal = length;
    gContext.isRead     = true;

    return i2cRunTransaction();
}

/*---------------------------------------------------------------------------
 * I2C0IntHandler -- I2C0 Interrupt Service Routine
 *
 * Called by hardware after every I2C transaction phase completes.
 * Advances the state machine and either arms the next phase or
 * signals the waiting task that the transaction is done.
 *
 * RULES for this ISR -- never violate these:
 *   1. Never block -- no xSemaphoreTake, no vTaskDelay, no polling loops
 *   2. Only call ISR-safe FreeRTOS APIs -- xSemaphoreGiveFromISR only
 *   3. Always call portYIELD_FROM_ISR at the end
 *   4. Always clear the interrupt flag first thing
 *
 * PITFALL -- ISR name must match vector table exactly.
 * In startup_ccs.c the I2C0 handler is listed as I2C0IntHandler.
 * If the name does not match, the NVIC jumps to the default handler
 * which loops forever -- extremely hard to debug.
 *--------------------------------------------------------------------------*/
void I2C0IntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t ui32Status;

    /* Step 1 -- Clear the interrupt flag immediately.
     * Must be first. If you read data before clearing, the peripheral
     * may re-trigger the interrupt on the same event. */
    ui32Status = MAP_I2CMasterIntStatusEx(I2C0_BASE, true);
    MAP_I2CMasterIntClearEx(I2C0_BASE, ui32Status);

    /* Step 2 -- Check for errors.
     * If the slave sent a NACK or the bus has an error, abort
     * the transaction immediately regardless of current state. */
    if (MAP_I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE)
    {
        gContext.error = true;
        gContext.state = I2C_STATE_ERROR;
    }

    /* Step 3 -- Advance the state machine */
    switch (gContext.state)
    {
        case I2C_STATE_WRITE_DATA:
            /* Register address was just sent and ACKed.
             * Now send the actual data byte and generate STOP. */
            MAP_I2CMasterDataPut(I2C0_BASE, gContext.writeData);
            MAP_I2CMasterControl(I2C0_BASE,
                                 I2C_MASTER_CMD_BURST_SEND_FINISH);
            gContext.state = I2C_STATE_DONE;
            break;

        case I2C_STATE_READ_SWITCH:
            /* Write phase complete -- register pointer is set on the slave.
             * Switch to read mode with a repeated START.
             * Decide whether this is a single byte or burst read. */
            MAP_I2CMasterSlaveAddrSet(I2C0_BASE,
                                      gContext.slaveAddr, true);

            if (gContext.bytesTotal == 1)
            {
                /* Single byte -- use SINGLE_RECEIVE which generates
                 * NACK + STOP automatically after one byte */
                MAP_I2CMasterControl(I2C0_BASE,
                                     I2C_MASTER_CMD_SINGLE_RECEIVE);
                gContext.state = I2C_STATE_READ_SINGLE;
            }
            else
            {
                /* Burst -- start receiving, more bytes to follow */
                MAP_I2CMasterControl(I2C0_BASE,
                                     I2C_MASTER_CMD_BURST_RECEIVE_START);
                gContext.state = I2C_STATE_READ_CONT;
            }
            break;

        case I2C_STATE_READ_SINGLE:
            /* SINGLE_RECEIVE complete -- byte is now in MDR.
             * Read it out and store into destination buffer. */
            gContext.pBuffer[0] = (uint8_t)MAP_I2CMasterDataGet(I2C0_BASE);
            gContext.bytesReceived = 1;
            gContext.state = I2C_STATE_IDLE;
            xSemaphoreGiveFromISR(gContext.xSemaphore,
                                  &xHigherPriorityTaskWoken);
            break;

        case I2C_STATE_READ_CONT:
            /* A byte has arrived. Store it. */
            gContext.pBuffer[gContext.bytesReceived] =
                (uint8_t)MAP_I2CMasterDataGet(I2C0_BASE);
            gContext.bytesReceived++;

            /* Is this the second-to-last byte?
             * If so, the next receive must use FINISH to generate
             * NACK + STOP after the last byte. */
            if (gContext.bytesReceived == gContext.bytesTotal - 1)
            {
                MAP_I2CMasterControl(I2C0_BASE,
                                     I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                gContext.state = I2C_STATE_READ_LAST;
            }
            else
            {
                /* More middle bytes to receive */
                MAP_I2CMasterControl(I2C0_BASE,
                                     I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            }
            break;

        case I2C_STATE_READ_LAST:
            /* Final byte received. Store it. Transaction is complete. */
            gContext.pBuffer[gContext.bytesReceived] =
                (uint8_t)MAP_I2CMasterDataGet(I2C0_BASE);
            gContext.bytesReceived++;
            gContext.state = I2C_STATE_DONE;

            /* Fall through to DONE -- intentional, no break */

        case I2C_STATE_DONE:
        case I2C_STATE_ERROR:
            /* Transaction complete or failed.
             * Give the semaphore to unblock the waiting task.
             *
             * xHigherPriorityTaskWoken will be set to pdTRUE if
             * the task we are unblocking has higher priority than
             * the currently running task. portYIELD_FROM_ISR below
             * will then cause an immediate context switch so the
             * higher priority task runs without waiting for the
             * next scheduler tick. */
            gContext.state = I2C_STATE_IDLE;
            xSemaphoreGiveFromISR(gContext.xSemaphore,
                                  &xHigherPriorityTaskWoken);
            break;

        default:
            /* Unexpected state -- should never reach here.
             * Give semaphore with error flag to unblock caller
             * rather than hanging forever. */
            gContext.error = true;
            gContext.state = I2C_STATE_IDLE;
            xSemaphoreGiveFromISR(gContext.xSemaphore,
                                  &xHigherPriorityTaskWoken);
            break;
    }

    /* Step 4 -- Yield if a higher priority task was woken.
     * This ensures the sensor task runs immediately when the
     * transaction completes rather than waiting for the next tick. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
