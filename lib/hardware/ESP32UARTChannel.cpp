#include "ESP32UARTChannel.h"
#include "fnSystem.h"
#include "../../include/pinmap.h"
#include "../../include/debug.h"

#include <soc/uart_reg.h>
#include <hal/gpio_types.h>
#include <soc/uart_struct.h>
#include <hal/uart_ll.h>

#define MAX_FLUSH_WAIT_TICKS 200

// Serial "debug port"
ESP32UARTChannel fnDebugConsole;

// Structure to hold received byte and timestamp
typedef struct
{
    uint8_t byte;
    int64_t timestamp_us;
} uart_rx_data_t;

// ISR - must be in IRAM for fastest execution
static void IRAM_ATTR uart_isr_handler(void *arg)
{
    uart_isr_context_t *ctx = (uart_isr_context_t *)arg;
    uart_dev_t *uart = (ctx->uart_num == 0) ? &UART0 : (ctx->uart_num == 1) ? &UART1 : &UART2;

    uint32_t status = uart->int_st.val;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check if RX FIFO has data
    if (status & (UART_RXFIFO_FULL_INT_ST_M | UART_RXFIFO_TOUT_INT_ST_M))
    {
        // Read all available bytes from FIFO
        while (uart->status.rxfifo_cnt > 0)
        {
            uart_rx_data_t rx_data;

            // Read byte and timestamp it
            rx_data.byte = uart->fifo.rw_byte;
            rx_data.timestamp_us = esp_timer_get_time();

            if (!ctx->discard_rx)
            {
                // Send to queue
                xQueueSendFromISR(ctx->rx_queue, &rx_data, &xHigherPriorityTaskWoken);
            }
        }

        // Clear interrupt flags
        uart->int_clr.rxfifo_full = 1;
        uart->int_clr.rxfifo_tout = 1;
    }

    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

void ESP32UARTChannel::begin(const ChannelConfig& conf)
{
    if (_uart_q)
        end();

    _uart_num = conf.device;
    read_timeout_ms = conf.read_timeout_ms;
    discard_timeout_ms = conf.discard_timeout_ms;
    Debug_printv("speed: %i", conf.uart_config.baud_rate);
    uart_param_config(_uart_num, &conf.uart_config);

    int tx, rx;
    if (_uart_num == 0)
    {
        rx = PIN_UART0_RX;
        tx = PIN_UART0_TX;
    }
    else if (_uart_num == 1)
    {
        rx = PIN_UART1_RX;
        tx = PIN_UART1_TX;
    }
    else if (_uart_num == 2)
    {
        rx = PIN_UART2_RX;
        tx = PIN_UART2_TX;
    }
    else
    {
        return;
    }

    uart_set_pin(_uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if (conf.isInverted)
        uart_set_line_inverse(_uart_num, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);

    halfDuplex = conf.isHalfDuplex;
    controlPins = conf.pins;

    if (controlPins.rts >= 0)
        fnSystem.set_pin_mode(controlPins.rts, gpio_mode_t::GPIO_MODE_INPUT);
    if (controlPins.cts >= 0)
    {
        fnSystem.set_pin_mode(controlPins.cts, gpio_mode_t::GPIO_MODE_OUTPUT);
        fnSystem.digital_write(controlPins.cts, DIGI_LOW);
    }

    if (controlPins.dtr >= 0)
        fnSystem.set_pin_mode(controlPins.dtr, gpio_mode_t::GPIO_MODE_INPUT);
    if (controlPins.dsr >= 0)
    {
        fnSystem.set_pin_mode(controlPins.dsr, gpio_mode_t::GPIO_MODE_OUTPUT);
        fnSystem.digital_write(controlPins.dsr, DIGI_LOW);
    }

    if (controlPins.dcd >= 0)
    {
        fnSystem.set_pin_mode(controlPins.dcd, gpio_mode_t::GPIO_MODE_OUTPUT);
        fnSystem.digital_write(controlPins.dcd, DIGI_HIGH);
    }

    if (controlPins.ri >= 0)
    {
        fnSystem.set_pin_mode(controlPins.ri, gpio_mode_t::GPIO_MODE_OUTPUT);
        fnSystem.digital_write(controlPins.ri, DIGI_HIGH);
    }

    if (_uart_num == FN_UART_DEBUG)
    {
        // Arduino default buffer size is 256
        int uart_buffer_size = UART_HW_FIFO_LEN(uart_num) * 2;
        int uart_queue_size = 10;
        int intr_alloc_flags = 0;

        // Install UART driver using an event queue here
        uart_driver_install(_uart_num, uart_buffer_size, 0, uart_queue_size, &_uart_q,
                            intr_alloc_flags);

        if (conf.rx_threshold)
            setRXThreshold(conf.rx_threshold);
    }
    else
    {
        // Setup ISR
        isr_context.uart_num = _uart_num;
        isr_context.discard_rx = false;
        isr_context.rx_queue = xQueueCreate(128, sizeof(uart_rx_data_t));
        // Get UART hardware register base
        uart_dev_t *uart = (_uart_num == 0) ? &UART0 : (_uart_num == 1) ? &UART1 : &UART2;

        // Set RX FIFO threshold directly (trigger on 1 byte)
        uart->conf1.rxfifo_full_thrhd = 1;

        // Clear any pending interrupts
        uart->int_clr.val = 0xffffffff;

        // Enable RX interrupts directly
        uart->int_ena.rxfifo_full = 1;
        uart->int_ena.rxfifo_tout = 1;

        // Allocate ISR
        int intr_source = (_uart_num == 0) ? ETS_UART0_INTR_SOURCE :
            (_uart_num == 1) ? ETS_UART1_INTR_SOURCE :
            ETS_UART2_INTR_SOURCE;
        esp_intr_alloc(intr_source, ESP_INTR_FLAG_IRAM, uart_isr_handler, &isr_context, NULL);
    }

    return;
}

void ESP32UARTChannel::end()
{
    uart_driver_delete(_uart_num);
    if (_uart_q)
        vQueueDelete(_uart_q);
    _uart_q = NULL;
}

#ifdef PRE_ISR
#ifdef CONFIG_IDF_TARGET_ESP32S3
void ESP32UARTChannel::updateFIFO()
{
    uart_event_t event;

    while (xQueueReceive(_uart_q, &event, 1))
    {
        if (event.type == UART_DATA)
        {
            size_t old_len = _fifo.size();
            _fifo.resize(old_len + event.size);
            int result = uart_read_bytes(_uart_num, &_fifo[old_len], event.size, 0);
            if (result < 0)
                result = 0;
            _fifo.resize(old_len + result);
        }
    }

    return;
}
#else /* ! CONFIG_IDF_TARGET_ESP32S3 */
void ESP32UARTChannel::updateFIFO()
{
    size_t avail;
    if (ESP_FAIL == uart_get_buffered_data_len(_uart_num, &avail))
        return;

    size_t old_len = _fifo.size();
    _fifo.resize(old_len + avail);
    int result = uart_read_bytes(_uart_num, &_fifo[old_len], avail, 0);
    if (result < 0)
        result = 0;
    _fifo.resize(old_len + result);

    return;
}
#endif /* CONFIG_IDF_TARGET_ESP32S3 */
#else /* ! PRE_ISR */
void ESP32UARTChannel::updateFIFO()
{
    uart_rx_data_t event;

    //Debug_printf("updateFIFO avail %d\n", uxQueueMessagesWaiting(isr_context.rx_queue));

    while (xQueueReceive(isr_context.rx_queue, &event, 0))
    {
        _fifo += (char) event.byte;
        lastByteTimestamp = event.timestamp_us;
        //Debug_printf("updateFIFO added byte %02x from %lld\n", event.byte, event.timestamp_us);
    }

    return;
}
#endif /* PRE_ISR */

void ESP32UARTChannel::flushOutput()
{
    if (_uart_num == FN_UART_DEBUG)
        uart_wait_tx_done(_uart_num, MAX_FLUSH_WAIT_TICKS);

    uart_dev_t *uart = UART_LL_GET_HW(_uart_num);
    while (!uart_ll_is_tx_idle(uart))
        ;
}

uint32_t ESP32UARTChannel::getBaudrate()
{
    uint32_t baud;
    uart_get_baudrate(_uart_num, &baud);
    return baud;
}

void ESP32UARTChannel::setBaudrate(uint32_t baud)
{
#ifdef DEBUG
    uint32_t before;
    uart_get_baudrate(_uart_num, &before);
#endif
    uart_set_baudrate(_uart_num, baud);
#ifdef DEBUG
    Debug_printf("set_baudrate change from %d to %d\r\n", before, baud);
#endif
}

size_t ESP32UARTChannel::dataOut(const void *buffer, size_t size)
{
    if (_uart_num == FN_UART_DEBUG)
        return uart_write_bytes(_uart_num, (const char *)buffer, size);

    if (halfDuplex)
        isr_context.discard_rx = true;

    uart_dev_t *uart = UART_LL_GET_HW(_uart_num);
    size_t count = 0;
    uint8_t *data = (uint8_t *) buffer;
    while (count < size) {
        // How many bytes can we write now without overflowing FIFO?
        size_t space = uart_ll_get_txfifo_len(uart);

        if (space > 0) {
            size_t chunk = std::min(size - count, space);
            uart_ll_write_txfifo(uart, &data[count], chunk);
            count += chunk;
        }
    }

    if (halfDuplex)
    {
        flushOutput();
        isr_context.discard_rx = false;
    }

    return count;
}

bool ESP32UARTChannel::getPin(int pin)
{
    if (pin < 0)
        return 0;
    return fnSystem.digital_read(pin) == DIGI_LOW;
}

void ESP32UARTChannel::setPin(int pin, bool state)
{
    if (pin >= 0)
        fnSystem.digital_write(pin, !state);
    return;
}

bool ESP32UARTChannel::getDTR()
{
    return getPin(controlPins.dtr);
}

void ESP32UARTChannel::setDSR(bool state)
{
    setPin(controlPins.dsr, state);
}

bool ESP32UARTChannel::getRTS()
{
    return getPin(controlPins.rts);
}

void ESP32UARTChannel::setCTS(bool state)
{
    setPin(controlPins.cts, state);
}

void ESP32UARTChannel::setDCD(bool state)
{
    setPin(controlPins.dcd, state);
}

void ESP32UARTChannel::setRI(bool state)
{
    setPin(controlPins.ri, state);
}

void ESP32UARTChannel::setRXThreshold(uint8_t thresh)
{
    uint32_t conf1 = READ_PERI_REG(UART_CONF1_REG(_uart_num));
    conf1 &= ~(UART_RXFIFO_FULL_THRHD_V << UART_RXFIFO_FULL_THRHD_S);
    conf1 |= (thresh & UART_RXFIFO_FULL_THRHD_V) << UART_RXFIFO_FULL_THRHD_S;
    WRITE_PERI_REG(UART_CONF1_REG(_uart_num), conf1);
}

uint8_t ESP32UARTChannel::getRXThreshold()
{
    uint32_t conf1 = READ_PERI_REG(UART_CONF1_REG(_uart_num));
    return (conf1 >> UART_RXFIFO_FULL_THRHD_S) & UART_RXFIFO_FULL_THRHD_V;
}
