// modbus_sensor.hpp
// This is a "header-only" wrapper class for libmodbus.
// It opens a Modbus-RTU connection and lets you read/write holding registers.

#pragma once                 // Tells the compiler to include this file only once
#include <modbus/modbus.h>  // The libmodbus C library header
#include <cstdint>          // For fixed-size integer types like uint16_t
#include <memory>           // For std::unique_ptr (smart pointer)
#include <stdexcept>        // For throwing exceptions on errors
#include <string>           // For std::string
#include <vector>           // For std::vector

// ============================================================================
// ModbusCtxDeleter
// ============================================================================
// libmodbus gives us a raw pointer of type modbus_t*.
// When we are done with it, we MUST call:
//   1. modbus_close(ctx)  - closes the serial port
//   2. modbus_free(ctx)   - frees the memory libmodbus allocated
//
// std::unique_ptr normally only knows how to delete simple pointers with `delete`.
// This small struct teaches unique_ptr how to clean up a modbus_t* properly.
// ============================================================================
struct ModbusCtxDeleter {

    // operator() makes an object callable like a function.
    // unique_ptr will call this automatically when it is destroyed.
    void operator()(modbus_t* ctx) const noexcept {

        // ctx could be nullptr, so we check before using it.
        if (ctx) {

            // Close the serial connection / socket.
            modbus_close(ctx);

            // Free the memory that libmodbus allocated for this context.
            modbus_free(ctx);
        }
    }
};

// ============================================================================
// ModbusSensor class
// ============================================================================
// RAII wrapper for a Modbus-RTU sensor.
// "RAII" means: the connection is opened in the constructor
// and automatically closed in the destructor.
// ============================================================================
class ModbusSensor {

private:
    // unique_ptr owns the modbus_t pointer.
    // When the ModbusSensor object is destroyed, ModbusCtxDeleter runs.
    std::unique_ptr<modbus_t, ModbusCtxDeleter> ctx_;

public:
    // ------------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------------
    // port      : serial device, e.g. "/dev/serial0"
    // baud      : baud rate, e.g. 115200
    // parity    : 'N' (none), 'E' (even), or 'O' (odd)
    // data_bits : usually 8
    // stop_bits : usually 1
    // slave     : Modbus slave ID of the sensor, usually 1
    // timeout_ms: how long to wait for a sensor reply
    // ------------------------------------------------------------------------
    ModbusSensor(const std::string& port, int baud, char parity,
                 int data_bits, int stop_bits, uint8_t slave, int timeout_ms)
        // The member initializer list runs before the constructor body.
        // Here we create the modbus context and store it in unique_ptr.
        : ctx_(modbus_new_rtu(port.c_str(), baud, parity, data_bits, stop_bits))
    {
        // modbus_new_rtu returns nullptr if it could not allocate the context.
        if (!ctx_) {
            throw std::runtime_error("modbus_new_rtu failed");
        }

        // Tell libmodbus which slave we are talking to.
        // If this fails, we throw an exception so the caller knows.
        if (modbus_set_slave(ctx_.get(), slave) == -1) {
            throw std::runtime_error(std::string("set_slave failed: ") + modbus_strerror(errno));
        }

        // Set the response timeout.
        // libmodbus wants (seconds, microseconds), so we convert ms to us.
        modbus_set_response_timeout(ctx_.get(), 0, timeout_ms * 1000);

        // Open the serial port.
        if (modbus_connect(ctx_.get()) == -1) {
            throw std::runtime_error(std::string("connect failed: ") + modbus_strerror(errno));
        }
    }

    // ------------------------------------------------------------------------
    // readHolding
    // ------------------------------------------------------------------------
    // Reads a single holding register at address 'addr'.
    // Returns the 16-bit value stored there.
    // ------------------------------------------------------------------------
    uint16_t readHolding(uint16_t addr) {
        uint16_t val = 0;  // Variable that will receive the register value.

        // modbus_read_registers(context, start_address, quantity, buffer)
        // We read 1 register into &val.
        // If it returns -1, the read failed.
        if (modbus_read_registers(ctx_.get(), addr, 1, &val) == -1) {
            throw std::runtime_error(std::string("read failed: ") + modbus_strerror(errno));
        }

        return val;
    }

    // ------------------------------------------------------------------------
    // readHoldings
    // ------------------------------------------------------------------------
    // Reads 'count' consecutive holding registers starting at 'addr'.
    // Returns them as a vector of uint16_t values.
    // ------------------------------------------------------------------------
    std::vector<uint16_t> readHoldings(uint16_t addr, int count) {
        // Create a vector with 'count' elements, all zero-initialized.
        std::vector<uint16_t> vals(count);

        // Read 'count' registers into the vector's internal data buffer.
        if (modbus_read_registers(ctx_.get(), addr, count, vals.data()) == -1) {
            throw std::runtime_error(std::string("bulk read failed: ") + modbus_strerror(errno));
        }

        return vals;
    }

    // ------------------------------------------------------------------------
    // writeHolding
    // ------------------------------------------------------------------------
    // Writes a 16-bit value into a single holding register.
    // ------------------------------------------------------------------------
    void writeHolding(uint16_t addr, uint16_t value) {
        // modbus_write_register(context, address, value)
        if (modbus_write_register(ctx_.get(), addr, value) == -1) {
            throw std::runtime_error(std::string("write failed: ") + modbus_strerror(errno));
        }
    }

    // ------------------------------------------------------------------------
    // flush
    // ------------------------------------------------------------------------
    // Discards any bytes already sitting in the serial port buffer.
    // Useful because some sensors keep streaming data even when idle.
    // ------------------------------------------------------------------------
    void flush() {
        modbus_flush(ctx_.get());
    }
};
