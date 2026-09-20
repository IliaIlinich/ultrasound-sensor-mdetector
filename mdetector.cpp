// mdetector.cpp
// Service binary that reads the ultrasound sensor and detects motion.

#include "modbus_sensor.hpp"  // Our shared wrapper
#include <atomic>             // For std::atomic (thread-safe flag)
#include <chrono>             // For time durations and clocks
#include <cmath>              // For std::abs
#include <csignal>            // For std::signal
#include <cstdio>             // For std::remove (delete a file)
#include <cstring>            // For C-string functions
#include <deque>              // For std::deque (sliding window buffer)
#include <fstream>            // For file output
#include <getopt.h>           // For command-line parsing
#include <iomanip>            // For output formatting
#include <iostream>           // For console output
#include <sstream>            // For string streams
#include <thread>             // For std::this_thread::sleep_for

// ============================================================================
// Global flag used to stop the program gracefully.
// ============================================================================
// "atomic" means reads/writes are safe even if a signal interrupts the program.
static std::atomic<bool> g_running{true};

// ============================================================================
// signal_handler
// ============================================================================
// This function runs when the user presses Ctrl+C (SIGINT)
// or when systemd sends SIGTERM to stop the service.
// ============================================================================
void signal_handler(int) {
    g_running = false;  // Tell the main loop to exit
}

// ============================================================================
// Options struct
// ============================================================================
// All runtime settings for the service.
// Everything has a default so you can run the binary with no arguments.
// ============================================================================
struct Options {
    std::string port = "/dev/serial0";          // Serial port
    int baud = 115200;                          // Baud rate
    uint8_t slave = 1;                          // Modbus slave ID
    int timeout_ms = 500;                       // Modbus reply timeout

    int interval_ms = 100;                      // How often to poll the sensor
    int avg_seconds = 5;                          // Baseline averaging window
    int jitter_mm = 50;                           // Deviation that counts as motion
    int distance_threshold_mm = 350;              // Closer than this can be motion

    std::string flag_file = "/tmp/motion.flg";    // Created when motion detected
    std::string unflag_file = "/tmp/no-motion.flg"; // Created when no motion
    std::string dump_file = "/dev/shm/mdetector.txt"; // Telemetry file
};

// ============================================================================
// print_usage
// ============================================================================
static void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog << " [options]\n"
              << "  --port           serial port (default /dev/serial0)\n"
              << "  --baud           baud rate (default 115200)\n"
              << "  --slave          Modbus slave id (default 1)\n"
              << "  --interval       poll interval ms (default 100)\n"
              << "  --average        averaging window seconds (default 5)\n"
              << "  --jitter         motion detection jitter mm (default 50)\n"
              << "  --distance       distance threshold mm (default 350)\n"
              << "  --flag           motion flag file (default /tmp/motion.flg)\n"
              << "  --unflag         no-motion flag file (default /tmp/no-motion.flg)\n"
              << "  --dump           telemetry dump file (default /dev/shm/mdetector.txt)\n";
}

// ============================================================================
// average
// ============================================================================
// Computes the arithmetic mean of the values in a deque.
// A deque is like a vector but efficient at both ends,
// which makes it good for a sliding window.
// ============================================================================
static double average(const std::deque<uint16_t>& buf) {
    if (buf.empty()) {
        return 0.0;  // Avoid dividing by zero
    }

    double sum = 0.0;
    for (auto value : buf) {
        sum += static_cast<double>(value);  // Cast to double for precise division
    }

    return sum / static_cast<double>(buf.size());
}

// ============================================================================
// set_flag
// ============================================================================
// Creates a file if active is true, otherwise deletes it.
// This is how the service communicates "motion" / "no motion" to other tools.
// ============================================================================
static void set_flag(const std::string& path, bool active) {
    if (active) {
        // Open and immediately close the file; just creating it is enough.
        std::ofstream f(path);
    } else {
        // Delete the file if it exists.
        std::remove(path.c_str());
    }
}

// ============================================================================
// write_dump
// ============================================================================
// Writes a one-line telemetry snapshot to a file.
// Other programs can read this file to see current state.
// ============================================================================
static void write_dump(const std::string& path, uint16_t current,
                       double avg, bool motion) {

    // Get the current wall-clock time.
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);

    // Open the file, truncating any previous contents.
    std::ofstream f(path, std::ios::trunc);
    if (!f) {
        return;  // If we cannot write, just skip silently
    }

    // Write a human-readable line with timestamp, distance, average, motion flag.
    f << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S")
      << " dist=" << current
      << " avg=" << std::fixed << std::setprecision(1) << avg
      << " motion=" << (motion ? 1 : 0) << "\n";
}

// ============================================================================
// main
// ============================================================================
int main(int argc, char** argv) {

    Options opt;  // Create options object with defaults.

    // ------------------------------------------------------------------------
    // Long-options table. We use getopt_long_only, which accepts
    // --option style arguments (with two dashes).
    // ------------------------------------------------------------------------
    static struct option long_opts[] = {
        {"port",     required_argument, nullptr, 0},
        {"baud",     required_argument, nullptr, 0},
        {"slave",    required_argument, nullptr, 0},
        {"interval", required_argument, nullptr, 0},
        {"average",  required_argument, nullptr, 0},
        {"jitter",   required_argument, nullptr, 0},
        {"distance", required_argument, nullptr, 0},
        {"flag",     required_argument, nullptr, 0},
        {"unflag",   required_argument, nullptr, 0},
        {"dump",     required_argument, nullptr, 0},
        {"help",     no_argument,       nullptr, 0},
        {nullptr,    0,                 nullptr, 0}   // End of table
    };

    int idx;  // Index of the matched long option.

    // Loop until getopt_long_only returns -1 (no more options).
    while (true) {
        int c = getopt_long_only(argc, argv, "", long_opts, &idx);

        if (c == -1) {
            break;  // No more options
        }

        // c == 0 means a long option was matched; 'idx' tells us which one.
        if (c != 0) {
            continue;
        }

        std::string name = long_opts[idx].name;  // e.g. "port"
        std::string val  = optarg ? optarg : "";  // The argument value, or empty

        // Update the matching field in the Options struct.
        if (name == "port") {
            opt.port = val;
        } else if (name == "baud") {
            opt.baud = std::stoi(val);
        } else if (name == "slave") {
            opt.slave = static_cast<uint8_t>(std::stoi(val));
        } else if (name == "interval") {
            opt.interval_ms = std::stoi(val);
        } else if (name == "average") {
            opt.avg_seconds = std::stoi(val);
        } else if (name == "jitter") {
            opt.jitter_mm = std::stoi(val);
        } else if (name == "distance") {
            opt.distance_threshold_mm = std::stoi(val);
        } else if (name == "flag") {
            opt.flag_file = val;
        } else if (name == "unflag") {
            opt.unflag_file = val;
        } else if (name == "dump") {
            opt.dump_file = val;
        } else if (name == "help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    // ------------------------------------------------------------------------
    // Register signal handlers so Ctrl+C / systemctl stop shut down cleanly.
    // ------------------------------------------------------------------------
    std::signal(SIGINT, signal_handler);   // Ctrl+C
    std::signal(SIGTERM, signal_handler);  // systemd stop

    // ------------------------------------------------------------------------
    // Main loop: connect to the sensor and poll forever.
    // ------------------------------------------------------------------------
    try {
        // Open the Modbus connection.
        ModbusSensor sensor(opt.port, opt.baud, 'N', 8, 1,
                            opt.slave, opt.timeout_ms);

        // Calculate how many samples fit into the averaging window.
        // Example: 5 seconds * 1000 ms / 100 ms interval = 50 samples.
        const int window_size = std::max(1, opt.avg_seconds * 1000 / opt.interval_ms);

        std::deque<uint16_t> history;  // Sliding window of recent distances.
        bool motion = false;             // Current motion state.

        // Keep looping until a signal sets g_running to false.
        while (g_running) {

            // Throw away any stale bytes that arrived before our request.
            sensor.flush();

            // Read the real-time distance register (0x0101).
            uint16_t d = sensor.readHolding(0x0101);

            // Add the new sample to the sliding window.
            history.push_back(d);

            // If the window grew too large, drop the oldest sample.
            if (static_cast<int>(history.size()) > window_size) {
                history.pop_front();
            }

            // Compute the average distance over the window.
            double avg = average(history);

            // Wait until we have enough samples before declaring motion.
            bool enough_samples = static_cast<int>(history.size()) >= window_size / 2;

            bool new_motion = false;

            if (enough_samples) {
                // Motion logic:
                // 1. Something must be close (below the threshold).
                // 2. The current reading must differ from the average by more than jitter.
                bool close = d < opt.distance_threshold_mm;
                bool changed = std::abs(static_cast<double>(d) - avg) > opt.jitter_mm;
                new_motion = close && changed;
            }

            // If the state changed, update the flag files and log to console.
            if (new_motion != motion) {
                motion = new_motion;

                set_flag(opt.flag_file, motion);
                set_flag(opt.unflag_file, !motion);

                std::cout << (motion ? "MOTION" : "NO MOTION")
                          << " dist=" << d << " avg=" << avg << "\n";
            }

            // Always write the telemetry dump file.
            write_dump(opt.dump_file, d, avg, motion);

            // Sleep until the next poll.
            std::this_thread::sleep_for(
                std::chrono::milliseconds(opt.interval_ms));
        }

        // --------------------------------------------------------------------
        // Clean shutdown: remove both flag files.
        // --------------------------------------------------------------------
        set_flag(opt.flag_file, false);
        set_flag(opt.unflag_file, false);

    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
