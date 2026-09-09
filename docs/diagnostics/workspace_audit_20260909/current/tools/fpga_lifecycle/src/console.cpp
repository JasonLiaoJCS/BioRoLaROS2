#include "console.hpp"
#include "driver_lifecycle.hpp"
#include <chrono>
#include <cerrno>
#include <ncurses.h>
#include <algorithm>
#include <climits>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace {
bool number(const std::string& text, long& value) {
    if (text.empty() || text.size() > 10) return false;
    for (char c : text) if (c < '0' || c > '9') return false;
    char* end = nullptr;
    value = std::strtol(text.c_str(), &end, 10);
    return end && *end == '\0' && value >= 0 && value < LONG_MAX;
}
void line(int row, const std::string& text, int width) {
    if (row >= 0 && row < LINES && width > 1)
        mvaddnstr(row, 0, text.c_str(), width - 1);
}
}

void Console::init(FpgaHandler* fpga, std::vector<bool>* power, std::mutex* mutex) {
    fpga_ = fpga;
    power_ = power;
    device_mutex_ = mutex;
    // Headless launches keep their ordinary stdout log and create no curses UI.
    if (lifecycle::headless || !isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) return;
    terminal_out_ = fdopen(dup(STDOUT_FILENO), "w");
    terminal_in_ = fdopen(dup(STDIN_FILENO), "r");
    saved_stdout_ = dup(STDOUT_FILENO);
    saved_stderr_ = dup(STDERR_FILENO);
    if (!terminal_out_ || !terminal_in_ || saved_stdout_ < 0 || saved_stderr_ < 0) {
        stop();
        throw std::runtime_error("Cannot open dedicated console terminal");
    }
    // A full/closed UI output must not prevent safe shutdown from joining us.
    int flags = fcntl(fileno(terminal_out_), F_GETFL);
    original_stdout_flags_ = flags;
    if (flags < 0 || fcntl(fileno(terminal_out_), F_SETFL, flags | O_NONBLOCK) < 0)
        throw std::runtime_error("Cannot make console output nonblocking");
#ifndef FPGA_CONSOLE_LOG
#define FPGA_CONSOLE_LOG "/tmp/fpga_driver_console.log"
#endif
    const char* log_path = FPGA_CONSOLE_LOG;
    int log_fd = open(log_path, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (log_fd < 0) {
        stop();
        throw std::runtime_error("Cannot open FPGA console log");
    }
    std::cout.flush();
    std::cerr.flush();
    fflush(nullptr);
    const bool redirected = dup2(log_fd, STDOUT_FILENO) >= 0 &&
                            dup2(log_fd, STDERR_FILENO) >= 0;
    close(log_fd);
    if (!redirected) {
        stop();
        throw std::runtime_error("Cannot redirect FPGA console log");
    }
    running_ = true;
    ui_ = std::thread(&Console::run, this);
}

Console::~Console() { stop(); }

void Console::stop() {
    running_ = false;
    if (ui_.joinable()) ui_.join();
    std::cout.flush();
    std::cerr.flush();
    fflush(nullptr);
    // Flush/close the UI while it is still nonblocking. Restore the shared
    // open-file flags via the saved fd so an interactive parent shell is unchanged.
    if (terminal_out_) { fclose(terminal_out_); terminal_out_ = nullptr; }
    if (saved_stdout_ >= 0) {
        if (original_stdout_flags_ >= 0)
            fcntl(saved_stdout_, F_SETFL, original_stdout_flags_);
        // Keep lifecycle/shutdown evidence in the log after the TTY disappears.
        close(saved_stdout_);
        saved_stdout_ = -1;
    }
    if (saved_stderr_ >= 0) {
        // Do not restore stderr to a disconnected terminal.
        close(saved_stderr_);
        saved_stderr_ = -1;
    }
    if (terminal_in_) { fclose(terminal_in_); terminal_in_ = nullptr; }
}

std::string Console::execute(const std::string& input) {
    if (lifecycle::stopping()) return "STOPPING: command rejected";
    std::string text = input;
    if (!text.empty() && text.front() == ':') text.erase(0, 1);
    std::istringstream stream(text);
    std::vector<std::string> words;
    for (std::string word; stream >> word;) words.push_back(word);
    if (words.empty()) return "Ready. Type :P D 1 then Enter. No command was sent.";
    if (words.size() == 1 && (words[0] == "help" || words[0] == "?"))
        return "Power: :P D/S/P 0/1 | Motor: :M id E/D/I/S/R value | Servo: :S id P value";
    long id = 0, value = 0;
    // Validate the complete command before touching any hardware.
    if (words[0] == "P") {
        if (words.size() != 3 || !number(words[2], value) || value > 1 ||
            (words[1] != "D" && words[1] != "S" && words[1] != "P"))
            return "ERROR: use :P D 0/1, :P S 0/1 or :P P 0/1";
        std::lock_guard<std::mutex> lock(*device_mutex_);
        if (lifecycle::stopping() || NiFpga_IsError(fpga_->status_) ||
            NiFpga_IsError(fpga_->moduleIO.status_)) return "STOPPING/FAULT: command rejected";
        const int index = words[1] == "D" ? 0 : (words[1] == "S" ? 1 : 2);
        power_->at(index) = value != 0;
    } else if (words[0] == "M") {
        if (words.size() != 4 || !number(words[1], id) || id > 5 ||
            !number(words[3], value)) return "ERROR: use :M id(0..5) E/D/I/S/R value";
        const std::string& op = words[2];
        if (op != "E" && op != "D" && op != "I" && op != "S" && op != "R")
            return "ERROR: unknown motor command";
        if ((op == "I" && value > 4096) || (op != "I" && value > 1))
            return "ERROR: I requires 0..4096; E/D/S/R require 0 or 1";
        std::lock_guard<std::mutex> lock(*device_mutex_);
        if (lifecycle::stopping() || NiFpga_IsError(fpga_->status_) ||
            NiFpga_IsError(fpga_->moduleIO.status_)) return "STOPPING/FAULT: command rejected";
        if (op == "E") fpga_->moduleIO.write_en_(id, value != 0);
        if (op == "D") fpga_->moduleIO.write_dir_(id, value != 0);
        if (op == "I") fpga_->moduleIO.write_iv_(id, value);
        if (op == "S") fpga_->moduleIO.write_state_(id, value != 0);
        if (op == "R") {
            fpga_->moduleIO.write_rp_(id, NiFpga_True);
            fpga_->moduleIO.write_rp_(id, NiFpga_False);
        }
    } else if (words[0] == "S") {
        if (words.size() != 4 || !number(words[1], id) || id > 5 ||
            words[2] != "P" || !number(words[3], value) || value > 65535)
            return "ERROR: use :S id(0..5) P value(0..65535); mode changes are not supported here";
        std::lock_guard<std::mutex> lock(*device_mutex_);
        if (lifecycle::stopping() || NiFpga_IsError(fpga_->status_) ||
            NiFpga_IsError(fpga_->moduleIO.status_)) return "STOPPING/FAULT: command rejected";
        fpga_->moduleIO.write_position_bus_(id, value);
    } else return "ERROR: unknown command. Type help then Enter.";
    return "Accepted: " + input + " (software command; verify hardware feedback)";
}

void Console::run() {
    // All curses calls, including startup and teardown, belong to this thread.
    SCREEN* screen = newterm(nullptr, terminal_out_, terminal_in_);
    if (!screen) {
        fprintf(terminal_out_, "Console could not initialize. Set TERM=xterm and restart.\n");
        fflush(terminal_out_);
        lifecycle::lose_terminal("terminal_init_failed");
        running_ = false;
        return;
    }
    set_term(screen);
    cbreak();
    noecho();
    keypad(stdscr, true);
    wtimeout(stdscr, 100);
    curs_set(1);
    // Treat bracketed paste as editing: its embedded Enter never runs a command.
    define_key("\033[200~", 0x600);
    define_key("\033[201~", 0x601);
    fputs("\033[?2004h", terminal_out_);
    fflush(terminal_out_);
    bool paste = false;
    bool paste_rejected = false;
    std::string command;
    std::string status = "Ready. Type at Command> below; Enter sends. Esc clears. Ctrl+L redraws.";
    unsigned long frame = 0;
    unsigned ready_errors = 0;
    while (running_ && !lifecycle::stopping()) {
        const auto frame_start = std::chrono::steady_clock::now();
        if (lifecycle::endpoint_lost(fileno(terminal_in_), fileno(terminal_out_)) ||
            feof(terminal_in_) || ferror(terminal_in_) || ferror(terminal_out_)) {
            lifecycle::lose_terminal("terminal_EOF_HUP_ERR");
            break;
        }
        int rows = 0, cols = 0;
        getmaxyx(stdscr, rows, cols);
        erase();
        line(0, "R-Slip FPGA Console v2 | local sbRIO readout | frame " + std::to_string(++frame), cols);
        if (rows >= 22 && cols >= 68) {
            std::lock_guard<std::mutex> lock(*device_mutex_);
            char buf[200];
            snprintf(buf, sizeof(buf), "POWER (software): Digital=%d  Signal=%d  Power=%d",
                     static_cast<int>(power_->at(0)), static_cast<int>(power_->at(1)), static_cast<int>(power_->at(2)));
            line(2, buf, cols);
            line(4, "Leg  DC encoder(counts)       TC   HE    Servo encoder(raw)", cols);
            const char* legs[] = {"L1", "L2", "L3", "R1", "R2", "R3"};
            for (int i = 0; i < 6; ++i) {
                const int32_t pos = fpga_->moduleIO.read_ep_(i);
                const uint32_t ticks = fpga_->moduleIO.read_tc_(i);
                const int hall = fpga_->moduleIO.read_he_(i);
                const unsigned servo = fpga_->moduleIO.read_position_encoder_(i);
                snprintf(buf, sizeof(buf), "%s M%d %12d %10u    %d    S%d %8u", legs[i], i, pos, ticks, hall, i, servo);
                line(5 + i, buf, cols);
            }
            snprintf(buf, sizeof(buf), "Servo CM=%u | UI reads are live; a fixed value is not a link check.", fpga_->moduleIO.read_cm_());
            line(11, buf, cols);
            line(12, "POWER CHANNELS: voltage / current (channel wiring must be verified)", cols);
            for (int i = 0; i < 4; ++i) {
                snprintf(buf, sizeof(buf), "CH%d %8.3f V %8.3f A   CH%d %8.3f V %8.3f A", i,
                         fpga_->powerboard_V_list_[i], fpga_->powerboard_I_list_[i], i + 4,
                         fpga_->powerboard_V_list_[i+4], fpga_->powerboard_I_list_[i+4]);
                line(13 + i, buf, cols);
            }
        } else {
            line(2, "Window too small: enlarge to at least 68 columns x 22 rows.", cols);
            line(3, "Input remains available below. Resizing does not send commands.", cols);
        }
        line(rows - 4, "Power: :P D 0/1  :P S 0/1  :P P 0/1 | help | Ctrl+C stops driver", cols);
        line(rows - 3, "Log: /tmp/fpga_driver_console.log | R resets encoder; it does not read.", cols);
        line(rows - 2, status, cols);
        const int available = cols > 12 ? cols - 11 : 0;
        const std::string visible = available > 0 && command.size() > static_cast<size_t>(available)
            ? command.substr(command.size() - available) : command;
        attron(A_REVERSE);
        line(rows - 1, "Command> " + visible, cols);
        attroff(A_REVERSE);
        if (rows > 0 && cols > 1) move(rows - 1, std::min(cols - 2, 9 + static_cast<int>(visible.size())));
        if (refresh() == ERR || ferror(terminal_out_)) {
            lifecycle::lose_terminal("terminal_output_ERR");
            break;
        }
        const int key = getch();
        if (key == ERR) {
            pollfd input{fileno(terminal_in_), POLLIN, 0};
            const int rc = poll(&input, 1, 0);
            if (lifecycle::endpoint_lost(fileno(terminal_in_), fileno(terminal_out_)) ||
                feof(terminal_in_) || ferror(terminal_in_) ||
                (rc > 0 && (input.revents & POLLIN) && ++ready_errors >= 3)) {
                lifecycle::lose_terminal("terminal_read_EOF_ERR");
                break;
            }
            if (rc == 0) ready_errors = 0;
            // ERR can return immediately. Bound CPU even on a broken curses/TTY.
            std::this_thread::sleep_until(frame_start + std::chrono::milliseconds(100));
            continue;
        }
        ready_errors = 0;
        if (key == 4) { lifecycle::lose_terminal("terminal_EOF_CtrlD"); break; }
        if (lifecycle::stopping()) break;
        if (key == KEY_RESIZE) { clearok(stdscr, true); continue; }
        if (key == 0x600) { paste = true; paste_rejected = false; continue; }
        if (key == 0x601) { paste = false; continue; }
        if (key == 12) { clearok(stdscr, true); continue; }
        if (key == 27) { command.clear(); paste = false; status = "Input cleared. No command sent."; continue; }
        if (key == KEY_BACKSPACE || key == 127 || key == 8) {
            if (!command.empty()) command.pop_back();
            continue;
        }
        if (key == '\n' || key == '\r' || key == KEY_ENTER) {
            if (paste) {
                command.clear();
                paste_rejected = true;
                status = "ERROR: paste one command only, then press Enter yourself.";
            } else {
                try { status = execute(command); }
                catch (const std::exception& error) { status = std::string("ERROR: ") + error.what(); }
                command.clear();
            }
            continue;
        }
        if (key >= 32 && key <= 126 && command.size() < 120 && !(paste && paste_rejected))
            command.push_back(static_cast<char>(key));
    }
    fputs("\033[?2004l", terminal_out_);
    fflush(terminal_out_);
    endwin();
    delscreen(screen);
}
