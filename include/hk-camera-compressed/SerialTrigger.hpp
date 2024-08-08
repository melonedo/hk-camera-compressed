#include "CameraMaster.hpp"
#include "serial/serial.h"

std::ostream &writeStringEscaped(std::ostream &out, std::string const &s) {
  for (auto ch : s) {
    switch (ch) {
    case '\'':
      out << "\\'";
      break;

    case '\"':
      out << "\\\"";
      break;

    case '\?':
      out << "\\?";
      break;

    case '\\':
      out << "\\\\";
      break;

    case '\a':
      out << "\\a";
      break;

    case '\b':
      out << "\\b";
      break;

    case '\f':
      out << "\\f";
      break;

    case '\n':
      out << "\\n";
      break;

    case '\r':
      out << "\\r";
      break;

    case '\t':
      out << "\\t";
      break;

    case '\v':
      out << "\\v";
      break;

    default:
      out << ch;
    }
  }

  return out;
}

class SerialTrigger {
  serial::Serial port;
  std::string readBuf;
  TriggerManager *trigger_manager;
  enum class State { STOPPED, RESET, RUNNING };
  State state;
  unsigned ack;
  unsigned ack_window;
  const bool verbose = false;

public:
  void init(TriggerManager *tm, unsigned ack_window_) {
    port.setPort("/dev/ttyACM0");
    port.setBaudrate(115200);
    port.setTimeout(100, 100, 0, 0, 0);
    port.open();
    // port.flushInput();
    trigger_manager = tm;
    ack_window = ack_window_;
    state = State::STOPPED;
    ack = 0;
    std::cout << "Serial open" << std::endl;
  }

  void stop() { port.close(); }

  void spin() {
    // port.read(readBuf, 10);
    readBuf.clear();
    size_t s = port.readline(readBuf, 65536, "\n");
    if (s) {
      if (verbose) {
        std::cout << "Data: [" << s << "]: ";
        writeStringEscaped(std::cout, readBuf);
        std::cout << std::endl;
      }

      // std::string::size_type start = 0;
      // std::string::size_type end = readBuf.find('\n');
      std::string line;

      // while (end != std::string::npos) {
      // if (readBuf[end - 1] != '\r')
      //   std::cout << "r: " << int(readBuf[end - 1]) << std::endl;
      // if (readBuf[end - 1] == '\r')
      //   readBuf[end - 1] = 0;

      // line = readBuf.substr(start, end - start - 1);
      // std::cout << "Line: " << start << "-" << end << " :" << line <<
      // std::endl;

      line = readBuf;
      if (!process_line(line)) {
        std::cout << "Ignore serial data: ";
        writeStringEscaped(std::cout, line);
        std::cout << std::endl;
      }

      //   start = end + 1;
      //   end = readBuf.find('\n', start);
      // }
    }

    if (state != State::STOPPED) {
      char buf[256];
      unsigned ret = snprintf(buf, sizeof buf, "pong %u\n", ack + ack_window);
      assert(ret < sizeof buf);
      port.write((uint8_t *)&buf, strnlen(buf, sizeof buf));
      if (verbose) {
        std::cout << "Serial write: ";
        writeStringEscaped(std::cout, buf);
        std::cout << std::endl;
      }
    }

    // readBuf = readBuf.substr(start);
  }

  bool process_line(const std::string &line) {
    if (state != State::RUNNING && line == "ping\n") {
      if (state == State::STOPPED) {
        std::cout << "Trigger reset" << std::endl;
        state = State::RESET;
      }
      return true;
    } else if (state != State::STOPPED) {
      unsigned n;
      unsigned long ts;
      int ret = sscanf(line.c_str(), "frame %u %lu", &n, &ts);
      if (ret != 2)
        return false;
      if (state == State::RESET) {
        if (n != 0)
          return false;
        std::cout << "Trigger running" << std::endl;
        state = State::RUNNING;
      }
      ret = trigger_manager->add_timestamp(n, ts);
      std::cout << "Timestamp: " << n << " " << ts << std::endl;
      assert(ret && "Timestamps must not arrive out of order");
      ack = n + 1;
      return true;
    }
    return false;
  }
};