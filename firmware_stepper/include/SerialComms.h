#ifndef SerialComms_h
#define SerialComms_h

#include <Arduino.h>

class SerialComms {
  public:
    enum CommandType {
      COMMAND_NONE,
      COMMAND_MOVE_RELATIVE,
      COMMAND_MOVE_ABSOLUTE,
      COMMAND_SET_SPEED,
      COMMAND_STOP,
      COMMAND_ENABLE,
      COMMAND_DISABLE,
      COMMAND_QUERY_POSITION,
      COMMAND_INVALID
    };

    static const size_t BufferSize = 128;

    struct Command {
      CommandType type = COMMAND_NONE;
      float xMm = 0.0f;
      float yMm = 0.0f;
      float zMm = 0.0f;
      unsigned int speedX = 0;
      unsigned int speedY = 0;
      unsigned int speedZ = 0;
      char raw[BufferSize] = { '\0' };
    };

    explicit SerialComms(HardwareSerial& serial = Serial);

    // Initialize the serial port at the requested baud rate.
    void begin(unsigned long baud = 115200);

    // Replace the active Serial port object at runtime.
    void setSerialPort(HardwareSerial& serial);

    // Store a port name string for the current connection (e.g. "COM3" or "/dev/ttyUSB0").
    void setPortName(const char* portName);
    const char* getPortName() const;

    // Read one complete command line from Serial and parse it.
    // Returns true when a new line has been received, even if the command is invalid.
    bool readCommand(Command& command);

    // Send a formatted position response back to the host.
    void sendPositionFeedback(float xMm, float yMm, float zMm);

    // Send a short text status message back to the host.
    void sendStatus(const char* message);

    // Send a short error message back to the host.
    void sendError(const char* message);

  private:
    HardwareSerial* _serial;
    char _buffer[BufferSize];
    size_t _index;
    char _portName[32];

    bool parseLine(const char* line, Command& command);
    static CommandType parseCommandType(const char* token);
    static bool parseAxisValues(char* token, float& xMm, float& yMm, float& zMm);
    static bool parseSpeedValues(char* token, unsigned int& speedX, unsigned int& speedY, unsigned int& speedZ);
};

#endif
