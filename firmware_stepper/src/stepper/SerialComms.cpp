#include "SerialComms.h"

SerialComms::SerialComms(HardwareSerial& serial)
  : _serial(&serial), _index(0)
{
  _portName[0] = '\0';
}

void SerialComms::begin(unsigned long baud) {
  if (_serial != nullptr) {
    _serial->begin(baud);
  }
  _index = 0;
}

void SerialComms::setSerialPort(HardwareSerial& serial) {
  _serial = &serial;
}

void SerialComms::setPortName(const char* portName) {
  if (portName == nullptr) {
    _portName[0] = '\0';
    return;
  }
  strncpy(_portName, portName, sizeof(_portName));
  _portName[sizeof(_portName) - 1] = '\0';
}

const char* SerialComms::getPortName() const {
  return _portName;
}

bool SerialComms::readCommand(Command& command) {
  if (_serial == nullptr) {
    command.type = COMMAND_INVALID;
    return false;
  }

  while (_serial->available() > 0) {
    int incoming = _serial->read();
    if (incoming < 0) {
      continue;
    }

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      if (_index == 0) {
        continue; // ignore blank lines or stray newlines
      }
      _buffer[_index] = '\0';
      bool parseOk = parseLine(_buffer, command);
      _index = 0;
      if (!parseOk) {
        command.type = COMMAND_INVALID;
      }
      return true;
    }

    if (_index + 1 < BufferSize) {
      _buffer[_index++] = static_cast<char>(incoming);
    } else {
      _index = 0;
    }
  }

  return false;
}

void SerialComms::sendPositionFeedback(float xMm, float yMm, float zMm) {
  _serial->print("<POSITION,");
  _serial->print(xMm, 3);
  _serial->print(",");
  _serial->print(yMm, 3);
  _serial->print(",");
  _serial->print(zMm, 3);
  _serial->println(">");
}

void SerialComms::sendStatus(const char* message) {
  _serial->print("<STATUS,");
  _serial->print(message);
  _serial->println(">");
}

void SerialComms::sendError(const char* message) {
  _serial->print("<ERROR,");
  _serial->print(message);
  _serial->println(">");
}

bool SerialComms::parseLine(const char* line, Command& command) {
  if (line == nullptr || *line == '\0') {
    command.type = COMMAND_INVALID;
    return false;
  }

  char buffer[BufferSize];
  strncpy(buffer, line, BufferSize);
  buffer[BufferSize - 1] = '\0';

  char* begin = buffer;
  while (*begin != '\0' && (!isprint(static_cast<unsigned char>(*begin)) || isspace(static_cast<unsigned char>(*begin)))) {
    ++begin;
  }

  char* end = begin + strlen(begin);
  while (end > begin && (!isprint(static_cast<unsigned char>(*(end - 1))) || isspace(static_cast<unsigned char>(*(end - 1))))) {
    *--end = '\0';
  }

  memset(command.raw, 0, sizeof(command.raw));
  strncpy(command.raw, begin, BufferSize - 1);
  command.raw[BufferSize - 1] = '\0';

  if (*begin == '\0') {
    command.type = COMMAND_INVALID;
    return false;
  }

  char* token = strtok(begin, " \t,");
  if (token == nullptr) {
    command.type = COMMAND_INVALID;
    return false;
  }

  command.type = parseCommandType(token);
  command.xMm = 0.0f;
  command.yMm = 0.0f;
  command.zMm = 0.0f;
  command.speedX = 0;
  command.speedY = 0;
  command.speedZ = 0;

  bool parseOk = true;

  switch (command.type) {
    case COMMAND_MOVE_RELATIVE:
    case COMMAND_MOVE_ABSOLUTE: {
      parseOk = parseAxisValues(strtok(nullptr, " \t,"), command.xMm, command.yMm, command.zMm);
      break;
    }
    case COMMAND_SET_SPEED: {
      parseOk = parseSpeedValues(strtok(nullptr, " \t,"), command.speedX, command.speedY, command.speedZ);
      break;
    }
    case COMMAND_STOP:
    case COMMAND_ENABLE:
    case COMMAND_DISABLE:
    case COMMAND_QUERY_POSITION:
      parseOk = true;
      break;
    default:
      parseOk = false;
      break;
  }

  if (!parseOk) {
    command.type = COMMAND_INVALID;
  }

  return parseOk;
}

SerialComms::CommandType SerialComms::parseCommandType(const char* token) {
  if (token == nullptr) {
    return COMMAND_INVALID;
  }

  char normalized[32];
  size_t length = strlen(token);
  if (length >= sizeof(normalized)) {
    length = sizeof(normalized) - 1;
  }

  for (size_t i = 0; i < length; ++i) {
    normalized[i] = toupper(static_cast<unsigned char>(token[i]));
  }
  normalized[length] = '\0';

  if (strcmp(normalized, "MOVE_REL") == 0 || strcmp(normalized, "REL_MOVE") == 0 || strcmp(normalized, "MOVE_RELATIVE") == 0) {
    return COMMAND_MOVE_RELATIVE;
  }
  if (strcmp(normalized, "MOVE_ABS") == 0 || strcmp(normalized, "ABS_MOVE") == 0 || strcmp(normalized, "MOVE_ABSOLUTE") == 0) {
    return COMMAND_MOVE_ABSOLUTE;
  }
  if (strcmp(normalized, "SET_SPEED") == 0 || strcmp(normalized, "SPEED") == 0) {
    return COMMAND_SET_SPEED;
  }
  if (strcmp(normalized, "STOP") == 0 || strcmp(normalized, "HALT") == 0 || strcmp(normalized, "ABORT") == 0) {
    return COMMAND_STOP;
  }
  if (strcmp(normalized, "ENABLE") == 0) {
    return COMMAND_ENABLE;
  }
  if (strcmp(normalized, "DISABLE") == 0) {
    return COMMAND_DISABLE;
  }
  if (strcmp(normalized, "QUERY_POS") == 0 || strcmp(normalized, "GET_POS") == 0 || strcmp(normalized, "POSITION?") == 0 || strcmp(normalized, "REPORT_POS") == 0) {
    return COMMAND_QUERY_POSITION;
  }

  return COMMAND_INVALID;
}

bool SerialComms::parseAxisValues(char* token, float& xMm, float& yMm, float& zMm) {
  if (token == nullptr) {
    return false;
  }

  xMm = atof(token);
  token = strtok(nullptr, " \t,");
  if (token == nullptr) {
    return false;
  }
  yMm = atof(token);

  token = strtok(nullptr, " \t,");
  if (token == nullptr) {
    return false;
  }
  zMm = atof(token);

  return true;
}

bool SerialComms::parseSpeedValues(char* token, unsigned int& speedX, unsigned int& speedY, unsigned int& speedZ) {
  if (token == nullptr) {
    return false;
  }

  speedX = static_cast<unsigned int>(strtoul(token, nullptr, 10));
  token = strtok(nullptr, " \t,");
  if (token == nullptr) {
    return false;
  }
  speedY = static_cast<unsigned int>(strtoul(token, nullptr, 10));

  token = strtok(nullptr, " \t,");
  if (token == nullptr) {
    return false;
  }
  speedZ = static_cast<unsigned int>(strtoul(token, nullptr, 10));

  return true;
}
