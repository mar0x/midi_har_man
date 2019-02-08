
#pragma once

struct midi_cmd {
  enum {
      CMD_BIT         = 0x80U,
      CMD_NOTE_OFF    = 0x80U,
      CMD_NOTE_ON     = 0x90U,
      CMD_CTRL_CHANGE = 0xB0U,
      CMD_PROG_CHANGE = 0xC0U,
      CMD_PITCH_CHANGE = 0xE0U,
  };

  enum {
      CTRL_ALL_SOUND_OFF  = 120,
      CTRL_RESET_ALL      = 121,
      CTRL_LOCAL_CONTROL  = 122,
  };

  enum {
      MAX_SIZE = 3
  };

  uint8_t cmd_[MAX_SIZE];
  uint8_t size_;

  midi_cmd() = default;
  midi_cmd(uint8_t ch, uint8_t cmd, uint8_t val) :
      size_(2)
  {
      cmd_[0] = cmd | (ch & 0x0FU);
      cmd_[1] = val & 0x7FU;
  }

  midi_cmd(uint8_t ch, uint8_t cmd, uint8_t ctl, uint8_t val) :
      size_(3)
  {
      cmd_[0] = cmd | (ch & 0x0FU);
      cmd_[1] = ctl & 0x7FU;
      cmd_[2] = val & 0x7FU;
  }

  uint8_t command() const { return cmd_[0] & 0xF0U; }
  uint8_t channel() const { return cmd_[0] & 0x0FU; }
  uint8_t controller() const { return cmd_[1]; }
  uint8_t value() const { return cmd_[2]; }
  uint8_t program() const { return cmd_[1]; }
  uint16_t pitch() const { return cmd_[1] | (cmd_[2] << 7); }

  operator bool() const {
      return
          (command() == CMD_PROG_CHANGE && size_ == 2) ||
          (command() == CMD_CTRL_CHANGE && size_ == 3) ||
          (command() == CMD_NOTE_OFF && size_ == 3) ||
          (command() == CMD_NOTE_ON && size_ == 3) ||
          (command() == CMD_PITCH_CHANGE && size_ == 3);
  }

  void reset() { size_ = 0; }

  template<typename STREAM>
  bool read(STREAM& s) {
      uint8_t b;

      while (!*this && s.available()) {
          b = s.read();

          read(b);
      }

      return *this;
  }

  template<typename STREAM>
  void write(STREAM& s) {
#if defined(DEBUG)
      for (uint8_t i = 0; i < size_; ++i) {
          debug(4, "midi write: ", cmd_[i]);
      }
#endif

      s.write(cmd_, size_);
  }

  bool read(uint8_t b) {
      debug(4, "midi read: ", b);

      if ((b & CMD_BIT) == CMD_BIT) {
          cmd_[0] = b;
          size_ = 1;

          return *this;
      }

      if (size_ > 0 && size_ < MAX_SIZE) {
          cmd_[size_++] = b;
      }

      return *this;
  }
};
