#include <array>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

constexpr int COLS = 16;
constexpr int ROWS = 2;

struct Adafruit_LiquidCrystal {
  Adafruit_LiquidCrystal(int) {}

  std::array<std::array<char, COLS>, ROWS> data;

  void clear();
  void begin(int cols, int rows);
  void render(void);
  void setCursor(int y, int x) {
    this->cy = y;
    this->cx = x;
  }

  void print(int i);
  void print(std::string string);

  int cx, cy;
};

struct Board {
  std::array<bool, 14> pins = {0};

  void render(void);
} g_board;

auto const g_program_start_time = std::chrono::high_resolution_clock::now();

unsigned long millis() {
  using namespace std::chrono;

  auto const now = high_resolution_clock::now();
  auto const diff = now - g_program_start_time;
  auto const duration = duration_cast<milliseconds>(diff);
  return duration.count();
}

void Board::render(void) {
  std::cout << "Board\n";
  std::cout << "Pins:";
  for (int i = 0; i < this->pins.size(); i++) {
    std::cout << " ";
    if (i < 10)
      std::cout << " ";
    std::cout << i;
  }
  std::cout << '\n';
  for (auto &v : this->pins) {
    if (v)
      std::cout << "##";
    else
      std::cout << "  ";
    std::cout << " ";
  }
  std::cout << '\n';
}

bool digitalRead(int pin) { return g_board.pins[pin]; }
void digitalWrite(int pin, int v) { g_board.pins[pin] = v; }
void pinMode(int pin, int mode) {};

void delay(int ms) { std::this_thread::sleep_for(1ms * ms); }

#define HIGH 1
#define LOW 1

#define INPUT 0
#define OUTPUT 0

struct DistanceSensor {
  DistanceSensor(int trig, int echo) {}
  double getCM() { return this->distance_value_cm; }

  void render(void);

  double distance_value_cm = 0;
};

struct Sserial {
  void begin(int baud) {}

  std::string log;

  template <typename T> void print(T v) {
    std::stringstream ss;
    ss << v;
    this->log += ss.str();
  }
  template <typename T> void println(T v) {
    std::stringstream ss;
    ss << v;
    this->log += ss.str() + '\n';
  }

  void render() {
    if (log.size() > 1024)
      log = log.substr(log.size() - 1024);

    std::cout << "Serial output:\n" << log;
  }
} Serial;

// End simulator stuff

#include <Adafruit_LiquidCrystal.h>
#include <DistanceSensor.h>

using ulong = unsigned long;

constexpr std::array<int, 4> BUTTONS = {2, 3, 4, 5};
constexpr int CONFIRM_BUTTON = 6;
constexpr int BUZZER = 7;
constexpr int ALARM = 8;

constexpr int ECHO_PIN = 12;
constexpr int TRIG_PIN = 13;

// constexpr ulong DURATION_BEFORE_ARMED = 10 * 1000;
// constexpr ulong DURATION_BEFORE_ALARM = 10 * 1000;
constexpr ulong DURATION_BEFORE_ARMED = 5 * 1000;
constexpr ulong DURATION_BEFORE_ALARM = 5 * 1000;

constexpr ulong BLINK_INTERVAL = 0.5 * 1000;

Adafruit_LiquidCrystal lcd(0);
DistanceSensor distance_sensor(TRIG_PIN, ECHO_PIN);

// SIMULATOR
void composite_render(void) {
  std::cout << "\033[2J\033[1;1H";
  lcd.render();
  distance_sensor.render();
  g_board.render();
  Serial.render();
  std::cout.flush();
}

struct State {
  enum class Kind {
    Unarmed,
    PINEntry,
    WaitingToLeave,
    Armed,
    Alarm,
  };

  Kind kind = Kind::Unarmed;

  double wall_distance = 250;

  ulong set_timestamp;
  bool has_detected_something = false;

  std::array<int, 4> entered_pin{0};
  int pin_len = 0;
  std::array<int, 4> set_pin{0};
} g_state;

void render_screen(void);
bool is_person_detected(void);

void setup(void) {
  pinMode(BUZZER, OUTPUT);
  pinMode(ALARM, OUTPUT);
  pinMode(CONFIRM_BUTTON, INPUT);
  for (auto btn : BUTTONS)
    pinMode(btn, INPUT);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);

  Serial.begin(9600);

  render_screen();
}

bool handle_pin_buttons(std::array<bool, 4> &prevs,
                        std::array<bool, 4> &states) {
  for (int i = 0; i < BUTTONS.size(); i++) {
    if (!prevs[i] && states[i]) {
      g_state.entered_pin[g_state.pin_len++] = i;
      return true;
    }
  }
  return false;
}

bool handle_unarming_state(void) {
  if (g_state.pin_len == g_state.set_pin.size()) {
    g_state.pin_len = 0;
    if (g_state.set_pin == g_state.entered_pin) {
      g_state.kind = State::Kind::Unarmed;
      digitalWrite(ALARM, 0);
      return true;
    }
  }
  return false;
}

void loop(void) {
  bool update_screen = false;

  static bool prev_confirm_state = 0;
  static std::array<bool, 4> prevs = {0};

  bool confirm_state = digitalRead(CONFIRM_BUTTON);
  static std::array<bool, 4> states = {};
  for (int i = 0; i < BUTTONS.size(); i++)
    states[i] = digitalRead(BUTTONS[i]);

  if (!prev_confirm_state && confirm_state) {
    if (g_state.kind == State::Kind::Unarmed) {
      g_state.kind = State::Kind::PINEntry;
      update_screen = true;
    } else if (g_state.kind == State::Kind::PINEntry) {
      g_state.kind = State::Kind::Unarmed;
      g_state.pin_len = 0;
      update_screen = true;
    }
  }

  if (g_state.kind == State::Kind::Unarmed) {
    if (!prevs[0] && states[0]) {
      g_state.wall_distance = distance_sensor.getCM();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("New wall dist:\n");
      lcd.print(static_cast<int>(g_state.wall_distance));
      lcd.print(".");
      lcd.print(static_cast<int>(
          (g_state.wall_distance -
           static_cast<double>(static_cast<int>(g_state.wall_distance))) *
          100));
      lcd.print(" cm");
      // SIMULATOR
      composite_render();
      delay(2000);
      update_screen = true;
    }
  } else if (g_state.kind == State::Kind::PINEntry) {
    bool res = handle_pin_buttons(prevs, states);
    if (res)
      update_screen = true;

    if (g_state.pin_len == g_state.entered_pin.size()) {
      g_state.kind = State::Kind::WaitingToLeave;
      g_state.set_pin = g_state.entered_pin;
      g_state.pin_len = 0;
      g_state.set_timestamp = millis();
      update_screen = true;
    }
  } else if (g_state.kind == State::Kind::WaitingToLeave) {
    auto now = millis();
    auto diff = now - g_state.set_timestamp;

    static bool buzzer_state = false;
    static ulong prev_millis = 0;
    if (now - prev_millis >= BLINK_INTERVAL) {
      prev_millis = now;
      buzzer_state = !buzzer_state;
      digitalWrite(BUZZER, buzzer_state);
    }

    if (diff > DURATION_BEFORE_ARMED) {
      g_state.set_timestamp = millis();
      g_state.kind = State::Kind::Armed;
      g_state.has_detected_something = false;
      update_screen = true;
    }
  } else if (g_state.kind == State::Kind::Armed) {
    bool res = handle_pin_buttons(prevs, states);
    if (res)
      update_screen = true;
    res = handle_unarming_state();
    if (res)
      update_screen = true;

    if (g_state.has_detected_something) {
      auto const now = millis();
      auto const diff = now - g_state.set_timestamp;
      if (diff > DURATION_BEFORE_ALARM) {
        g_state.kind = State::Kind::Alarm;
        digitalWrite(ALARM, 1);
        update_screen = true;
      }
    } else {
      bool one = is_person_detected();
      delay(50);
      bool two = is_person_detected();

      if (one && two) {
        g_state.has_detected_something = true;
        g_state.set_timestamp = millis();
        update_screen = true;
      }
    }
  } else if (g_state.kind == State::Kind::Alarm) {
    bool res = handle_pin_buttons(prevs, states);
    if (res)
      update_screen = true;
    res = handle_unarming_state();
    if (res)
      update_screen = true;
  }

  if (update_screen)
    render_screen();

  prevs = states;
  prev_confirm_state = confirm_state;
}

void render_pin_entry(void) {
  for (int i = 0; i < g_state.entered_pin.size(); i++) {
    if (i < g_state.pin_len)
      lcd.print("*");
    else
      lcd.print("_");
  }
}

void render_screen(void) {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (g_state.kind == State::Kind::Unarmed) {
    lcd.print("System Unarmed\n");
    lcd.print("Press for arming");
  } else if (g_state.kind == State::Kind::PINEntry) {
    lcd.print("Enter PIN: ");
    render_pin_entry();
    lcd.setCursor(1, 0);
    lcd.print("Or abort arming.");
  } else if (g_state.kind == State::Kind::WaitingToLeave) {
    lcd.print("Waiting to leave");
  } else if (g_state.kind == State::Kind::Armed) {
    if (g_state.has_detected_something) {
      lcd.print("Detected.");
    } else {
      lcd.print("Armed.");
    }
    lcd.setCursor(1, 0);
    lcd.print("PIN: ");
    render_pin_entry();
  } else if (g_state.kind == State::Kind::Alarm) {
    lcd.print("Alarm active.");
    lcd.setCursor(1, 0);
    lcd.print("PIN: ");
    render_pin_entry();
  }
}

bool is_person_detected(void) {
  auto cm = distance_sensor.getCM();
  if (cm < 20)
    return false;
  if (cm > g_state.wall_distance - 20)
    return false;
  return true;
}

// Simulator stuff

void setup_terminal() {
  termios terminal_settings;
  tcgetattr(STDIN_FILENO, &terminal_settings);
  terminal_settings.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &terminal_settings);
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void reset_terminal() {
  termios terminal_settings;
  tcgetattr(STDIN_FILENO, &terminal_settings);
  terminal_settings.c_lflag |= (ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &terminal_settings);
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
}

int getch() {
  char ch;
  int bytesRead = read(STDIN_FILENO, &ch, 1);
  if (bytesRead == 1) {
    return ch;
  } else {
    return 0;
  }
}

constexpr std::chrono::duration SLEEP_AMOUNT = 5ms;

static auto volatile g_running = true;
void int_handler(int) { g_running = false; }

int main(void) {
  setup_terminal();
  setup();
  composite_render();
  std::this_thread::sleep_for(SLEEP_AMOUNT);

  signal(SIGINT, int_handler);

  std::array<bool, 14> to_reset = {0};

  while (g_running) {
    for (int i = 0; i < to_reset.size(); i++) {
      if (!to_reset[i])
        continue;
      digitalWrite(i, 0);
    }

    int ch;
    while ((ch = getch())) {
      switch (ch) {
        { // Simulation-specific
        case 'q':
          g_running = false;
          break;
        }

        { // Distance sensor
        case '+':
          distance_sensor.distance_value_cm += 5;
          break;
        case '-':
          distance_sensor.distance_value_cm -= 5;
          break;
        }

        { // PIN Buttons
        case '1':
          digitalWrite(BUTTONS[0], 1);
          to_reset[BUTTONS[0]] = 1;
          break;
        case '2':
          digitalWrite(BUTTONS[1], 1);
          to_reset[BUTTONS[1]] = 1;
          break;
        case '3':
          digitalWrite(BUTTONS[2], 1);
          to_reset[BUTTONS[2]] = 1;
          break;
        case '4':
          digitalWrite(BUTTONS[3], 1);
          to_reset[BUTTONS[3]] = 1;
          break;
        }

        { // Confirmation button
        case '\n':
          digitalWrite(CONFIRM_BUTTON, 1);
          to_reset[CONFIRM_BUTTON] = 1;
          break;
        }
      }
    }

    loop();
    composite_render();
    std::this_thread::sleep_for(SLEEP_AMOUNT);
  }

  reset_terminal();

  std::cout << "Goodbye!\n";
}

void Adafruit_LiquidCrystal::clear() {
  for (auto &row : this->data) {
    for (auto &c : row)
      c = 0;
  }
  this->cx = this->cy = 0;
}

void Adafruit_LiquidCrystal::begin(int cols, int rows) { this->clear(); }

void Adafruit_LiquidCrystal::print(int v) {
  char buffer[COLS + 1];
  int dataw = snprintf(buffer, sizeof(buffer), "%d", v);

  for (int i = 0; i < dataw && cy < ROWS; ++i) {
    if (cx >= COLS) {
      cx = 0;
      ++cy;
      if (cy >= ROWS) {
        cy = 0;
        break;
      }
    }
    this->data[cy][cx++] = buffer[i];
  }
}

void Adafruit_LiquidCrystal::print(std::string v) {
  for (auto const &c : v) {
    if (cy >= ROWS) {
      cy = 0;
      cx = 0;
      break;
    }
    if (c == '\n') {
      cx = 0;
      cy += 1;
      continue;
    }
    this->data[cy][cx] = c;
    cx += 1;
    if (cx >= COLS) {
      cx = 0;
      cy += 1;
    }
  }
}

void Adafruit_LiquidCrystal::render(void) {
  constexpr std::string_view BLUE_BG = "\033[44m";
  constexpr std::string_view WHITE_FG = "\033[97m";
  constexpr std::string_view RST = "\033[0m";

  std::cout << "LCD" << '\n';
  std::cout << ',';
  for (int i = 0; i < COLS; i++)
    std::cout << '-';
  std::cout << ",\n";

  for (auto const &row : this->data) {
    std::cout << '|';
    for (auto const &c : row) {
      if (c == 0)
        std::cout << BLUE_BG << WHITE_FG << ' ' << RST;
      else
        std::cout << BLUE_BG << WHITE_FG << c << RST;
    }
    std::cout << "|\n";
  }

  std::cout << '\'';
  for (int i = 0; i < COLS; i++)
    std::cout << '-';
  std::cout << '\'' << '\n';
}

void DistanceSensor::render(void) {
  std::cout << "Distance Sensor: " << this->distance_value_cm << " cm\n";
}
