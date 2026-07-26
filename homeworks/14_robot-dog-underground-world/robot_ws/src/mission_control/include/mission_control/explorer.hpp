#pragma once

#include <array>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

namespace mission_control {

struct Vector2d {
  int x = 0;
  int y = 0;

  bool operator==(const Vector2d other) const { return x == other.x && y == other.y; }
  bool operator!=(const Vector2d other) const { return !(*this == other); }

  bool operator<(const Vector2d other) const { return std::tie(y, x) < std::tie(other.y, other.x); }

  Vector2d operator+(const Vector2d other) const { return {.x = x + other.x, .y = y + other.y}; }
  Vector2d operator-(const Vector2d other) const { return {.x = x - other.x, .y = y - other.y}; }
};

// Типи клітинок карти
enum class Cell {
  Unknown,    // ще не бачили
  Wall,       // # стіна
  Free,       // . прохід
  Start,      // S стартова клітинка
  Contact,    // C - активний контакт
  Processed,  // x - оброблений контакт (тепер прохідний)
};

// MoveCommand.msg
enum class Move {
  Up,
  Down,
  Left,
  Right,
};

// StudentStatus.msg
enum class Phase {
  Exploring,
  Engaging,
  Returning,
  Done,
  Failed,
};

// Одна клітинка з LocalScan яка прийшла в топіку (CellObservation.msg)
struct ScanCell {
  int x = 0;
  int y = 0;
  char cell_type = '\0';
  int contact_id = 0;
};

// LocalScan.msg
struct Scan {
  int robot_x = 0;
  int robot_y = 0;
  std::vector<ScanCell> cells;
};

// Активний контакт
struct Contact {
  int id = 0;
  int x = 0;
  int y = 0;
};

enum class ActionKind {
  Trigger,  // викликати /payload/trigger
  Move,     // публікувати /robot/cmd_move
  Done,     // місію завершено, все побачено та всі контакти оброблені
  Fail,     // застряг - немає наступного кроку
};

struct Action {
  ActionKind kind = ActionKind::Fail;
  Phase phase = Phase::Failed;
  Move move = Move::Up;
  int contact_id = 0;
  int contact_x = 0;
  int contact_y = 0;
};

class Explorer {
public:
  Action step(const Scan& scan)
  {
    const Vector2d robot = {scan.robot_x, scan.robot_y};
    integrate(scan, robot);

    // 1. Видимий активний контакт - спочатку обробити
    const auto hit = first_active_contact(scan);
    if (hit.has_value()) {
      Action action;
      action.kind = ActionKind::Trigger;
      action.phase = Phase::Engaging;
      action.contact_id = hit->id;
      action.contact_x = hit->x;
      action.contact_y = hit->y;
      return action;
    }

    // 2. Крок уперед - перша невідвідана прохідна сусідня клітинка
    for (const Move dir : kMoveOrder) {
      const Vector2d next = robot + delta(dir);
      if (is_known_passable(next) && visited_.find(next) == visited_.end()) {
        return make_move_action(dir, Phase::Exploring);
      }
    }

    // 3. Немає куди вперед  повернення на попередню клітинку шляху
    if (path_.size() >= 2) {
      const Vector2d previous = path_.at(path_.size() - 2);
      // Якщо непройдених досяжних клітинок уже нема - повернення в S
      const Phase phase = has_unvisited_reachable(robot) ? Phase::Exploring : Phase::Returning;
      return make_move_action(dir_between(robot, previous), phase);
    }

    // 4. Робот у стартовій клітинці і йти нікуди
    if (!has_unvisited_reachable(robot)) {
      Action action;
      action.kind = ActionKind::Done;
      action.phase = Phase::Done;
      return action;
    }

    Action action;
    action.kind = ActionKind::Fail;
    action.phase = Phase::Failed;
    return action;
  }

  // Для для логів
  Vector2d robot() const { return path_.empty() ? Vector2d{} : path_.back(); }
  std::size_t visited_count() const { return visited_.size(); }
  bool visited(const Vector2d position) const { return visited_.find(position) != visited_.end(); }

private:
  static constexpr std::array<Move, 4> kMoveOrder = {Move::Up, Move::Right, Move::Down, Move::Left};

  static Cell parse_cell(const char& type)
  {
    switch (type) {
      case '#':
        return Cell::Wall;
      case '.':
        return Cell::Free;
      case 'S':
        return Cell::Start;
      case 'C':
        return Cell::Contact;
      case 'x':
        return Cell::Processed;
      default:
        return Cell::Unknown;
    }
  }

  static bool is_passable(const Cell cell) { return cell == Cell::Free || cell == Cell::Start || cell == Cell::Processed; }

  static Vector2d delta(const Move dir)
  {
    switch (dir) {
      case Move::Up:
        return Vector2d{0, -1};
      case Move::Down:
        return Vector2d{0, 1};
      case Move::Left:
        return Vector2d{-1, 0};
      case Move::Right:
        return Vector2d{1, 0};
      default:
        return Vector2d{0, 0};
    }
  }

  static Move dir_between(const Vector2d from, const Vector2d to)
  {
    const Vector2d delta = to - from;
    if (delta.x == 1) {
      return Move::Right;
    }
    if (delta.x == -1) {
      return Move::Left;
    }
    if (delta.y == 1) {
      return Move::Down;
    }
    return Move::Up;  // delta.y == -1
  }

  static Action make_move_action(const Move dir, const Phase phase)
  {
    Action action;
    action.kind = ActionKind::Move;
    action.phase = phase;
    action.move = dir;
    return action;
  }

  // Обробити скан та зберегти у пам'яті
  void integrate(const Scan& scan, const Vector2d robot)
  {
    // Збереження клітинки та перезапис "С" на "x" якщо потрібно
    for (const auto& cell : scan.cells) {
      map_[Vector2d{cell.x, cell.y}] = parse_cell(cell.cell_type);
    }

    if (!started_) {
      path_.push_back(robot);
      visited_.insert(robot);
      started_ = true;
      return;
    }

    if (robot == path_.back()) {
      // Позиція не змінилась - шлях без змін.
    }
    else if (path_.size() >= 2 && robot == path_.at(path_.size() - 2)) {
      path_.pop_back();  // повернулись на попередню клітинку
    }
    else {
      path_.push_back(robot);  // ступили на нову клітинку
    }

    visited_.insert(robot);
  }

  bool is_known_passable(const Vector2d position) const
  {
    const auto it = map_.find(position);
    return it != map_.end() && is_passable(it->second);
  }

  // Знаходимо найменший за id активний контакт у поточному скані
  static std::optional<Contact> first_active_contact(const Scan& scan)
  {
    std::optional<Contact> best;
    for (const auto& cell : scan.cells) {
      if (parse_cell(cell.cell_type) != Cell::Contact) {
        continue;
      }
      if (!best.has_value() || cell.contact_id < best->id) {
        best = Contact{cell.contact_id, cell.x, cell.y};
      }
    }
    return best;
  }

  // Чи лишились ще досяжні непройдені клітинки у відомій карті, з врахуванням активного контакту
  bool has_unvisited_reachable(const Vector2d from) const
  {
    const auto is_traversable = [this](const Vector2d position) {
      const auto it = map_.find(position);
      return it != map_.end() && (is_passable(it->second) || it->second == Cell::Contact);
    };

    std::set<Vector2d> seen;
    std::queue<Vector2d> frontier;
    frontier.push(from);
    seen.insert(from);

    while (!frontier.empty()) {
      const Vector2d current = frontier.front();
      frontier.pop();

      if (is_traversable(current) && visited_.find(current) == visited_.end()) {
        return true;
      }

      for (const Move dir : kMoveOrder) {
        const Vector2d next = current + delta(dir);
        if (seen.find(next) == seen.end() && is_traversable(next)) {
          seen.insert(next);
          frontier.push(next);
        }
      }
    }

    return false;
  }

  std::map<Vector2d, Cell> map_;
  std::set<Vector2d> visited_;
  std::vector<Vector2d> path_;
  bool started_ = false;
};

}  // namespace mission_control
