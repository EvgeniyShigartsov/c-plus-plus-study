#pragma once
#include <queue>
#include <mutex>
#include <optional>

template <typename T>
class ThreadSafeQueue {
public:
  void push(const T& item)
  {
    std::lock_guard<std::mutex> lock(mutex);
    queue.push(item);
  }

  std::optional<T> tryPop()
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (queue.empty()) {
      return std::nullopt;
    }
    T item = queue.front();
    queue.pop();
    return item;
  }

  bool empty()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return queue.empty();
  }

private:
  std::queue<T> queue;
  std::mutex mutex;
};