#pragma once

#include <deque>
#include <functional>

namespace liokit::utils {

template <class D_PTR>
void QueuePopOld(std::deque<D_PTR>& queue, double timepoint) {
  while (!queue.empty()) {
    if (queue.front()->timestamp < timepoint) {
      queue.pop_front();
    } else {
      break;
    }
  }
}

template <class D_PTR>
void QueuePopOldWithHandle(std::deque<D_PTR>& queue, double timepoint, std::function<void(const D_PTR&)> handle) {
  while (!queue.empty()) {
    if (queue.front()->timestamp < timepoint) {
      handle(queue.front());
      queue.pop_front();
    } else {
      break;
    }
  }
}

}