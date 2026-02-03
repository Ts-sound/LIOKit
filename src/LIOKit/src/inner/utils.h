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

template <typename Container, typename Handler>
void QueuePopOldWithHandle(Container& queue, double timepoint, Handler&& handler) {
  while (!queue.empty()) {
    if (queue.front()->timestamp < timepoint) {
      handler(queue.front());
      queue.pop_front();
    } else {
      break;
    }
  }
}

}