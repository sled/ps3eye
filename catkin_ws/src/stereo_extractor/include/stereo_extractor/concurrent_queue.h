/*
 * concurrent_queue.h
 *
 *  Created on: Apr 1, 2013
 *      Author: sled
 */

#ifndef CONCURRENT_QUEUE_H_
#define CONCURRENT_QUEUE_H_

#include <queue>
#include <boost/thread.hpp>

namespace stereo_extractor {

// thread safe multi-producer, multi-consumer queue
template<typename T> class ConcurrentQueue {

public:

  // try to pop a value if possible, two consequent calls to empty() and pop() won't guarantee that the data hasn't been
  // popped somewhere inbetween
  bool try_pop(T& popped_element) {
    // acquire lock
    boost::mutex::scoped_lock lock(queue_mutex_);

    if(queue_.empty()) {
      // couldn't pop an element
      return false;
    }

    popped_element = queue_.front();
    queue_.pop();
    return true;
  }

  // important: return by reference to avoid copy constructor exceptions, otherwise the element will be lost!
  void wait_and_pop(T& popped_element) {
    // acquire lock
    boost::mutex::scoped_lock lock(queue_mutex_);

    while(queue_.empty()) {
      // put thread to sleep, unlock lock, wait for signal and reiterate
      data_available_condition_.wait(lock);
    }

    // queue is not empty anymore, pop value
    popped_element = queue_.front();
    queue_.pop();
  }

  T wait_and_pop() {
    // acquire lock
    boost::mutex::scoped_lock lock(queue_mutex_);

    while(queue_.empty()) {
      // put thread to sleep, unlock lock, wait for signal and reiterate
      data_available_condition_.wait(lock);
    }

    // queue is not empty anymore, pop value
    T popped_element = queue_.front();
    queue_.pop();

    return popped_element;
  }

  bool empty() const {
    boost::mutex::scoped_lock lock(queue_mutex_);
    return queue_.empty();
  }

  void push(T const& element) {
    // kill lock outside of block
    boost::mutex::scoped_lock lock(queue_mutex_);
    queue_.push(element);
    lock.unlock();
    // notify a thread that data is available
    data_available_condition_.notify_one();
  }

  typename std::queue<T>::size_type size() const {
    boost::mutex::scoped_lock lock(queue_mutex_);
    return queue_.size();
  }

private:
  mutable boost::mutex queue_mutex_;
  boost::condition_variable data_available_condition_;
  std::queue<T> queue_;
};
}

#endif /* CONCURRENT_QUEUE_H_ */
