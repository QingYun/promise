#pragma once

#define PROMISE_DISALLOW_COPY(class_name) \
  class_name(const class_name&) = delete; \
  class_name& operator=(const class_name&) = delete

#define PROMISE_DISALLOW_MOVE(class_name) \
  class_name(class_name&&) = delete; \
  class_name& operator=(class_name&&) = delete