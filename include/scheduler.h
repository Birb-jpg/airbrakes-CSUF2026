#pragma once
#include <cstdint>

using TaskCallback = void (*)();

struct Task {
    TaskCallback callback;  // The function to execute
    uint32_t period_us;     // How often to run
    uint32_t last_us;       // Last execution timestamp
    uint32_t dt_us;         // Actual delta time of last execution
};

template <uint8_t MAX_TASKS>
class Scheduler {
public:
    Scheduler() : task_count_(0) {}

    bool addTask(TaskCallback callback, uint32_t period_us, uint32_t now_us = 0) {
        if (task_count_ >= MAX_TASKS) return false;

        tasks_[task_count_] = {
            .callback = callback,
            .period_us = period_us,
            .last_us = now_us,
            .dt_us = period_us // Default dt to period on startup
        };
        task_count_++;
        return true;
    }

    void tick(uint32_t now_us) {
        for (uint8_t i = 0; i < task_count_; ++i) {
            Task& t = tasks_[i];
            
            // Underflow-safe duration check
            if (now_us - t.last_us >= t.period_us) {
                t.dt_us = now_us - t.last_us;
                t.last_us = now_us;
                
                if (t.callback) {
                    t.callback();
                }
            }
        }
    }

    float getTaskDtSeconds(uint8_t task_index) const {
        if (task_index >= task_count_) return 0.0f;
        return tasks_[task_index].dt_us * 1e-6f;
    }

    uint8_t taskCount() const {
        return task_count_;
    }

private:
    Task tasks_[MAX_TASKS];
    uint8_t task_count_;
};
