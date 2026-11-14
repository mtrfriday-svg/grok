#ifndef PERFORMER_H
#define PERFORMER_H

#include <cstddef>

class Performer {
public:
    static Performer& GetInstance();
    void Init();
    bool Start(const char* name);
    void Cancel();
    void NotePat();
    bool Running() const { return running_; }
    bool InTask() const;
    const char* Current() const { return current_; }

private:
    Performer() = default;
    static void Task(void* arg);
    void Dispatch(const char* name);
    bool Wait(int ms);
    bool PatTaken();

    bool running_ = false;
    bool cancel_ = false;
    bool pat_ = false;
    void* task_ = nullptr;
    void* queue_ = nullptr;
    char current_[24] = {0};
};

// Called from drive/stop on any task. No-op when the performer task itself is driving.
void PerformerCancelFromOutside();

#endif
