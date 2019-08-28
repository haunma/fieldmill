#ifndef __DTIMERS_H
#define __DTIMERS_H

class DTimers
{
public:
    // Constructor/destructor/init
    DTimers();
    virtual ~DTimers() {};

    // Public API
    void Init();       // Call during system init

private:
    DTimers(const DTimers&);
    void operator=(const DTimers&);
};

extern DTimers g_timers;

#endif // __DTIMERS_H defined