#ifndef __NFF_HEURISTIC__
#define __NFF_HEURISTIC__

#include "PDDL.hxx"
#include "nff_state.hxx"

namespace NFF
{
 class Heuristic {

  protected:
    PDDL::Task &task;

  public:
    
    Heuristic() : task(PDDL::Task::instance()) { }
    virtual ~Heuristic() { }

    virtual float eval( State &s ) = 0;

  };
}

#endif // nff_heuristic.hxx
