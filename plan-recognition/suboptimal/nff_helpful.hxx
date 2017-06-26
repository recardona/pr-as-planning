#ifndef __HELPFUL_H__
#define __HELPFUL_H__


namespace NFF
{

class BestSupporter;
class RelaxedPlan;
class State;

class HelpfulActionExtractor 
{
    
protected:
    	PDDL::Task &task;
	BestSupporter &bs;
    
public:
	HelpfulActionExtractor(BestSupporter &bs) 
		: task(PDDL::Task::instance()), bs(bs) 
	{ 
	}
	
	virtual ~HelpfulActionExtractor() 
	{ 
	}
	virtual void extract( State &s, RelaxedPlan &rp, std::vector<unsigned> &helpful);
};

class RPHA_w_mutexes : public HelpfulActionExtractor 
{
    
public:

    	RPHA_w_mutexes(BestSupporter &bs) 
		: HelpfulActionExtractor(bs) 
	{
	}
	
	virtual ~RPHA_w_mutexes() { }
	virtual void extract( State &s, RelaxedPlan &rp, std::vector<unsigned> &helpful);
};

class NoActions : public HelpfulActionExtractor 
{
public:
    	NoActions(BestSupporter &bs) :
		HelpfulActionExtractor(bs)
	{ 
	}

	virtual ~NoActions() { }
	virtual void extract( State &s, RelaxedPlan &rp, std::vector<unsigned> &helpful);
};
  
class PlanActions : public HelpfulActionExtractor 
{

public:
    	PlanActions(BestSupporter &bs) 
		: HelpfulActionExtractor(bs)
	{ 
	}
	virtual ~PlanActions() 
	{ 
	}
	virtual void extract( State &s, RelaxedPlan &rp, std::vector<unsigned> &helpful);
};

}
#endif
