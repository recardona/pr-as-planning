#ifndef __NFF_MIN_ADAPTER__
#define __NFF_MIN_ADAPTER__

#include "nff_heuristic_adapter.hxx"
#include <limits>

namespace NFF
{

class MinHeuristicAdapter : public HeuristicAdapter 
{
    
protected:
	std::vector<HeuristicAdapter*> heuristics; 
	std::vector<std::pair<float, std::vector<unsigned> > > results;
    
public:
	void addHeuristic(HeuristicAdapter *toadd) 
	{
		heuristics.push_back(toadd);
	}

	virtual bool suggestsHelpful() 
	{
		bool sH = false;
		for(unsigned i = 0; i < heuristics.size(); i++) 
		{
			sH |= heuristics[i]->suggestsHelpful();
		}
		return sH;
	}
    
	virtual float eval(State &s, std::vector<unsigned> &helpful) 
	{
		float min = std::numeric_limits<float>::max();
		unsigned minindex = -1;
		for(unsigned i = 0; i < heuristics.size(); i++) 
		{
			results[i].first = heuristics[i]->eval(s, results[i].second);
			if(results[i].first < min) 
			{
				min = results[i].first;
				minindex = i;
			}
		}
		helpful = results[minindex].second;
		return min;
	}
};


}

#endif // nff_min_adapter.hxx
