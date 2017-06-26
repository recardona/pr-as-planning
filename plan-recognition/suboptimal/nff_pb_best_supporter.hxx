#ifndef __NFF_PB_BEST_SUPPORTER__
#define __NFF_PB_BEST_SUPPORTER__

#include "nff_best_supporter.hxx"

namespace NFF
{

  /* propagation based best supporter */
class PBBestSupporter : public BestSupporter 
{

protected:

    	std::vector<float> fact_cost;
	/* cost(pre(a)) */
	std::vector<float> pcs_cost;
	
	/* attempt apply action ? */
	//    std::vector<bool> aa;
	
	virtual void initialize( State &s );
	virtual void doPropagation( ) = 0; 
	
	virtual void setSupporter(unsigned f, unsigned a) 
	{
		BestSupporter::setSupporter(f, a);
		fact_cost[f] = getPCsCost(a) + getActionCost(a);
	}

public:
	PBBestSupporter();

	virtual ~PBBestSupporter();

	virtual float getFluentCost(unsigned f) { return fact_cost[f]; }
	virtual float getPCsCost(unsigned a) { return pcs_cost[a]; }
	virtual void printSupport(unsigned f, std::ostream& os);
	
	virtual void printSupport(unsigned f) 
	{
		printSupport(f, std::cout);
	}
};

}

#endif // nff_pb_best_supporter.hxx
