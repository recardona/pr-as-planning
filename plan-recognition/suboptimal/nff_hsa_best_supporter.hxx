#ifndef __NFF_HSA_BEST_SUPPORTER__
#define __NFF_HSA_BEST_SUPPORTER__

#include "nff_pb_best_supporter.hxx"

namespace NFF
{

class hsaBestSupporter : public virtual PBBestSupporter 
{
  
protected:
    	//    std::vector<RelaxedPlan> supportPlan;
	//    std::vector<std::list<unsigned> > supportPlan;
	std::vector<RelaxedPlan> supportPlan;
	State*			 current_state;
	
	virtual void initialize( State &s );
	virtual void doPropagation( );
	void merge(RelaxedPlan&, RelaxedPlan&);

	virtual void setSupporter(unsigned f, unsigned a) 
	{

		RelaxedPlan rp;
		RelaxedPlan::iterator it;
      
		for(unsigned i = 0; i < task.useful_ops()[a]->prec_vec().size(); i++) 
		{
			merge(rp, supportPlan[task.useful_ops()[a]->prec_vec()[i]]);
		}
      
		it = rp.begin();
		while((*it < a) && (it != rp.end())) it++;
		rp.insert(it, a);

		setSupporter(f, a, rp);
	}

	void setSupporter(unsigned f, unsigned a, RelaxedPlan &plan) 
	{
		PBBestSupporter::setSupporter(f, a);
		supportPlan[f] = plan;
	}
 
	bool valid_obs_extension( unsigned ao, RelaxedPlan& pi );
	bool check_for_loops( Operator_Vec& supportees, unsigned prec, RelaxedPlan& pi ); 
public:
	hsaBestSupporter(); 

	virtual ~hsaBestSupporter();
	virtual void computeSupports( State &s );
	virtual void printSupport(unsigned f, std::ostream& os);
	virtual void printSupport(unsigned f);
};


}

#endif // nff_hsa_best_supporter.hxx
