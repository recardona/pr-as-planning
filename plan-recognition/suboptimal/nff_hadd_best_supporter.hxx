#ifndef __NFF_HADD_BEST_SUPPORTER__
#define __NFF_HADD_BEST_SUPPORTER__

#include "nff_pb_best_supporter.hxx"

namespace NFF
{

class haddBestSupporter : public virtual PBBestSupporter 
{

protected:
    	virtual void doPropagation( );  
public:
      	virtual ~haddBestSupporter() { }
	virtual void computeSupports( State &s );
};
  
}

#endif // nff_hadd_best_supporter.hxx
