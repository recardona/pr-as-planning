/*
    Miguel Ramirez, Nir Lipovetzky, Hector Geffner
    C^3: A planner for the sequential, satisficing track of the IPC-6
    Copyright (C) 2008  

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef __NFF_ALGOBASE__
#define __NFF_ALGOBASE__

#include "nff.hxx"

namespace NFF
{

class DFS_Skeleton
{
public:
	DFS_Skeleton();
	~DFS_Skeleton();

	unsigned		expanded;
	unsigned		C1_failed;
	unsigned		C2_failed;
	unsigned		C2_pruned;
	unsigned		dead_ends;
	unsigned		duplicates;
	unsigned		max_label_size;
	float			avg_label_size;
	float			avg_branching_factor;
	unsigned		evaluated;
	unsigned		preferences_used;
	// For ease of use (again)
	Operator_Vec			path_found;

};

class BNB_Skeleton : public DFS_Skeleton
{
public:
	BNB_Skeleton();
	~BNB_Skeleton();
	
	unsigned		num_plans;
	unsigned 		B;
};

}

#endif // nff_algobase.hxx
