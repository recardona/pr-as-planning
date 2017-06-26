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
#include "nff_algobase.hxx"

namespace NFF
{

DFS_Skeleton::DFS_Skeleton()
	: expanded( 0 ), C1_failed(0), C2_failed(0), C2_pruned(0), dead_ends(0), duplicates(0),
	max_label_size(0), avg_label_size(0), avg_branching_factor(0), evaluated(0),
	preferences_used(0)
{
}

DFS_Skeleton::~DFS_Skeleton()
{
}

BNB_Skeleton::BNB_Skeleton()
	: DFS_Skeleton(), num_plans(0), B(0)
{
}

BNB_Skeleton::~BNB_Skeleton()
{
}

}
