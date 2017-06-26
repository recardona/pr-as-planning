/*
    Miguel Ramirez, Hector Geffner
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
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <signal.h>
#include "utils.hxx"
#include "PDDL.hxx"
#include "nff_options.hxx"
#include "nff_h2.hxx"
#include "nff_state.hxx"
#include "nff_hsa_best_supporter.hxx"
#include "nff_harp_adapter.hxx"
#include "nff_helpful.hxx"
#include "nff_ehc.hxx"
#include "nff_bfs.hxx"

void write_plan ( NFF::Operator_Vec& plan )
{
	PDDL::Task& task = PDDL::Task::instance();
	NFF_Options& prog_opts = NFF_Options::instance();
	std::string plan_fname = prog_opts.instance_filename();
	if ( plan_fname.find( "pddl" ) == std::string::npos )
		plan_fname += "soln";
	else
		plan_fname.replace( plan_fname.find("pddl"), 4, "soln" );

	std::ofstream plan_stream( plan_fname.c_str() );
	
	double plan_cost = 0.0;
	for ( unsigned i = 0; i < plan.size(); i++ )
	{
		plan_stream << i << " : ";
		task.print_operator( plan[i], plan_stream );
		plan_stream << " [" << task.op_cost(plan[i]) << ", " << (task.useful_ops()[plan[i]]->accounts_obs() ? "Obs" : "Regular") << "]" << std::endl;
		plan_cost += task.op_cost(plan[i]);
	}	
	plan_stream << ";;MetricValue " << plan_cost << std::endl;
	plan_stream.close();

}

void eval_s0( )
{
	PDDL::Task& task = PDDL::Task::instance();

	NFF::State* s0 = NFF::State::make_initial_state();
	NFF::BestSupporter *bs = new NFF::hsaBestSupporter;
	NFF::HelpfulActionExtractor *hae = new NFF::PlanActions(*bs);
	NFF::HARPHeuristicAdapter *ha = new NFF::HARPHeuristicAdapter(bs, hae);
	ha->eval( *s0 );
	std::ofstream out( "hyp-rank" );
	out << ha->num_obs_accounted() << std::endl;
	out.close();
	NFF::Operator_Vec linear_relaxed_plan;
	ha->relaxed_plan().linearize( *s0, linear_relaxed_plan );
	write_plan( linear_relaxed_plan );
}

void solve()
{ 
	PDDL::Task& task = PDDL::Task::instance();
	NFF_Options& opt = NFF_Options::instance();
	NFF::Enforced_Hill_Climbing incomplete_search;
	unsigned num_obs_accounted = 0;
	if (opt.bfs() || !incomplete_search.solve())
	{
		NFF::Best_First_Search complete_search;
		if (!complete_search.solve())
		{
			std::exit(1);
		}
		for ( unsigned k = 0; k < complete_search.path_found.size(); k++ )
			if ( task.is_obs( complete_search.path_found[k] ) )
				num_obs_accounted++;
		write_plan( complete_search.path_found );
	}
	else
	{
		for ( unsigned k = 0; k < incomplete_search.path_found.size(); k++ )
			if ( task.is_obs( incomplete_search.path_found[k] ) )
				num_obs_accounted++;
		write_plan( incomplete_search.path_found );
	}
	std::ofstream out( "hyp-rank" );
	out << num_obs_accounted << std::endl;
	out.close();
	
}

int main( int argc, char** argv )
{
	double t0, tf;

	NFF_Options::parse_command_line( argc, argv );
	NFF_Options& prog_opts = NFF_Options::instance();

	PDDL::Task& task = PDDL::Task::instance();
	task.setup();


	std::ofstream stats( "execution.stats" );	

	if ( prog_opts.verbose_mode() )
	{
		std::ofstream fluents_out( "fluents.list" );
		std::ofstream ops_out( "operators.list" );
		std::ofstream init_out( "initial.list" );
		std::ofstream goal_out( "goal.list" );
	
		task.print_fluents( fluents_out );
		task.print_operators( ops_out );
		task.print_initial_state( init_out );
		task.print_goal_state( goal_out );

		fluents_out.close();
		ops_out.close();
	}
	stats << "Fluents=" << task.fluents().size() << std::endl;
	std::cout << "Fluents=" << task.fluents().size() << std::endl;
	stats << "Operators=" << task.useful_ops().size() << std::endl;	
	std::cout << "Operators=" << task.useful_ops().size() << std::endl;

	if ( NFF_Options::instance().only_grounding() )
	{
		std::cout << "Stopping after PDDL task has been processed" << std::endl;
		std::exit(0);
	}

	task.set_h2( new NFF::H2 );
	t0 = time_used();
	task.h2().compute_only_mutexes();
	bool solvable = task.h2().extract_op_reachability_info();
	task.h2().compute_e_deletes();
	tf = time_used();
	
	stats << "Preprocessing=";  report_interval( t0, tf, stats );

	if ( !solvable )
	{
		stats << "Plan=No" << std::endl;
		// OK, let's see what pairs of End preconditions are mutex
		std::cout << "Planning task not solvable" << std::endl;
		
		NFF::Atom_Vec& goals = task.useful_ops()[task.end()]->prec_vec();
		for ( unsigned k = 0; k < goals.size(); k++ )
			for ( unsigned j = k; j < goals.size(); j++ )
				if ( task.h2().value( goals[k], goals[j] ) == std::numeric_limits<unsigned>::max() )
				{
					std::cout << "Goal atom pair (";
					task.print_fluent( goals[k], std::cout );
					std::cout << ", ";
					task.print_fluent( goals[j], std::cout );
					std::cout << ") is mutex" << std::endl;
				}

		std::exit(0);
	}	

	#ifndef NDEBUG
	std::cout << "Initial state: ";
	NFF::State* s0 = NFF::State::make_initial_state();	
	s0->print( std::cout );
	std::cout << std::endl;
	std::cout << "Goal state: ";
	NFF::State* sG = NFF::State::make_goal_state();
	sG->print( std::cout );
	std::cout << std::endl;
	#endif

	if ( NFF_Options::instance().only_s0() )
		eval_s0();
	else
		solve();
	stats.close();
	
	std::exit(0);
	return 0;
}
