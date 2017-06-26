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
		std::cout << "Planning task not solvable" << std::endl;
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

	
	stats.close();
	
	std::exit(0);
	return 0;
}
