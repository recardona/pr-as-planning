#include <iostream>
#include <cstdlib>
#include <fstream>
#include <signal.h>
#include "utils.hxx"
#include "PDDL.hxx"
#include "options.hxx"
#include "pr_obs_reader.hxx"
#include "pr_strips_mapping.hxx"
#include "string_ops.hxx"

int main( int argc, char** argv )
{
	double t0, tf;

	Options::parse_command_line( argc, argv );
	Options& prog_opts = Options::instance();

	std::ofstream stats( "execution.stats" );	


	t0 = time_used();
	PDDL::Task& task = PDDL::Task::instance();
	task.setup();

	tf = time_used();
	stats << "Domain=" << task.domain_name() << std::endl;
	stats << "Problem=" << task.problem_name() << std::endl;	

	stats << "Preprocessing=";  report_interval( t0, tf, stats );


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
	std::cout << "Initial state: ";

	PR_Observation_Stream_Reader obs_stream_reader;
	obs_stream_reader.parse( prog_opts.obs_filename() );

	if ( !prog_opts.prob_pr_mode() )
	{
		PR_STRIPS_Mapping writer( obs_stream_reader.obs_stream() );
		writer.write();
		
		return 0;
	}
	system( "rm -rf prob-PR" );
	system( "mkdir prob-PR" );

	PR_STRIPS_Mapping writer( obs_stream_reader.obs_stream(), false, prog_opts.convert_to_integer(), prog_opts.factor() );
	std::string path( "prob-PR/O/" );
	writer.set_base_path( path );
	std::string cmd = "mkdir " + path;
	system( cmd.c_str() );
	writer.write();

	PR_STRIPS_Mapping writer2( obs_stream_reader.obs_stream(), true, prog_opts.convert_to_integer(), prog_opts.factor() );
	path =  "prob-PR/neg-O/";
	writer2.set_base_path( path );
	cmd = "mkdir " + path;
	system( cmd.c_str() );
	writer2.write();

	stats.close();
	std::exit(0);

	return 0;
}
