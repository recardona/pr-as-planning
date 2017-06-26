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
#include "nff_options.hxx"
#include "global_options.hxx"
#include <getopt.h>
#include <iostream>
#include <cstdlib>

const char* NFF_Options::m_optstring = "d:i:v:h?g:IB";

NFF_Options::NFF_Options()
	: m_verbose( false ),m_only_grounding( false ), 
	m_only_eval_s0( false ), m_bfs( false )
{
}

NFF_Options::~NFF_Options()
{
}

NFF_Options& NFF_Options::instance()
{
	static NFF_Options instance;
	return instance;
}

void NFF_Options::parse_command_line( int argc, char** argv )
{
	int opt = getopt( argc, argv, m_optstring );
	bool domain_specified = false;
	bool instance_specified = false;
	
	Global_Options& g_options = Global_Options::instance();
	NFF_Options& options = instance();
	
	while( opt != -1 )
	{
		switch (opt)
		{
		case 'd' : // domain file
			options.m_domain_fname = optarg;
			g_options.set_domain_filename( options.m_domain_fname );
			domain_specified = true;
			break;
		case 'i' : // instance file
			options.m_instance_fname = optarg;
			g_options.set_instance_filename( options.m_instance_fname );
			instance_specified = true;
			break;
		case 'I' :
			options.m_only_eval_s0 = true;
			break;
		case 'v' : // verbose mode activated
			options.m_verbose = atoi(optarg);
			break;
		case 'g' :
		{
			int step = atoi(optarg);
			switch( step )
			{
			case 1:
				std::cout << "Stopping after Grounding" << std::endl; 
				options.m_only_grounding = true;
				break;
			default:
				std::cerr << "Unrecognized program step ("<< step << "), aborting" << std::endl;
				print_usage();
				std::exit(0);
				break;	
			}
			break;
		}
		case 'B':
			options.m_bfs = true;
			break;
		case '?':
		case 'h':
			print_usage();
			std::exit(0);
			break;
		default:
			std::cerr << "Unrecognized option" << opt << std::endl;
			print_usage();
		}
		opt = getopt( argc, argv, m_optstring );
	}

	if ( !domain_specified )
	{
		std::cerr << "PDDL domain description not specified" << std::endl;
		print_usage();
		std::exit(1);
	}	
	if ( !instance_specified )
	{
		std::cerr << "PDDL instance description not specified" << std::endl;
		print_usage();
		std::exit(1);
	}
}

void NFF_Options::print_usage()
{
	std::cerr << "Sub-optimal plan recognizer (c) Universitat Pompeu Fabra, 2008" << std::endl;
	std::cerr << "Authors: Hector Geffner, Miguel Ramirez" << std::endl;
	std::cerr << std::endl;
	std::cerr << "Usage: ./subopt_PR -d <domain file> -i <instance file> [-I|-v|-g:1]" << std::endl << std::endl;
	std::cerr << "Mandatory parameters:" << std::endl;
	std::cerr << "-d            Domain specification in PDDL 2.1" << std::endl;
	std::cerr << "-i            Instance specification in PDDL 2.1" << std::endl;
	std::cerr << "-I            Evaluate only based on info from initial state" << std::endl;
	std::cerr << std::endl;
	std::cerr << "Optional parameters: " << std::endl;
	std::cerr << "-v            Verbose Mode ON (default is OFF)" << std::endl;
	std::cerr << "-g <number>   Debugging - Stop execution when some process step completes:" << std::endl;
	std::cerr << "      1                   PDDL parsing, ADL -> STRIPS, and preprocessing finishes" << std::endl;
	std::cerr << "-B            Bypass EHC search and solve the problem by Greedy Best First Search" << std::endl;
}
