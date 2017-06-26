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
#include "PDDL.hxx"
#include <iostream>
#include <cassert>
#include "nff_h2.hxx"
#include "utils.hxx"
#include "global_options.hxx"
#include "libff.h"

namespace PDDL
{

Fluent::Fluent( int code, bool is_explanation )
	: m_code( code ), m_is_obs_explanation( is_explanation ), m_enabled( true )
{
}

Fluent::~Fluent()
{
}
Operator::Operator()
	: m_xdels( NULL ), m_metric(0.0f), m_explains_obs( false ), m_forgoes_obs( false ),
	m_req_explanation( 0 ), m_provided_explanation( 0 )
{
	PDDL::Task& task = PDDL::Task::instance();
	m_precondition = new Fluent_Set( task.fluent_count() );
	m_adds = new Fluent_Set( task.fluent_count() );
	m_dels = new Fluent_Set( task.fluent_count() );
}

Operator::Operator( int code )
	: m_xdels(NULL), m_code( code ), m_metric(0.0f), m_explains_obs( false ), m_forgoes_obs( false ),
	m_req_explanation( 0 ), m_provided_explanation( 0 )
{
	PDDL::Task& task = PDDL::Task::instance();
	m_precondition = new Fluent_Set( task.fluent_count() );
	m_adds = new Fluent_Set( task.fluent_count() );
	m_dels = new Fluent_Set( task.fluent_count() );
}

Operator*	Operator::new_regular_op(int code)
{
	return new Operator(code);
}

Operator*	Operator::new_explain_op( int code )
{
	Operator* op = new Operator(code);
	op->m_explains_obs = true;
	op->m_forgoes_obs = false;
	return op;
}

Operator*	Operator::new_forgo_op( int code )
{
	Operator* op = new Operator(code);
	op->m_explains_obs = false;
	op->m_forgoes_obs = true;
	return op;
}


void Operator::make_xdel_lists()
{
	PDDL::Task& task = PDDL::Task::instance();
	m_xdels = new Fluent_Set( 2*(task.fluent_count()+1) );
	for ( unsigned i = 0; i < m_del_vec.size(); i++ )
	{
		m_xdel_vec.push_back( NFF::Lit( m_del_vec[i] ) );
		m_xdels->set( NFF::toInt( m_xdel_vec.back() ) );
	}
	for ( unsigned i = 0; i < m_add_vec.size(); i++ )
	{
		m_xdel_vec.push_back( NFF::Lit( m_add_vec[i], true ) );
		m_xdels->set( NFF::toInt( m_xdel_vec.back() ) );
	}
}

Operator::~Operator()
{
	delete m_precondition;
	delete m_adds;
	delete m_dels;
	if ( m_xdels ) delete m_xdels;
}

Task::Task()
	: m_string_table( String_Table::instance() ), m_glog( "grounding.stats" )
{
}

Task::~Task()
{
}

Task& Task::instance()
{
	static Task the_instance;
	return the_instance;
}

void Task::setup()
{
	Global_Options& opt = Global_Options::instance();
	
	float t0 = time_used();
	FF_parse_problem( opt.domain_filename().c_str(), opt.instance_filename().c_str() );
	float tf = time_used(); 
	m_glog << "Parsing="; report_interval( t0, tf, m_glog );
	
	t0 = time_used();
	FF_instantiate_problem();	
	tf = time_used();
	m_glog << "Instantiation="; report_interval( t0, tf, m_glog );

	t0 = time_used();
	create_fluents();
	tf = time_used();
	m_glog << "Fluent_Copying="; report_interval( t0, tf, m_glog );

	t0 = time_used();
	create_init_and_goal();
	tf = time_used();
	m_glog << "Init_And_Goal_Copying="; report_interval( t0, tf, m_glog );

	t0 = time_used();
	create_operators();
	tf = time_used();
	m_glog << "Operators_Copying="; report_interval( t0, tf, m_glog );

	// Setup costs
	m_op_costs.resize( useful_ops().size() );
	for ( unsigned o = 0; o < useful_ops().size(); o++ )
		m_op_costs[o] = useful_ops()[o]->metric_cost();	

	// Sort observations
	sort_observations();
	match_ops_and_obs();
}

void Task::sort_observations()
{
	// Find first observation
	if ( m_explain_ops.empty() ) return;
	
	// Allocate room for explaining ops for first obs
	m_sorted_obs.push_back( std::vector<unsigned>() );
		
	for ( unsigned i = 0; i < m_explain_ops.size(); i++ )
	{
		PDDL::Operator* op_ptr = useful_ops()[ m_explain_ops[i] ];
		if ( op_ptr->explanation_req() == 0 )
		{
			m_sorted_obs[0].push_back( m_explain_ops[i] );
			m_sorted_explanations.push_back( op_ptr->explains() ); 
		}
	}

	std::cout << "Operators explaining first obs: ";
	for ( unsigned k = 0; k < m_sorted_obs[0].size(); k++ )
	{
		print_operator( m_sorted_obs[0][k], std::cout );
		if ( k < m_sorted_obs[0].size() - 1 )
			std::cout << ", ";
	}
	std::cout << std::endl;
	
	PDDL::Operator* prev_op_ptr = useful_ops()[ m_sorted_obs[0][0] ];
	m_sorted_explanations.push_back( useful_ops()[m_sorted_obs[0][0]]->explains() );

	std::vector<unsigned> new_ops;
	do
	{
		new_ops = required_by( prev_op_ptr->explains() );
	
		if ( !new_ops.empty() )
		{
			// Allocate space for new obs
			m_sorted_obs.push_back( std::vector<unsigned>() );
			for ( unsigned k = 0; k < new_ops.size(); k++ )
				if (new_ops[k] != end() )
					m_sorted_obs.back().push_back( new_ops[k] );
			if ( !m_sorted_obs.back().empty() )
			{
				prev_op_ptr = useful_ops()[m_sorted_obs.back()[0]];
				m_sorted_explanations.push_back( prev_op_ptr->explains() );
			}
			else
				prev_op_ptr = NULL;
		}
	} while ( prev_op_ptr != NULL );
	std::cout << "Sorted observations:" << std::endl;
	for ( unsigned k = 0; k < m_sorted_obs.size(); k++ )
	{
		for ( unsigned j = 0; j < m_sorted_obs[k].size(); j++ )
		{
			print_operator( m_sorted_obs[k][j], std::cout );
			if ( j < m_sorted_obs[k].size() - 1 )
				std::cout << ", ";
		}
		std::cout << std::endl;
	}
}

bool Task::precedes_in_obs_seq( unsigned op1, unsigned op2 )
{
	assert( is_obs(op1) && is_obs(op2) );
	assert( op1 != op2 );
	bool found_op1 = false, found_op2 = false;
	unsigned op1_idx = std::numeric_limits<unsigned>::max(), op2_idx = std::numeric_limits<unsigned>::max();

	/*
	#ifndef NDEBUG
	std::cout << "Does op: ";
	print_operator( op1, std::cout );
	std::cout << " go before ";
	print_operator( op2, std::cout );
	std::cout << std::endl;
	#endif
	*/

	for ( unsigned k = 0; k < m_sorted_obs.size() && !found_op1; k++ )
	{
		for ( unsigned j = 0; j < m_sorted_obs[k].size(); j++ )
			if ( m_sorted_obs[k][j] == op1 ) 
			{ 
				op1_idx = k;
				found_op1 = true; 
				break; 
			}
	}

	for ( unsigned k = 0; k < m_sorted_obs.size() && !found_op2; k++ )
	{
		for ( unsigned j = 0; j < m_sorted_obs[k].size(); j++ )
			if ( m_sorted_obs[k][j] == op2 ) 
			{ 
				op2_idx = k;
				found_op2 = true; 
				break; 
			}
	}

	/*
	#ifndef NDEBUG
	std::cout << "Index of first: " << op1_idx << " and second: " << op2_idx << std::endl; 
	#endif	
	*/

	assert( found_op1 || found_op2 );
	return op1_idx < op2_idx;
}

void Task::create_fluents()
{
	fluents().push_back( NULL );

	m_is_explained.resize( gnum_ft_conn );
	
	for ( int i = 0; i < gnum_ft_conn; i++ )
	{
		std::string ft_name = FF::get_ft_name(i);
		
		int str_code = str_tab().get_code( ft_name );
		Fluent* new_f = NULL;
		if ( ft_name.find( "EXPLAINED" ) != std::string::npos )
		{
			new_f = new Fluent( str_code, true );
			new_f->disable();
			fluents().push_back( new_f );
			print_fluent( fluents().size()-1, std::cout );
			std::cout << std::endl;
			m_is_explained[fluents().size()-1] = true;
		}
		else
		{
			new_f = new Fluent( str_code );
			fluents().push_back( new_f );
		}
	}	
	m_required_by.resize( fluents().size() );
	m_added_by.resize( fluents().size() );
	m_deleted_by.resize( fluents().size() );
	m_e_deleted_by.resize( fluents().size() );
}

void Task::create_init_and_goal()
{
	FF::get_initial_state( m_initial_state );
	for ( unsigned i = 0; i < m_initial_state.size(); i++ )
		m_initial_state[i]++;
	std::string start_name( "(START)" );
	m_start_id = 0;
	useful_ops().push_back( new Operator( str_tab().get_code( start_name ) ) );
	for ( unsigned i = 0; i < m_initial_state.size(); i++ )
	{
		useful_ops()[0]->adds().set( m_initial_state[i] );
		useful_ops()[0]->add_vec().push_back( m_initial_state[i] );
	}
	useful_ops()[0]->make_xdel_lists();

	std::vector<unsigned> goals;
	FF::get_goal_state( goals );
	for ( unsigned i = 0; i < goals.size(); i++ )
		m_goal_state.push_back( goals[i]+1 );
	std::string end_name( "(END)" );
	m_end_id = 1;
	useful_ops().push_back( new Operator( str_tab().get_code( end_name ) ) );
	for ( unsigned i = 0; i < m_goal_state.size(); i++ )
	{
		useful_ops()[1]->preconds().set( m_goal_state[i] );
		useful_ops()[1]->prec_vec().push_back( m_goal_state[i] );
		required_by(m_goal_state[i]).push_back( 1 );
	}
	useful_ops()[1]->make_xdel_lists();
}

void Task::create_operators()
{
	for ( int i = 0; i < gnum_ef_conn; i++ )
	{
		if ( gef_conn[i].removed == TRUE ) continue;
		if ( gef_conn[i].illegal == TRUE ) continue;

		std::string op_name = FF::get_op_name(i);
		Operator* new_op = NULL;
		if ( op_name.find("EXPLAIN_OBS" ) != std::string::npos )
		{
			// Operator is an observation
			std::cout << op_name << " is " << useful_ops().size()-1 << std::endl;
			new_op = Operator::new_explain_op( str_tab().get_code( op_name ) );
			useful_ops().push_back( new_op );
			m_explain_ops.push_back( useful_ops().size()-1 );
		}
		else if ( op_name.find("FORGO_OBS") != std::string::npos )
		{
			// Operator is a forgo
			new_op = Operator::new_forgo_op( str_tab().get_code( op_name ) );
			useful_ops().push_back( new_op );
		}
		else
		{
			new_op = Operator::new_regular_op( str_tab().get_code( op_name ) );
			useful_ops().push_back( new_op );
		}	
	
		Operator* curr_op = useful_ops().back();
	
		for ( int j = 0; j < gef_conn[i].num_PC; j++ )
		{
			unsigned p = gef_conn[i].PC[j] + 1;
			m_required_by[p].push_back( useful_ops().size()-1 );
			curr_op->preconds().set( p );
			curr_op->prec_vec().push_back( p );
			if ( is_explained( p ) ) curr_op->set_req_explanation( p );
		}		

		for ( int j = 0; j < gef_conn[i].num_A; j++ )
		{
			unsigned p = gef_conn[i].A[j] + 1;
			m_added_by[p].push_back( useful_ops().size()-1 );
			curr_op->adds().set( p );
			curr_op->add_vec().push_back( p );
			if ( is_explained( p ) ) curr_op->set_explained( p );
		}

		for ( int j = 0; j < gef_conn[i].num_D; j++ )
		{
			unsigned p = gef_conn[i].D[j] + 1;
			m_deleted_by[p].push_back( useful_ops().size()-1 );
			curr_op->dels().set( p );
			curr_op->del_vec().push_back( p );
		}
		curr_op->make_xdel_lists();
		if ( gef_conn[i].num_IN == 0 )
			curr_op->set_metric_cost( 0 );
		else if ( gef_conn[i].num_IN >= 1 )
			curr_op->set_metric_cost( gef_conn[i].cost );//IN_c[0] );
	}
	m_op_edeletes.resize( useful_ops().size() );
	for ( unsigned p = 0; p < useful_ops().size(); p++ )
		m_fast_op_edeletes.push_back( new Fluent_Set( fluents().size()+1 ) );
	m_is_obs.resize( useful_ops().size() );
	std::cout << std::endl;
	std::cout << "Explain obs found:";
	for ( unsigned i = 0; i < m_explain_ops.size(); i++ )
	{
		std::cout << m_explain_ops[i] << " ";
		m_is_obs[ m_explain_ops[i] ] = true;
	}
	std::cout << std::endl;
}

void Task::match_ops_and_obs()
{
	m_ops_to_obs.resize( useful_ops().size() );
	for ( unsigned i = 2; i < useful_ops().size(); i++ )
	{
		PDDL::Operator* op_ptr = useful_ops()[i];
		Token& op_name = str_tab().get_token( op_ptr->code() );
		Token clean_op_name( ++std::find(op_name.begin(),op_name.end(), '('), std::find(op_name.begin(),op_name.end(), ' ') );
		if ( op_ptr->accounts_obs() ) continue;
		for ( unsigned j = 0; j < m_sorted_obs.size(); j++ )
		{
			for ( unsigned k = 0; k < m_sorted_obs[j].size(); k++ )
			{
				PDDL::Operator* obs_op_ptr = useful_ops()[ m_sorted_obs[j][k] ];
				Token& obs_name = str_tab().get_token( obs_op_ptr->code() );
				if ( obs_name.find( clean_op_name ) != std::string::npos )
				{
					#ifndef NDEBUG
					std::cout << "Operator"; print_operator( i, std::cout ); 
					std::cout << " matched with obs"; print_operator( m_sorted_obs[j][k], std::cout );
					std::cout << std::endl;
					#endif
					m_ops_to_obs[i].push_back( m_sorted_obs[j][k] );
				}
			}
		}
	}
}

void Task::print_fluent( Fluent* fi, std::ostream& os )
{
	os << str_tab().get_token( fi->code() );
}

void Task::print_fluents( std::ostream& os )
{
	for ( unsigned i = 1; i < fluents().size(); i++ )
	{
		os << i << " ";
		Fluent* fi = fluents()[i];
		print_fluent( fi, os );
		os << std::endl;
	}
}

void Task::print_initial_state( std::ostream& os )
{
	os << "Initial state:" << std::endl;
	for ( unsigned i = 0;  i < initial_state().size(); i++ )
	{
		print_fluent( fluents()[initial_state()[i]], os );
		os << std::endl;
	}
}

void Task::print_goal_state( std::ostream& os )
{
	os << "Goal state:" << std::endl;
	for ( unsigned i = 0; i < goal_state().size(); i++ )
	{
		print_fluent( fluents()[goal_state()[i]], os );
		os << std::endl;
	}
}

void Task::print_operator_full( Operator* oi, std::ostream& os )
{
	os << str_tab().get_token( oi->code() );
	os << ":" << std::endl;
	os << "\t" << "Preconditions:" << std::endl;
	for ( unsigned i = 0; i < oi->prec_vec().size(); i++ )
	{
		unsigned pi = oi->prec_vec()[i];
		os << "\t\t";
		print_fluent( fluents()[pi], os );
		os << std::endl;
	}
	os << "\t" << "Adds:" << std::endl;
	for ( unsigned i = 0; i < oi->add_vec().size(); i++ )
	{
		unsigned pi = oi->add_vec()[i];
		os << "\t\t";
		print_fluent( fluents()[pi], os );
		os << std::endl;
	}

	os << "\t" << "Deletes:" << std::endl;
	for ( unsigned i = 0; i < oi->del_vec().size(); i++ )
	{
		unsigned pi = oi->del_vec()[i];
		os << "\t\t";
		print_fluent( fluents()[pi], os );
		os << std::endl;
	}

}

void Task::print_operator( Operator* oi, std::ostream& os )
{
	os << str_tab().get_token( oi->code() );
}

void Task::print_operators( std::ostream& os )
{
	for ( unsigned i = 0; i < useful_ops().size(); i++ )
	{
		Operator* oi = useful_ops()[i];
		os << "Name:" << std::endl;
		print_operator_full( oi, os );
		os << "Cost: " << oi->metric_cost() << std::endl;
	}
}


void Task::set_start_time_lb( std::vector<unsigned>& stimes )
{
	assert( stimes.size() == useful_ops().size() );
	m_start_times_lb = stimes;
	m_is_reachable.resize( useful_ops().size() );
	for ( unsigned i = 0; i < useful_ops().size(); i++ )
		m_is_reachable[i] = ( start_time_lb(i) !=  std::numeric_limits<unsigned>::max() ? true : false );
}


}
