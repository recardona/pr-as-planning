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
#ifndef __NFF_OPTIONS__
#define __NFF_OPTIONS__

#include <string>


class NFF_Options
{
public:
	static NFF_Options& 	instance();
	static void 		parse_command_line( int argc, char** argv );
	~NFF_Options();
	static void		print_usage();

	std::string& 		domain_filename() { return m_domain_fname; }
	std::string& 		instance_filename() { return m_instance_fname; }
	unsigned     		verbose_mode() { return m_verbose; }
	bool			only_grounding() { return m_only_grounding; }
	bool			only_s0() { return m_only_eval_s0; }
	bool			bfs() { return m_bfs; }

private:
	NFF_Options();
private:

	std::string		m_domain_fname;
	std::string		m_instance_fname;
	unsigned		m_verbose;
	static const char*	m_optstring;
	bool			m_only_grounding;
	bool			m_only_eval_s0;
	bool			m_bfs;
};


#endif // nff_options.hxx
