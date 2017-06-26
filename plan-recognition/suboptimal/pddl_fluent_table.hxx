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
#ifndef __PDDL_FLUENT_SET_TABLE__
#define __PDDL_FLUENT_SET_TABLE__

#include "jenkins_12bit.hxx"
#include "pddl_basic_types.hxx"
#include <vector>
#include <list>

namespace PDDL
{

template <typename T>
class Fluent_Set_Hashtable
{
public:
	typedef T								Object;
	typedef std::list< std::pair< unsigned, Object* > > 			Node_List;
	typedef std::vector< Node_List > 					Table;

	Fluent_Set_Hashtable( unsigned hash_sz );
	~Fluent_Set_Hashtable();
	
	void clear();
	
	size_t compute_hash( std::vector<unsigned>& atoms) const
	{
		if ( atoms.empty() )
		{
			unsigned k = 0;
			unsigned h = jenkins_hash( (ub1*)&k, sizeof(unsigned), 0 );
			return h;
		}

		std::sort( atoms.begin(), atoms.end() );
		unsigned h = jenkins_hash( (ub1*)(&atoms[0]), sizeof(unsigned), 0 );
		for ( unsigned i = 1; i < atoms.size(); i++ )
		{
			h = jenkins_hash( (ub1*)(&atoms[i]), sizeof(unsigned), h );
		}	
		return h;
	}

	Object* get_element( unsigned h)
	{
		assert( m_buckets.size() > 0 );
		unsigned address = h & ( m_buckets.size()-1 );
		
		for ( typename Node_List::iterator i = m_buckets[address].begin();
			i != m_buckets[address].end(); i++ )
			if ( i->first == h ) return i->second;
		return NULL;
	}

	Object* get_element( unsigned h, Object* other )
	{
		assert( m_buckets.size() > 0 );
		unsigned address = h & ( m_buckets.size()-1 );
		
		for ( typename Node_List::iterator i = m_buckets[address].begin();
			i != m_buckets[address].end(); i++ )
			if ( (i->first == h) && (i->second->operator==(*other) ) ) return i->second;
		return NULL;
	}	

	void add_element( unsigned h, Object* obj )
	{
		unsigned address = h & ( m_buckets.size() - 1);
		m_buckets[address].push_back( std::make_pair( h, obj ) );
	}

	void remove_element( unsigned h )
	{
		unsigned address = h & ( m_buckets.size() - 1);
		
		typename  Node_List::iterator p = m_buckets[address].end();
		for ( typename Node_List::iterator i = m_buckets[address].begin();
			i != m_buckets[address].end(); i++ )
		{
			if ( i->first == h ) // element found
			{
				delete i->second;
				p = i;
				break;
			}
		}	
		if ( p != m_buckets[address].end() )
			m_buckets[address].erase( p );
	}

	void remove_element( unsigned h, Object* obj )
	{
		unsigned address = h & ( m_buckets.size() - 1);
		
		typename  Node_List::iterator p = m_buckets[address].end();
		for ( typename Node_List::iterator i = m_buckets[address].begin();
			i != m_buckets[address].end(); i++ )
		{
			if ( i->first == h && (i->second->operator==(*obj)) ) // element found
			{
				delete i->second;
				p = i;
				break;
			}
		}	
		if ( p != m_buckets[address].end() )
			m_buckets[address].erase( p );
	}
	
protected:
	Table 		m_buckets;
};

// Inlined methods
template <typename T>
Fluent_Set_Hashtable<T>::Fluent_Set_Hashtable( unsigned hash_sz )
{
	m_buckets.resize(hash_sz);
}

template <typename T>
Fluent_Set_Hashtable<T>::~Fluent_Set_Hashtable()
{
	for ( unsigned i = 0; i < m_buckets.size(); i++ )
	{
		for ( typename Node_List::iterator it = m_buckets[i].begin();
			it != m_buckets[i].end(); it++ )
			delete it->second;
		m_buckets[i].clear();	
	}
	m_buckets.clear();
}

template <typename T>
void Fluent_Set_Hashtable<T>::clear()
{
	for ( unsigned i = 0; i < m_buckets.size(); i++ )
	{
		for ( typename Node_List::iterator it = m_buckets[i].begin();
			it != m_buckets[i].end(); it++ )
			delete it->second;
		m_buckets[i].clear();	
	}
}

}

#endif // pddl_fluent_table.hxx
