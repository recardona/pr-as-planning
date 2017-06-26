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
#ifndef __SQUARE_MATRIX__
#define __SQUARE_MATRIX__

template <typename T>
class Square_Matrix
{
public:

	Square_Matrix()
		: m_data(NULL), m_N( 0 )
	{
	}
	
	Square_Matrix( unsigned N )
		: m_data(NULL), m_N( N )
	{
		resize( m_N*m_N );
	}
	
	~Square_Matrix()
	{
		if ( m_data ) delete [] m_data;
	}

	T& operator ()( unsigned i, unsigned j )
	{
		return m_data[i*m_N+j];
	}
	
	const T& operator()(unsigned i, unsigned j ) const
	{
		return m_data[i*m_N+j];
	}
	
	T& at( unsigned i, unsigned j )
	{
		assert( i >= 0 && i < m_N );
		assert( j >= 0 && j < m_N );
		return m_data[i*m_N+j];
	}

protected:

	void resize( unsigned sz )
	{
		m_data = new T[sz];
	}
	
protected:
	T*        m_data;
	unsigned  m_N;
};

#endif // square_matrix.hxx
