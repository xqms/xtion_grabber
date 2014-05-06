// Pool allocator that provides boost::shared_ptr access
// Author: Max Schwarz <max.schwarz@online.de>

#ifndef UTILS_POOL_H
#define UTILS_POOL_H

#include <stack>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <ros/console.h>

namespace utils
{

template<class T, class Allocator = std::allocator<T> >
class Pool : public boost::enable_shared_from_this<Pool<T, Allocator> >
{
public:
	typedef boost::shared_ptr<Pool<T, Allocator> > Ptr;

	Pool(const Allocator& alloc = Allocator());
	~Pool();

	template<typename... Args>
	boost::shared_ptr<T> create(Args&&... args);
private:
	void takeBack(T* obj);

	std::stack<T*> m_pool;
	Allocator m_alloc;
};

// IMPLEMENTATION
template<class T, class Allocator>
Pool<T, Allocator>::Pool(const Allocator& alloc)
 : m_alloc(alloc)
{
}

template<class T, class Allocator>
Pool<T, Allocator>::~Pool()
{
	while(!m_pool.empty())
	{
		m_alloc.destroy(m_pool.top());
		m_alloc.deallocate(m_pool.top(), 1);
		m_pool.pop();
	}
}

template<class T, class Allocator>
template<typename... Args>
boost::shared_ptr<T> Pool<T, Allocator>::create(Args&&... args)
{
	T* obj;

	if(m_pool.empty())
	{
		obj = m_alloc.allocate(1, 0);
		m_alloc.construct(obj);
	}
	else
	{
		obj = m_pool.top();
		m_pool.pop();
	}

	return boost::shared_ptr<T>(
		obj,
		boost::bind(&Pool<T, Allocator>::takeBack, this->shared_from_this(), _1)
	);
}

template<class T, class Allocator>
void Pool<T, Allocator>::takeBack(T* obj)
{
	m_pool.push(obj);
}

};

#endif
