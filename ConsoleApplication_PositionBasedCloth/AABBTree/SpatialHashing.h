#ifndef SPATIAL_HASHING_H
#define SPATIAL_HASHING_H

#include "AABBox.h"

#include <Eigen\Dense>
#include <map>
#include <cmath>
#include <list>
#include <functional>

class Entry
{
public:
	Entry(Eigen::Vector3i const & v) 
		: entry(v)
	{
		num = hashing();
		//std::cout << entry << std::endl;
		//std::cout << hashSize << std::endl;
		//std::cout << num << std::endl << std::endl;
	}

	Entry(int x, int y, int z)
		: Entry(Eigen::Vector3i(x, y, z)) {}

	bool operator< (Entry const & rhs) const
	{
		return num < rhs.num;
	}

	int x() { return entry.x(); }
	int y() { return entry.y(); }
	int z() { return entry.z(); }

private:
	Eigen::Vector3i entry;
	unsigned long num;
	unsigned long const hashSize = 1993l;
	
	unsigned long hashing()
	{
		return (unsigned long(entry.x()) * 73856093l % hashSize) ^
			(unsigned long(entry.y()) * 19349663l % hashSize) ^
				(unsigned long(entry.z()) * 83492791l % hashSize);
	}

};

template <typename PrimitiveRef, typename PointType>
class SpatialHashing
{
public:
	typedef std::pair<PrimitiveRef, AABBox<PointType> > Node;

	SpatialHashing(PointType const & minCor,
		PointType const & maxCor,
		PointType const & cellSize
		) :
		m_minCor(minCor), m_maxCor(maxCor), m_cellSize(cellSize)
	{
		m_range.x() = (int)(std::ceil)((m_maxCor - m_minCor).x() / m_cellSize.x());
		m_range.y() = (int)(std::ceil)((m_maxCor - m_minCor).y() / m_cellSize.y());
		m_range.z() = (int)(std::ceil)((m_maxCor - m_minCor).z() / m_cellSize.z());
	}
	
	bool insert(PrimitiveRef && ref)
	{
		AABBox<PointType> box = AABBoxOf<PointType, PrimitiveRef>(ref);
		Entry min = m_vertexHashing(box.minCor());
		Entry max = m_vertexHashing(box.maxCor());
		if (min.x() < 0 || min.y() < 0 || min.z() < 0 || 
			max.x() > m_range.x() || max.y() > m_range.y() || max.z() > m_range.z())
				return false;
		for (int _i = min.x(); _i <= max.x(); ++_i) 
			for (int _j = min.y(); _j <= max.y(); ++_j)
			for (int _k = min.z(); _k <= max.z(); ++_k)
			{
				insertTable(Entry(_i, _j, _k), Node(ref, std::move(box)));
			}
		return true;
	}

	template <typename OtherPrimitiveRef>
	std::list<Node> candidate(OtherPrimitiveRef const & other) const
	{
		AABBox<PointType> box = AABBoxOf<PointType, OtherPrimitiveRef>(other);
		Entry min = m_vertexHashing(box.minCor());
		Entry max = m_vertexHashing(box.maxCor());
		std::list<Node> res;
		for (int _i = min.x(); _i <= max.x(); ++_i)
			for (int _j = min.y(); _j <= max.y(); ++_j)
				for (int _k = min.z(); _k <= max.z(); ++_k)
				{
					Entry pos(_i, _j, _k);
					if (m_table.find(pos) == m_table.end())
						continue;
					//std::cout << "insert size " << m_table.at(pos).size() << std::endl;
					res.insert(res.end(), m_table.at(pos).begin(), m_table.at(pos).end());
				}
		return std::move(res);
	}

	
private:
	std::map<Entry, std::list<Node> > m_table;

	PointType const m_minCor;
	PointType const m_maxCor;
	PointType const m_cellSize;
	Eigen::Vector3i m_range;

	std::function<Entry(PointType const &)> const m_vertexHashing = [this](PointType const & position) -> Entry
	{
		PointType relativePos = position - m_minCor;
		return Entry(
			(int)(std::ceil)(relativePos.x() / m_cellSize.x()),
			(int)(std::ceil)(relativePos.y() / m_cellSize.y()),
			(int)(std::ceil)(relativePos.z() / m_cellSize.z()));
	};

	void insertTable(Entry && pos, Node && node)
	{
		if (m_table.find(pos) == m_table.end())
		{
			m_table[pos] = std::list<Node>();
		}
		std::list<Node> & list = m_table[pos];
		list.push_back(node);
	}


};


#endif