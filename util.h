#pragma once

#include <vector>

// Finds position of an element with given ID in given vector
// Returns -1 if element is not found
// ! Assumes elements have an m_ID member variable !
// ! Assumes elements are ordered on m_ID !
template <typename T>
int BinarySearch(std::vector<T> vec, int id) {
	unsigned min = 0;
	unsigned max = vec.size() - 1;
	while (min <= max) {
		unsigned mid = (min + max) / 2;
		if (vec[mid]->m_ID == id) return mid;
		if (vec[mid]->m_ID < id) min = mid+1;
		else max = mid-1;
	}
	return -1;
}