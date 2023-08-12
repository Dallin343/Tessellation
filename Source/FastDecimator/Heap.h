//
// Created by dallin on 6/27/23.
//

#ifndef TESSELLATION_HEAP_H
#define TESSELLATION_HEAP_H
#include "Types.h"

namespace FastDecimator {
    class MinCompare
    {
    public:
        bool operator()(const PairCost l, const PairCost r) const {
            return l.cost > r.cost;
        }
    };

    class EdgeCostHeap : public std::priority_queue<PairCost, std::vector<PairCost>, MinCompare>
    {
    private:
        bool _remove(const SM_vertex_descriptor& searchVd, bool& need_to_heap) {
            auto it = std::find_if(this->c.begin(), this->c.end(), [&](const PairCost& item) {
                return item.pair.first == searchVd || item.pair.second == searchVd;
            });

            if (it == this->c.end()) {
                return false;
            }
            if (it == this->c.begin()) {
                // deque the top element
                this->pop();
            }
            else {
                // remove element and re-heap
                this->c.erase(it);
                need_to_heap = true;
            }
            return true;
        }

        bool _remove(const SM_vertex_descriptor& searchVd1, const SM_vertex_descriptor& searchVd2) {
            auto it = std::find_if(this->c.begin(), this->c.end(), [&](const PairCost& item) {
                return (item.pair.first == searchVd1 && item.pair.second == searchVd2) ||
                       (item.pair.first == searchVd2 && item.pair.second == searchVd1);
            });

            if (it == this->c.end()) {
                return false;
            }
            else if (it == this->c.begin()) {
                // deque the top element
                this->pop();
            }
            else {
                // remove element and re-heap
                this->c.erase(it);
                return true;
            }
            return false;
        }
    public:

        bool remove(const SM_vertex_descriptor& searchVd, bool remove_all) {
            bool need_to_heap = false;
            if (remove_all) {
                while (this->_remove(searchVd, need_to_heap)) {}
            }
            else {
                this->_remove(searchVd, need_to_heap);
            }

            if (need_to_heap) {
                std::make_heap(this->c.begin(), this->c.end(), this->comp);
            }
            return true;
        }

        bool remove(const SM_vertex_descriptor& searchVd1, const SM_vertex_descriptor& searchVd2) {
            if (this->_remove(searchVd1, searchVd2)) {
                std::make_heap(this->c.begin(), this->c.end(), this->comp);
            }
            return true;
        }

        bool remove(const std::vector<SM_vertex_descriptor>& searchVds) {
            bool need_to_heap = false;
            for (const auto& searchVd : searchVds) {
                this->_remove(searchVd, need_to_heap);
            }

            if (need_to_heap) {
                std::make_heap(this->c.begin(), this->c.end(), this->comp);
            }
            return true;
        }

        bool remove(const std::vector<std::pair<SM_vertex_descriptor, SM_vertex_descriptor>>& searchVds) {
            bool need_to_heap = false;
            for (const auto& [vd1, vd2] : searchVds) {
                if (this->_remove(vd1, vd2)) {
                    need_to_heap = true;
                }
            }

            if (need_to_heap) {
                std::make_heap(this->c.begin(), this->c.end(), this->comp);
            }
            return true;
        }
    };
}

#endif //TESSELLATION_HEAP_H
