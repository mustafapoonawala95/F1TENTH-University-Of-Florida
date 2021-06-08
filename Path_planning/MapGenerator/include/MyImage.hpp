#ifndef HOMEWORK_6_HPP_
#define HOMEWORK_6_HPP_
#include <cstddef>
#include <string>
#include <vector>
//#include "io_tools.hpp"

namespace ipb::hw6 {
    class image{
        private:
            int rows_;
            int cols_;
            int max_val_;
            std::vector<uint8_t> data_;

        public:
            void justdoit();
            // Getter functions for rows,cols, and max_val
            int rows() const;
            void rows(int row);
            int cols() const;
            void cols(int col);
            int max_val() const;
            void max_val(int maxv);
            std::vector<int> data() const;
            void data(std::vector<int> Data);
            uint8_t& at(int r, int c);
	    void CreateObstacle();
            //std::vector<float> ComputeHistogram(int bins) const;
            //void DownScale(int scale);
            
    };

}
#endif  // HOMEWORK_6_HPP_
