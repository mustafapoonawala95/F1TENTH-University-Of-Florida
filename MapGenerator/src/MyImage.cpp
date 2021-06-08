#include "io_tools.hpp"
#include "MyImage.hpp"
#include <iostream>

namespace ipb::hw6 {
    void image::justdoit(){
        std::cout << "Member function checking! \n";
    };
    //============================================================
    // ========= Defining setter and getter functions. ===========
    //============================================================
    int image::rows() const {
        return rows_;
    };

    void image::rows(int row){
        rows_ = row;
    };

    int image::cols() const {
        return cols_;
    };

    void image::cols(int col){
        cols_ = col;
    };

    int image::max_val() const {
        return max_val_;
    };

    void image::max_val(int maxv){
        max_val_ = maxv;
    };

    std::vector<int> image::data() const {
        std::vector<int> Data_int(data_.size());
        for(int i=0;i<data_.size();i++){
            Data_int[i] = data_[i];
            //std::cout << "The int value pushed into Data_int is: " << Data_int[i] << "\n";// And as read as int " << int(data_[i]) 
            //<< "\n";
            }
        return Data_int;
    };

    void image::data(std::vector<int> Data){
        data_.resize(Data.size());
        for(int i=0;i<Data.size();i++){
            //data_.push_back(uint8_t(Data[i]));
            data_[i] = Data[i];
            //std::cout << Data[i] << " Data[i] << The int value push read as is: " << data_[i] << " And as read as int " << int(data_[i]) 
            //<< "\n";
            }
        //std::cout << "The data[510] is: " << int(data_[510]) << "\n";
    };
    //=============================================================
    uint8_t& image::at(int r, int c){
        return data_[r*cols_ + c];
    };

    void image::CreateObstacle(){

        int r1, c1, r2, c2;
        std::cout << "Enter the zero index based row and column values of the top left corner of the obstacle and press enter.\n"; 
        std::cin >> r1 >> c1;
        std::cout << "Now enter the zero index based row and column values of the bottom right corner of the obstacle and press enter.\n";
        std::cin >> r2 >> c2;
        int dc = c2-c1;
        int dr = r2-r1;
        for (int i=0;i<=dr;i++){
            for(int j=0;j<=dc;j++){
                data_.at((r1+i)*cols_ + (c1+j)) = 5;
            }
        }
        //std::cout << c1x << " " << c1y << " " << c2x << " " << c2y << "\n";
    };
}