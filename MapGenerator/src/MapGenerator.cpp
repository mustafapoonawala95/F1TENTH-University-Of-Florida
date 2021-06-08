#include <iostream>
#include "MyImage.hpp"
#include <algorithm>
#include "io_tools.hpp"

int main(){
    std::cout << "Creating an image object of class image.\n";
    ipb::hw6::image TestImage;

    
    //==============================================================
    int rows;// = TestImage.rows();
    int cols;
    std::cout << "Enter the number of rows for the test map: \n";
    std::cin >> rows;
    std::cout << "Enter the number of columns for the test map: \n";
    std::cin >> cols;
    TestImage.rows(rows);
    TestImage.cols(cols);
    std::cout << "The rows now are: " << TestImage.rows() << "\n";
    std::cout << "The cols now are: " << TestImage.cols() << "\n";
    TestImage.max_val(255);
    std::vector<int> SomeData(rows*cols,250);
    TestImage.data(SomeData);
    bool create_obstacle = true;
    char answer;

    while(create_obstacle){
        std::cout << " Do you want to create an Obstacle (Y/N)?\n";
        std::cin >> answer;
        switch( answer )
        {
            case 'Y': case 'y': 
            create_obstacle = true;
            TestImage.CreateObstacle();
            break;

            case 'N': case 'n':
            create_obstacle = false;
            break;
        }
    }
    bool k = igg::io_tools::WriteToPgm(TestImage, "TestMap.pgm");
}

