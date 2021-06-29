/*
 * 使用样条函数对离散的点进行平滑——未完成
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

using namespace std;

/*
 * 使用样条函数对离散点进行平滑
 */

class point2d{
public:
    double x;
    double y;
    double z;
    point2d():x(0), y(0), z(0){}
    point2d(double x_, double y_, double z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }
};

bool loadMap(string filename, vector<point2d>& path)
{
    path.clear();
    string fileFlag("");
    int pointNums(0);
    int x, y, z;
    ifstream file;
    file.open(filename);
    file >> fileFlag;
    if(fileFlag != "FID")
        return false;
    file >> pointNums;
    for(int i = 0; i < pointNums; i++)
    {
        file >> x >> y >> z;
        point2d tempPoint(x, y, z);
        path.push_back(tempPoint);
    }
    return true;
}

int main(int argc, char** argv)
{


    return 0;
}
