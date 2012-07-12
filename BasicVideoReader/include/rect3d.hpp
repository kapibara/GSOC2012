#ifndef RECT3D_H
#define RECT3D_H

#include <opencv2/opencv.hpp>

#include <iostream>


namespace cv
{

template<class T>
struct Point3D_
{
    Point3D_()
    {
        isIni = false;
    }

    Point3D_(const T xin,const T yin, const T zin)
    {
        x = xin;
        y = yin;
        z = zin;
        isIni = true;
    }


    friend std::ostream& operator << (std::ostream &out, const Point3D_ &p){

        out << "[" << p.x << ";" << p.y << ";" << p.z << "]";

        return out;
    }

    T x,y,z;
    bool isIni;
};

template<class T>
struct Rect3D_
{
    Rect3D_()
    {
        isIni = false;
    }

    Rect3D_(const T xin,const T yin,const T zin,const T widthin,const T heightin,const T depthin)
    {
        x = xin;
        y = yin;
        z = zin;
        width = widthin;
        height = heightin;
        depth = depthin;
        isIni = true;
    }

    Rect3D_(Rect_<T> &rect,T zin,T depthin)
    {
        x = rect.x;
        y = rect.y;
        z = zin;
        width = rect.width;
        height = rect.height;
        depth = depthin;
        isIni = true;
    }

    Point3D_<T> center()
    {
        return Point3D_<T>(x + width/2,y + height/2, z + depth/2);
    }

    friend std::ostream& operator << (std::ostream &out, const Rect3D_ &rect){

        out << "[" << rect.x << ";" << rect.y << ";" << rect.z << ";"
            << rect.width << ";" << rect.height << ";" << rect.depth << "]";

        return out;
    }

    T x,y,z,width,height,depth;
    bool isIni;
};



typedef Rect3D_<int> Rect3D;
typedef Point3D_<int> Point3D;

}

#endif // RECT3D_H
