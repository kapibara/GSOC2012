#ifndef DYNAMICTIMEWARPING_H
#define DYNAMICTIMEWARPING_H

#include <vector>
#include <opencv2/opencv.hpp>

class DynamicTimeWarping
{
public:
    enum {EUCLEDIAN = 0};
    typedef std::pair<int,int> IndexPair;

    DynamicTimeWarping();

    static void match(double &cost,
                      std::vector<IndexPair> &index,
                      const std::vector<double> &contour1,
                      const std::vector<double> &contour2,
                      int metric = EUCLEDIAN);

    static void partialMatch(double &cost,
                             std::vector<IndexPair> &index,
                             const std::vector<double> &contour1,
                             const std::vector<double> &contour2,
                             int metric = EUCLEDIAN){};

private:
    static void matchEuc(double &cost,
                         std::vector<IndexPair> &index,
                         const std::vector<double> &contour1,
                         const std::vector<double> &contour2);

    static inline double min(int &idx,double d1,double d2, double d3)
    {
        if(d1<d2){
            if(d3<d1){
                idx = 3;
                return d3;
            }else{
                idx = 1;
                return d1;
            }
        }
        else{
            if(d3<d2){
                idx = 3;
                return d3;
            }else{
                idx = 2;
                return d2;
            }
        }
    }

    static inline double distance(double d1, double d2)
    {
        return fabs(d2-d1);
    }

};

#endif // DYNAMICTIMEWARPING_H
