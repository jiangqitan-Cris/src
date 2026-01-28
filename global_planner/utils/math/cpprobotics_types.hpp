#ifndef _CPPROBOTICS_TYPES_H
#define _CPPROBOTICS_TYPES_H

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace cpprobotics{

using Vec_d=std::vector<double>;
using Poi_d=std::array<double, 2>;
using Vec_Poi=std::vector<Poi_d>;

};

#endif
