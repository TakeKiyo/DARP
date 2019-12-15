#pragma once

#include"InputData.hpp"
#include <vector>
//ルートの数は車両の数

class RouteList {
private:
    int VehicleNum;
    vector<vector<int> > Routelist;

public:
    int getVehicleNum();
	RouteList(int VehicleNum);
    void makeInitialRoute(int RequestSize);
    int getRouteSize(int number);
    int getRoute(int RouteNumber,int RouteOrder);
};