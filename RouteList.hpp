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
    ~RouteList();
    void makeInitialRoute(int RequestSize);
    int getRouteSize(int number);
    vector<int>* getRoutePointerByIndex(int index);
    int getRouteListSize();
    int getRoute(int RouteNumber,int RouteOrder);
    void InnerRouteChange_requestSet(); //リクエストをセットでルート内 ランダム
    void InnerRouteChange_node(int customerSize); //ノードごとにルート内 ランダム
    void OuterRouteChange_random(int customerSize); // 他のルートに挿入 インデックスはランダム
    void InnerRouteChange_specified(int customerSize,int worst); //ペナルティの大きいノードを単体で違う場所に挿入
    void InnerOrderChange_requestset(int RouteIndex); //ルートのインデックスを指定してリクエストをセットでルート内
    void InnerOrderChange_node(int customersize,int RouteIndex); //ルートのインデックスを指定してノードごとにルート内 
    void OuterRouteChange_specified(int customerSize,int worstRouteIndex); // 一番ペナルティの大きいルートからリクエストを選んで他に挿入
    void OuterRouteChange_specified_double(int customerSize,int worstRouteIndex,int bestRouteIndex); //ペナルティが一番大きいものから小さいものに挿入
};