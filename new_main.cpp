#include <iostream>
#include <cmath>
#include <tuple>
#include <utility>
#include <algorithm>
#include "InputData.hpp"
#include "RouteList.hpp"
#include "Location.hpp"
#include "Cost.hpp"
#include "myfunction.hpp"
#include <gurobi_c++.h>
#include "cpu_time.c"
#include <numeric>
using namespace std;



int main(int argc, char *argv[]){
    int i,j;
    string tmp;
    int worstPosition;
    double tmpPenalty; 
    // 入力を受け取り
    InputData inputdata;
    string inputfile = argv[1];
    inputdata.setInputData(inputfile = argv[1]);
    cout << "車両数:" << inputdata.getVehicleNum() << endl;
    cout << "リクエスト数:" << inputdata.getRequestSize() << endl;
    cout << "ルートの最大長さ:" << inputdata.getMaximumRouteDuration() << endl;
    cout << "車両の容量:" << inputdata.getVehicleCapacity() << endl;

    // n:カスタマーサイズ 
    // m:車両数
    int n=inputdata.getRequestSize()/2;
    int m=inputdata.getVehicleNum();

    // 2店間の距離と時間を計算
    Location* loc1;
    Location* loc2;
    Cost cost(inputdata.getRequestSize());
    for (i=0;i<=inputdata.getRequestSize();i++){
        for (j=0;j<=inputdata.getRequestSize();j++){
            // コストを計算
            loc1 = inputdata.getLocationPointer(i);
            loc2 = inputdata.getLocationPointer(j);
            cost.setCost(i,loc1->getLat(),loc1->getLng(),j,loc2->getLat(),loc2->getLng());
        }
    }
    // 乗車時間のペナルティ関数をここでいれる 
    // これおそらくいらない
    for(int i=1;i<=n;i++){
        vector<double> vec{ 0.0, 0.0, cost.getCost(i,i+n),0,cost.getCost(i,i+n)+1.0,1};
        inputdata.setRideTimePenalty(i,vec);
        vec.clear();
    }

    // RouteListクラス:複数のルートをまとめて保持するクラス 
    RouteList routelist(m);
    routelist.makeInitialRoute(inputdata.getRequestSize());

    // ペナルティ
    double TotalPenalty=0;

    // ルートの総距離と時間枠ペナルティと乗客数のペナルティ比
    double ALPHA = 1.0; //ルートの距離
    double BETA = 500.0; //時間枠ペナ
    double GAMMA = 1.0; //乗客数ペナ

    // イテレーション回数
    // int COUNT_MAX = 100000;

    // 最適解
    double BestTotalPenalty;
    double BestRouteDistance;
    double BestPenalty;

    int ImprovedNumber=0;
    
    // クラスをまたいでenvとmodelを保持できなさそうだからmainの中で宣言する
    try{

        GRBEnv env=GRBEnv(true);
        env.set("LogFile", "mip1.log");
        env.start();
        vector<GRBModel> modelList;
        for(int i=0;i<m;i++){
            GRBModel modeli = GRBModel(env);
            modelList.push_back(modeli);
        }

        GRBModel model = GRBModel(env);
        GRBVar DepartureTime[2*n+1];
        GRBVar DepotTime[2*m];
        GRBVar RideTime[n+1];
        GRBVar DepartureTimePenalty[2*n+1];
        GRBVar RideTimePenalty[n+1];


        GRBVar DepartureTimeArr[m][2*n+1];
        GRBVar DepotTimeArr[m][2*m];
        GRBVar RideTimeArr[m][n+1];
        GRBVar DepartureTimePenaltyArr[m][2*n+1];
        GRBVar RideTimePenaltyArr[m][n+1];

        // 各ノードの出発時刻の変数を追加
        for(i=0;i<=2*n;i++){
            tmp = "t_"+to_string(i); //ノードiの出発時刻をt_iとする
            for (j=0;j<m;j++){
                DepartureTimeArr[j][i] = modelList[j].addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            }
            DepartureTime[i] = model.addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
        }
        for(i=0;i<=2*n;i++){
            tmp = "p_t_"+to_string(i); //ノードiの出発時刻をt_iとする
            for (j=0;j<m;j++){
                DepartureTimePenaltyArr[j][i] = modelList[j].addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            }
            DepartureTimePenalty[i] = model.addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
        }

        // デポの出発と帰る時刻
        for(i=0;i<m;i++){
            tmp = "ds_"+to_string(i);
            DepotTime[i] = model.addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            for (j=0;j<m;j++){
                DepotTimeArr[j][i] = modelList[j].addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            }
        }

        for(i=m;i<2*m;i++){
            tmp = "de_"+to_string(i);
            for(j=0;j<m;j++){
                DepotTimeArr[j][i] = modelList[j].addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            }
            DepotTime[i] = model.addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
        }

        //乗車時間
        for (i=0;i<=n;i++){
            tmp = "rt"+to_string(i);
            RideTime[i] = model.addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            for(j=0;j<m;j++){
                RideTimeArr[j][i] = modelList[j].addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            }
        }
        for (i=0;i<=n;i++){
            tmp = "p_rt"+to_string(i);
            RideTimePenalty[i] = model.addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            for (j=0;j<m;j++){
                RideTimePenaltyArr[j][i] = modelList[j].addVar(0.0,(double)inputdata.getLatestArrivalDepotTime(),0.0,GRB_CONTINUOUS,tmp);
            }
        }
        // ここまで変数とペナルティ変数を定義


        // 出発時刻のペナルティを取得 区分数は4
        double departureX[2*n+1][4];
        double departureY[2*n+1][4];
        for (i=1;i<=n;i++){ //pickup
            for(j=0;j<4;j++){
                departureX[i][j] = inputdata.getPickupPointer(i)->getPickupPenaltyXValue(j);
                departureY[i][j] =  inputdata.getPickupPointer(i)->getPickupPenaltyYValue(j);
            }
        }
        for(i=n+1;i<=2*n;i++){ //dropoff
            for(j=0;j<4;j++){
                departureX[i][j] = inputdata.getDropoffPointer(i-n)->getDropoffPenaltyXValue(j);
                departureY[i][j] = inputdata.getDropoffPointer(i-n)->getDropoffPenaltyYValue(j);
            }
        }

        // 出発時刻に関する区分線形関数を追加
        for (j=0;j<m;j++){
            for(i=1;i<=2*n;i++){
                modelList[j].addGenConstrPWL(DepartureTimeArr[j][i],DepartureTimePenaltyArr[j][i],4,departureX[i],departureY[i],"pwl");
            }
        }

        for(i=1;i<=2*n;i++){
            model.addGenConstrPWL(DepartureTime[i],DepartureTimePenalty[i],4,departureX[i],departureY[i],"pwl");
        }
        
        // 目的関数を追加
        GRBLinExpr objection;
        for(i=1;i<=2*n;i++){
            objection += DepartureTimePenalty[i];
        }
        for(i=1;i<=n;i++){
            objection += RideTimePenalty[i];
        }
        model.setObjective(objection, GRB_MINIMIZE);


        GRBLinExpr objectionArr[m];
        for(j=0;j<m;j++){
            for(i=1;i<=2*n;i++){
                objectionArr[j] += DepartureTimePenaltyArr[j][i];
            }
            for(i=1;i<=n;i++){
                objectionArr[j] += RideTimePenaltyArr[j][i];
            }
        }
        for (j=0;j<m;j++){
            modelList[j].setObjective(objectionArr[j],GRB_MINIMIZE);
        }

        // 乗車時間の定義 rt_i = t_i+n -t_i （制約として追加)
        for (i=1;i<=n;i++){
            tmp = "RideConst"+to_string(i);
            model.addConstr(RideTime[i] == DepartureTime[i+n]-DepartureTime[i]-10,tmp);
            for (j=0;j<m;j++){
                modelList[j].addConstr(RideTimeArr[j][i] == DepartureTimeArr[j][i+n]-DepartureTimeArr[j][i]-10,tmp);
            }
        }

        // 乗車時間の区分線形関数を取得 区分数は3つ
        double ridetimeX[n+1][3];
        double ridetimeY[n+1][3];
        for(i=1;i<=n;i++){
            ridetimeX[i][0] = 0;
            // ridetimeX[i][1] = cost.getCost(i,i+n);
            // ridetimeX[i][2] = cost.getCost(i,i+n)+1;
            ridetimeX[i][1] = 90;
            ridetimeX[i][2] = 91;
            ridetimeY[i][0] = 0;
            ridetimeY[i][1] = 0;
            ridetimeY[i][2] = 1;
        }

        // 乗車時間の区分線形関数を追加  区分数は3つ
        for(i=1;i<=n;i++){
            model.addGenConstrPWL(RideTime[i],RideTimePenalty[i],3,ridetimeX[i],ridetimeY[i],"rtpwl");
        }
        for (j=0;j<m;j++){
            for (i=1;i<=n;i++){
                modelList[j].addGenConstrPWL(RideTimeArr[j][i],RideTimePenaltyArr[j][i],3,ridetimeX[i],ridetimeY[i],"rtpwl");
            }
        }


        // ここまでの設定は最適化の中で不変
        // ここからはイテレーションごとに変化する設定

        // ここでルートを受け取って、ルートの順番の制約を追加する
        // ルートの長さもここで計算する
        double RouteDistance=0; //RouteDistanceはイテレーション毎に0で初期化
        vector<GRBConstr> RouteOrderConstr;
        GRBTempConstr *tempconstr;
        tempconstr = new GRBTempConstr;
        string constrname; //制約の名前付け これは重要じゃない
        // double tmp_double;
        for(i=0;i<routelist.getRouteListSize();i++){
            for(j=1;j<routelist.getRouteSize(i)-2;j++){ 
                // 1から始めるのは最初のデポは別で設定するから 
                // getRouteSize(i)-2なのはj→j+1を考えてるから
                RouteDistance += cost.getCost(routelist.getRoute(i,j),routelist.getRoute(i,j+1));
                // 論文の8の式
                *tempconstr = DepartureTime[routelist.getRoute(i,j)] + 10.0 +  cost.getCost(routelist.getRoute(i,j),routelist.getRoute(i,j+1)) <= DepartureTime[routelist.getRoute(i,j+1)];
                constrname = "constr"+to_string(i)+ "_" + to_string(j);
                RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
            }
        }
        // デポの時刻DepotTimeとの制約も追加
        for(i=0;i<routelist.getRouteListSize();i++){
            RouteDistance += cost.getCost(0,routelist.getRoute(i,1)); //i番目の車両の1番目
            RouteDistance += cost.getCost(routelist.getRoute(i,routelist.getRouteSize(i)-2),0);//i番目の車両のデポを除く最後
            // デポと1番目の制約
            *tempconstr = DepotTime[i] +  cost.getCost(0,routelist.getRoute(i,1)) <= DepartureTime[routelist.getRoute(i,1)];
            constrname = to_string(i) + "constr_depot_1";
            RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
            // 最後とデポの制約
            *tempconstr = DepartureTime[routelist.getRoute(i,routelist.getRouteSize(i)-2)] + 10.0 +  cost.getCost(routelist.getRoute(i,routelist.getRouteSize(i)-2),0) == DepotTime[i+m];
            constrname = to_string(i) + "constr_last_depot";
            RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
        }

        // MaximumRouteDurationを設定
        for(i=0;i<m;i++){
            tmp = "MaximumRouteDutaion"+to_string(i);
            model.addConstr(480 >= DepotTime[i+m]-DepotTime[i],tmp);
        }



        // ここからルートの制約を車両数分持つモデル
        vector<double> RouteDistanceVec(m,0); //RouteDistanceはイテレーション毎に0で初期化
        vector<double> PenaltyVec(m,0);
        vector<vector<GRBConstr> > RouteOrderConstrVec;
        for(i=0;i<m;i++){
            vector<GRBConstr> tmp;
            RouteOrderConstrVec.push_back(tmp);
        }
        // GRBTempConstr *tempconstr;
        // tempconstr = new GRBTempConstr;
        // string constrname; //制約の名前付け これは重要じゃない
        for (i=0;i<routelist.getRouteListSize();i++){
            for(j=1;j<routelist.getRouteSize(i)-2;j++){
                RouteDistanceVec[i] += cost.getCost(routelist.getRoute(i,j),routelist.getRoute(i,j+1));
                *tempconstr = DepartureTimeArr[i][routelist.getRoute(i,j)] + 10.0 +  cost.getCost(routelist.getRoute(i,j),routelist.getRoute(i,j+1)) <= DepartureTimeArr[i][routelist.getRoute(i,j+1)];
                constrname = "constr"+to_string(i)+ "_" + to_string(j);
                RouteOrderConstrVec[i].push_back(modelList[i].addConstr(*tempconstr,constrname));
            }
        }
        // デポの時刻DepotTimeとの制約も追加
        for(i=0;i<routelist.getRouteListSize();i++){
            RouteDistanceVec[i] += cost.getCost(0,routelist.getRoute(i,1)); //i番目の車両の1番目
            RouteDistanceVec[i] += cost.getCost(routelist.getRoute(i,routelist.getRouteSize(i)-2),0);//i番目の車両のデポを除く最後
            // デポと1番目の制約
            *tempconstr = DepotTimeArr[i][i] +  cost.getCost(0,routelist.getRoute(i,1)) <= DepartureTimeArr[i][routelist.getRoute(i,1)];
            constrname = to_string(i) + "constr_depot_1";
            RouteOrderConstrVec[i].push_back(modelList[i].addConstr(*tempconstr,constrname));
            // 最後とデポの制約
            *tempconstr = DepartureTimeArr[i][routelist.getRoute(i,routelist.getRouteSize(i)-2)] + 10.0 +  cost.getCost(routelist.getRoute(i,routelist.getRouteSize(i)-2),0) == DepotTimeArr[i][i+m];
            constrname = to_string(i) + "constr_last_depot";
            RouteOrderConstrVec[i].push_back(modelList[i].addConstr(*tempconstr,constrname));
        }


        for (i=0;i<m;i++){
            modelList[i].optimize();
            PenaltyVec[i] = modelList[i].get(GRB_DoubleAttr_ObjVal);
        }
        RouteDistance = accumulate(RouteDistanceVec.begin(),RouteDistanceVec.end(),0.0);
        double Penalty = accumulate(PenaltyVec.begin(),PenaltyVec.end(),0.0);


        
        

        

        // 計算開始時間
        double starttime = cpu_time();
        // optimize
        model.set(GRB_IntParam_OutputFlag, 0); //ログの出力をoff
        model.optimize();

        cout << "RouteDistance:" << RouteDistance << endl;
        cout << "penalty: " << Penalty << endl;

        TotalPenalty = ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal);
        BestTotalPenalty = TotalPenalty;
        BestRouteDistance = RouteDistance;
        BestPenalty = model.get(GRB_DoubleAttr_ObjVal);
        // ここでルートの順番の制約をremove
        for(i=0;i<RouteOrderConstr.size();i++){
            model.remove(RouteOrderConstr[i]);
        }
        //RouteOrderConstrとtempconstrのメモリ解放
        delete tempconstr;
        vector<GRBConstr>().swap(RouteOrderConstr);

        // worstPosition = 1; 
        // tmpPenalty = 0;
        // for(i=1;i<=2*n;i++){
        //     if (DepartureTimePenalty[i].get(GRB_DoubleAttr_X) > tmpPenalty){
        //         worstPosition = i;
        //         tmpPenalty = DepartureTimePenalty[i].get(GRB_DoubleAttr_X);
        //     }
        // }
        for(i=0;i<routelist.getRouteListSize();i++){
            for(j=0;j<routelist.getRouteSize(i);j++){
                cout << routelist.getRoute(i,j) << " ";
            }
            cout << endl;
        }


        // **************************イテレーション開始************************************
        cout << "しょきかいの近傍スタート" << endl;
        RouteList bestroutelist(m);
        bool recent_changed_flag[m];
        for (int tmp=0;tmp<m;tmp++) recent_changed_flag[tmp] = true;
        double QP;
        int search_count = 0;
        double PenaltyArray[m];
        // /*
        // 初期解のルート内近傍
        for (int RouteIndex=0;RouteIndex<m;RouteIndex++){ //車両ごと
            cout << "RouteIndex:" <<RouteIndex << " " << pow((routelist.getRouteSize(RouteIndex)-2),2)*2 << endl;
            for (int neighborhood=0;neighborhood<pow((routelist.getRouteSize(RouteIndex)-2),2)*2;neighborhood++){ //近傍サイズを探索
                RouteList *TmpRouteList;
                TmpRouteList = new RouteList(m); //メモリの確保
                GRBTempConstr *tempconstr;
                tempconstr = new GRBTempConstr;
                *TmpRouteList = routelist;
                TmpRouteList->InnerOrderChange_node(n,RouteIndex); //ノードごとにルート内

                // ルートの制約を追加
                // ここでdistanceは計算しちゃう
                QP=0;
                RouteDistance=0;
                for(i=0;i<TmpRouteList->getRouteListSize();i++){
                    int current_person=0;
                    for(j=1;j<TmpRouteList->getRouteSize(i)-2;j++){
                        if (TmpRouteList->getRoute(i,j)<=n){
                            current_person+=1;
                        }else{
                            current_person -= 1;
                        }
                        if (current_person > 6){
                            QP += current_person-6;
                        }
                        RouteDistance += cost.getCost(TmpRouteList->getRoute(i,j),TmpRouteList->getRoute(i,j+1));
                        // 論文8の式
                        *tempconstr = DepartureTime[TmpRouteList->getRoute(i,j)] + 10.0 + cost.getCost(TmpRouteList->getRoute(i,j),TmpRouteList->getRoute(i,j+1)) <= DepartureTime[TmpRouteList->getRoute(i,j+1)];
                        constrname = "constr"+to_string(i)+ "_" + to_string(j);
                        RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                    }
                }
                // デポの時刻DepotTimeとの制約も追加
                for(i=0;i<TmpRouteList->getRouteListSize();i++){
                    RouteDistance += cost.getCost(0,TmpRouteList->getRoute(i,1));
                    RouteDistance += cost.getCost(TmpRouteList->getRouteSize(i)-2,0);
                    // デポと1番目の制約
                    *tempconstr = DepotTime[i] + cost.getCost(0,TmpRouteList->getRoute(i,1)) <= DepartureTime[TmpRouteList->getRoute(i,1)];
                    constrname = to_string(i) + "constr_depot_1";
                    RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                    // 最後とデポの制約
                    *tempconstr = DepartureTime[TmpRouteList->getRoute(i,TmpRouteList->getRouteSize(i)-2)] + 10.0 + cost.getCost(TmpRouteList->getRoute(i,TmpRouteList->getRouteSize(i)-2),0) == DepotTime[i+m];
                    constrname = to_string(i) + "constr_last_depot";
                    RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                }  //このかっこがないと値は上がる
                // LP実行(optimize)
                model.optimize();
                search_count++;
                // ペナルティを計算して比較
                // 良い解の場合 
                if (ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal) + GAMMA*QP < TotalPenalty){
                    routelist = *TmpRouteList;
                    if(QP==0) {
                        bestroutelist = *TmpRouteList;
                        TotalPenalty = ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal);
                        cout << "改善 " << TotalPenalty << " count:" << search_count  <<endl;
                        BestTotalPenalty = TotalPenalty;
                        BestRouteDistance = RouteDistance;
                        BestPenalty = model.get(GRB_DoubleAttr_ObjVal);
                        for(int TmpRouteNum=0;TmpRouteNum<m;TmpRouteNum++){
                            double tmpPenalty = 0;
                            for(int order=1; order<routelist.getRouteSize(TmpRouteNum)-1;order++){
                                tmpPenalty += DepartureTimePenalty[routelist.getRoute(TmpRouteNum,order)].get(GRB_DoubleAttr_X);
                                if (routelist.getRoute(TmpRouteNum,order)<=n) tmpPenalty+= RideTimePenalty[routelist.getRoute(TmpRouteNum,order)].get(GRB_DoubleAttr_X);
                            }
                            PenaltyArray[TmpRouteNum] = tmpPenalty;
                        } 
                        worstPosition = 0; 
                        tmpPenalty = 0;
                        for(i=1;i<=2*n;i++){
                            if (DepartureTimePenalty[i].get(GRB_DoubleAttr_X) > tmpPenalty){
                                worstPosition = i;
                                tmpPenalty = DepartureTimePenalty[i].get(GRB_DoubleAttr_X);
                            }
                        }
                    }
                }
                // 悪い解ならなにもしない
                // ルートの制約をremove
                for(i=0;i<RouteOrderConstr.size();i++){
                    model.remove(RouteOrderConstr[i]);
                }

                //RouteOrderConstrとtemoconstrのメモリ解放
                vector<GRBConstr>().swap(RouteOrderConstr);
                delete tempconstr;

                //TmpRouteListクラスのメモリ解放
                delete TmpRouteList; 
            // } 
            }
            cout << "イテレーション回数:" << search_count << endl;
            for(int jkl=0;jkl<m;jkl++){
                cout << PenaltyArray[jkl] << " ";
            }
            cout << endl;
            cout << "----------------------------------" << endl;
        }
        // */
        cout << "初期解のルート間おわり " << search_count << endl;
        for(i=0;i<routelist.getRouteListSize();i++){
            for(j=0;j<routelist.getRouteSize(i);j++){
                cout << routelist.getRoute(i,j) << " ";
            }
            cout << endl;
        }

        double TmpTotalPenalty,TmpRouteDistance,TmpBestPenalty;
        tuple<int, int,int,int> TmpTuple_for_random;
        tuple<int,int,int,int,int,int> TmpTuple_for_swap;
        int beforeindex,afterindex;
        RouteList OuterRoutelist(m);
        int inter_count = 0;

        int total_count = 0;

        int iteration_count = 0;


        vector<pair<int,int> > NeighborList; //1番目が顧客の番号、2番目が車両の番号
        for (int i=1;i<=n;i++){
            for (int j=0;j<m;j++){
                NeighborList.push_back(make_pair(i,j));
            }
        }
        cout << "近傍リストのサイズ_挿入:" << NeighborList.size() << endl;
        random_shuffle(NeighborList.begin(),NeighborList.end()); //近傍リストをシャッフル

         // 交換
        vector<pair<int,int> > SwapList;
        for (int i=1;i<=n;i++){
            for (int j=1;j<=n;j++){
                if (i != j){
                    SwapList.push_back(make_pair(i,j));
                }
            }
        }
        random_shuffle(SwapList.begin(),SwapList.end());
        cout << "近傍リストのサイズ:" << SwapList.size() << endl;


        int NeighrListCount = 0;
        tuple<int,int> Tuple_swap;
        int SwapCount = 0;

        // /*
        // 挿入
        while(NeighrListCount<NeighborList.size()){
            cout << NeighborList[NeighrListCount].first << "  " << NeighborList[NeighrListCount].second <<"を挿入" << endl;
            cout << "回数:" << NeighrListCount << endl;
            TmpTotalPenalty = 100000000000000000.0;
            OuterRoutelist = routelist;
            // ルート間の挿入
            int first,second;
            first = NeighborList[NeighrListCount].first;
            second = NeighborList[NeighrListCount].first+n;
            beforeindex = OuterRoutelist.Outer_Relocate(n,m,NeighborList[NeighrListCount].first);
            afterindex = NeighborList[NeighrListCount].second;
            NeighrListCount += 1;
            if (beforeindex == afterindex) {
                cout << "beforeとafterのindexが同じ" << endl;
                continue;
            }
            // afterindexに2つの頂点を挿入
            int aftersize=OuterRoutelist.getRouteSize(afterindex);

            bool isBetter= false;
            for(int f=1;f<aftersize;f++){
                    for (int s=f+1;s<=aftersize;s++){
                        RouteList *TmpRouteList;
                        TmpRouteList = new RouteList(m); //メモリの確保
                        GRBTempConstr *tempconstr;
                        tempconstr = new GRBTempConstr;
                        *TmpRouteList = OuterRoutelist;
                        TmpRouteList->insertRoute(afterindex,f,first);
                        TmpRouteList->insertRoute(afterindex,s,second);
                        
                        // ルートの制約を追加
                        // ここでdistanceは計算しちゃう
                        QP=0;
                        RouteDistance=0;
                        for(i=0;i<TmpRouteList->getRouteListSize();i++){
                            int current_person=0;
                            for(j=1;j<TmpRouteList->getRouteSize(i)-2;j++){
                                if (TmpRouteList->getRoute(i,j)<=n){
                                    current_person+=1;
                                }else{
                                    current_person -= 1;
                                }
                                if (current_person > 6){
                                    QP += current_person-6;
                                }
                                RouteDistance += cost.getCost(TmpRouteList->getRoute(i,j),TmpRouteList->getRoute(i,j+1));
                                // 論文8の式
                                *tempconstr = DepartureTime[TmpRouteList->getRoute(i,j)] + 10.0 + cost.getCost(TmpRouteList->getRoute(i,j),TmpRouteList->getRoute(i,j+1)) <= DepartureTime[TmpRouteList->getRoute(i,j+1)];
                                constrname = "constr"+to_string(i)+ "_" + to_string(j);
                                RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                            }
                        }
                        // デポの時刻DepotTimeとの制約も追加
                        for(i=0;i<TmpRouteList->getRouteListSize();i++){
                            RouteDistance += cost.getCost(0,TmpRouteList->getRoute(i,1));
                            RouteDistance += cost.getCost(TmpRouteList->getRouteSize(i)-2,0);
                            // デポと1番目の制約
                            *tempconstr = DepotTime[i] + cost.getCost(0,TmpRouteList->getRoute(i,1)) <= DepartureTime[TmpRouteList->getRoute(i,1)];
                            constrname = to_string(i) + "constr_depot_1";
                            RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                            // 最後とデポの制約
                            *tempconstr = DepartureTime[TmpRouteList->getRoute(i,TmpRouteList->getRouteSize(i)-2)] + 10.0 + cost.getCost(TmpRouteList->getRoute(i,TmpRouteList->getRouteSize(i)-2),0) == DepotTime[i+m];
                            constrname = to_string(i) + "constr_last_depot";
                            RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                        }  
                        // LP実行(optimize)
                        model.optimize();
                        search_count++;
                        try {
                            // ペナルティを計算して比較
                            // 良い解の場合 
                            if (ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal) + GAMMA*QP < TmpTotalPenalty){
                                if (QP==0){
                                    routelist = *TmpRouteList;
                                    TmpTotalPenalty = ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal);
                                    cout << "改善 " << TmpTotalPenalty << " distance:" << RouteDistance << " count:" << search_count  <<endl;
                                    // BestTotalPenalty = TmpTotalPenalty;
                                    TmpRouteDistance = RouteDistance;
                                    TmpBestPenalty = model.get(GRB_DoubleAttr_ObjVal);
                                    // if (TmpTotalPenalty < BestTotalPenalty){
                                    //     isBetter = true;
                                    // }
                                }
                            }
                        }catch(GRBException e){
                            cout << "no solution" << endl;
                            search_count--;
                        }
                        // 悪い解ならなにもしない
                        // ルートの制約をremove
                        for(i=0;i<RouteOrderConstr.size();i++){
                            model.remove(RouteOrderConstr[i]);
                        }
                        //RouteOrderConstrとtemoconstrのメモリ解放
                        vector<GRBConstr>().swap(RouteOrderConstr);
                        delete tempconstr;
                        //TmpRouteListクラスのメモリ解放
                        delete TmpRouteList; 
                        if (isBetter) {
                            cout << "途中でいい解を発見" << endl;
                            break;
                        }
                    }
                    if (isBetter) {
                        cout << "途中でいい解を発見" << endl;
                        break;
                    }
            }
            // 一番いい位置に挿入して、それが既存より良かったら移動
            if (TmpTotalPenalty < BestTotalPenalty){
                ImprovedNumber += 1;
                cout << "解を移動" << endl;
                bestroutelist = routelist;
                BestTotalPenalty = TmpTotalPenalty;
                BestRouteDistance = TmpRouteDistance;
                BestPenalty =  TmpBestPenalty;
                for(i=0;i<bestroutelist.getRouteListSize();i++){
                    for(j=0;j<bestroutelist.getRouteSize(i);j++){
                        cout << bestroutelist.getRoute(i,j) << " ";
                    }
                    cout << endl;
                    }
            } else{
                    cout << "よくない" << endl;
                    routelist = bestroutelist;
            }

        }
        // */
        // 挿入ここまで

        /*
        // ここから交換
        while(SwapCount < SwapList.size()){
            cout <<  "交換 " << SwapCount << endl; 
            TmpTotalPenalty = 100000000000000000.0;
            OuterRoutelist = routelist; 
            Tuple_swap = OuterRoutelist.Outer_Swap(n,m,SwapList[SwapCount].first,SwapList[SwapCount].second);
            beforeindex = get<0>(Tuple_swap);
            afterindex = get<1>(Tuple_swap);
            int before_first,before_second,after_first,after_second;
            before_first = SwapList[SwapCount].first;
            before_second = SwapList[SwapCount].first + n;
            after_first = SwapList[SwapCount].second;
            after_second = SwapList[SwapCount].second + n;
            SwapCount++;
            if (beforeindex == afterindex){
                cout << "beforeとafterのindexが同じ" << endl;
                continue;
            }
            // beforeindexを先に改善
            int beforesize = OuterRoutelist.getRouteSize(beforeindex);
            RouteList TmpGoodRoute(m);
            for (int f=1;f<beforesize;f++){
                    for (int s=f+1;s<=beforesize;s++){
                        RouteList *TmpRouteList;
                        TmpRouteList = new RouteList(m); //メモリの確保
                        GRBTempConstr *tempconstr;
                        tempconstr = new GRBTempConstr;
                        *TmpRouteList = OuterRoutelist;
                        TmpRouteList->insertRoute(beforeindex,f,after_first);
                        TmpRouteList->insertRoute(beforeindex,s,after_second);
                        // ルートの制約を追加
                        // ここでdistanceは計算しちゃう
                        QP=0;
                        RouteDistance=0;
                        for(i=0;i<TmpRouteList->getRouteListSize();i++){
                            int current_person=0;
                            for(j=1;j<TmpRouteList->getRouteSize(i)-2;j++){
                                if (TmpRouteList->getRoute(i,j)<=n){
                                    current_person+=1;
                                }else{
                                    current_person -= 1;
                                }
                                if (current_person > 6){
                                    QP += current_person-6;
                                }
                                RouteDistance += cost.getCost(TmpRouteList->getRoute(i,j),TmpRouteList->getRoute(i,j+1));
                                // 論文8の式
                                *tempconstr = DepartureTime[TmpRouteList->getRoute(i,j)] + 10.0 + cost.getCost(TmpRouteList->getRoute(i,j),TmpRouteList->getRoute(i,j+1)) <= DepartureTime[TmpRouteList->getRoute(i,j+1)];
                                constrname = "constr"+to_string(i)+ "_" + to_string(j);
                                RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                            }
                        }
                        // デポの時刻DepotTimeとの制約も追加
                        for(i=0;i<TmpRouteList->getRouteListSize();i++){
                            RouteDistance += cost.getCost(0,TmpRouteList->getRoute(i,1));
                            RouteDistance += cost.getCost(TmpRouteList->getRouteSize(i)-2,0);
                            // デポと1番目の制約
                            *tempconstr = DepotTime[i] + cost.getCost(0,TmpRouteList->getRoute(i,1)) <= DepartureTime[TmpRouteList->getRoute(i,1)];
                            constrname = to_string(i) + "constr_depot_1";
                            RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                            // 最後とデポの制約
                            *tempconstr = DepartureTime[TmpRouteList->getRoute(i,TmpRouteList->getRouteSize(i)-2)] + 10.0 + cost.getCost(TmpRouteList->getRoute(i,TmpRouteList->getRouteSize(i)-2),0) == DepotTime[i+m];
                            constrname = to_string(i) + "constr_last_depot";
                            RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                        }  
                        // LP実行(optimize)
                        model.optimize();
                        search_count++;
                        if (ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal) + GAMMA*QP < TmpTotalPenalty){
                            if (QP==0){
                                TmpGoodRoute = *TmpRouteList;
                                TmpTotalPenalty = ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal);
                                // cout << "改善 " << TmpTotalPenalty << " fとs:"  << f << " " << s <<endl;
                            }
                        }
                        // 悪い解ならなにもしない
                        // ルートの制約をremove
                        for(i=0;i<RouteOrderConstr.size();i++){
                            model.remove(RouteOrderConstr[i]);
                        }
                        //RouteOrderConstrとtemoconstrのメモリ解放
                        vector<GRBConstr>().swap(RouteOrderConstr);
                        delete tempconstr;
                        //TmpRouteListクラスのメモリ解放
                        delete TmpRouteList; 
                    }
            }
            TmpTotalPenalty = 1000000000000.0;
            // afterindexを改善
            int aftersize = OuterRoutelist.getRouteSize(afterindex);
            for (int f=1;f<aftersize;f++){
                    for (int s=f+1;s<=aftersize;s++){
                        RouteList *TmpRouteList;
                        TmpRouteList = new RouteList(m); //メモリの確保
                        GRBTempConstr *tempconstr;
                        tempconstr = new GRBTempConstr;
                        *TmpRouteList = TmpGoodRoute;
                        TmpRouteList->insertRoute(afterindex,f,before_first);
                        TmpRouteList->insertRoute(afterindex,s,before_second);
                        // ルートの制約を追加
                        // ここでdistanceは計算しちゃう
                        QP=0;
                        RouteDistance=0;
                        for(i=0;i<TmpRouteList->getRouteListSize();i++){
                            int current_person=0;
                            for(j=1;j<TmpRouteList->getRouteSize(i)-2;j++){
                                if (TmpRouteList->getRoute(i,j)<=n){
                                    current_person+=1;
                                }else{
                                    current_person -= 1;
                                }
                                if (current_person > 6){
                                    QP += current_person-6;
                                }
                                RouteDistance += cost.getCost(TmpRouteList->getRoute(i,j),TmpRouteList->getRoute(i,j+1));
                                // 論文8の式
                                *tempconstr = DepartureTime[TmpRouteList->getRoute(i,j)] + 10.0 + cost.getCost(TmpRouteList->getRoute(i,j),TmpRouteList->getRoute(i,j+1)) <= DepartureTime[TmpRouteList->getRoute(i,j+1)];
                                constrname = "constr"+to_string(i)+ "_" + to_string(j);
                                RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                            }
                        }
                        // デポの時刻DepotTimeとの制約も追加
                        for(i=0;i<TmpRouteList->getRouteListSize();i++){
                            RouteDistance += cost.getCost(0,TmpRouteList->getRoute(i,1));
                            RouteDistance += cost.getCost(TmpRouteList->getRouteSize(i)-2,0);
                            // デポと1番目の制約
                            *tempconstr = DepotTime[i] + cost.getCost(0,TmpRouteList->getRoute(i,1)) <= DepartureTime[TmpRouteList->getRoute(i,1)];
                            constrname = to_string(i) + "constr_depot_1";
                            RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                            // 最後とデポの制約
                            *tempconstr = DepartureTime[TmpRouteList->getRoute(i,TmpRouteList->getRouteSize(i)-2)] + 10.0 + cost.getCost(TmpRouteList->getRoute(i,TmpRouteList->getRouteSize(i)-2),0) == DepotTime[i+m];
                            constrname = to_string(i) + "constr_last_depot";
                            RouteOrderConstr.push_back(model.addConstr(*tempconstr,constrname));
                        }  
                        model.optimize();
                        search_count++;
                        if (ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal) + GAMMA*QP < TmpTotalPenalty){
                            if (QP==0){
                                routelist = *TmpRouteList;
                                TmpTotalPenalty = ALPHA*RouteDistance + BETA*model.get(GRB_DoubleAttr_ObjVal);
                                cout << "改善 " << TmpTotalPenalty << " distance:" << RouteDistance << " count:" << search_count  <<endl;
                                TmpRouteDistance = RouteDistance;
                                TmpBestPenalty = model.get(GRB_DoubleAttr_ObjVal);
                            }
                        }
                        // 悪い解ならなにもしない
                        // ルートの制約をremove
                        for(i=0;i<RouteOrderConstr.size();i++){
                            model.remove(RouteOrderConstr[i]);
                        }
                        //RouteOrderConstrとtemoconstrのメモリ解放
                        vector<GRBConstr>().swap(RouteOrderConstr);
                        delete tempconstr;
                        //TmpRouteListクラスのメモリ解放
                        delete TmpRouteList; 
                    }
            }
            // 一番いい位置に挿入して、それが既存よりよかったら移動
            if (TmpTotalPenalty < BestTotalPenalty){
                    cout << "よい" << endl;
                    bestroutelist = routelist;
                    BestTotalPenalty = TmpTotalPenalty;
                    BestRouteDistance = TmpRouteDistance;
                    BestPenalty =  TmpBestPenalty;
                    for(i=0;i<bestroutelist.getRouteListSize();i++){
                        for(j=0;j<bestroutelist.getRouteSize(i);j++){
                            cout << bestroutelist.getRoute(i,j) << " ";
                        }
                        cout << endl;
                    }
            } else{
                    cout << "よくない" << endl;
                    routelist = bestroutelist;
            }
        }
        */


        cout << "総カウント数:" << search_count << endl;
        cout << "n:" << n << " m:" << m << endl;
        cout << "インスタンス:" << inputfile << endl;
        cout << "係数β:" << BETA << endl;
        cout << "RouteDistance: " << BestRouteDistance << endl;
        cout << "bestpena:"<<BestPenalty << endl;
        cout << "NeighbohList:" << NeighborList.size() << endl;
        cout << "改善回数:" << ImprovedNumber << endl;
 
 
        // for(i=1;i<=2*n;i++){
        //      cout << DepartureTime[i].get(GRB_StringAttr_VarName) << " "
        //     << DepartureTime[i].get(GRB_DoubleAttr_X) << " "
        //     << DepartureTimePenalty[i].get(GRB_DoubleAttr_X) << endl;
        // }
        // for(i=1;i<=n;i++){
        //     cout << RideTime[i].get(GRB_StringAttr_VarName) << " "
        //     << RideTime[i].get(GRB_DoubleAttr_X) << " "
        //     << RideTimePenalty[i].get(GRB_DoubleAttr_X) << endl;
        // }
        
        // 計算終了の時間
        double endtime = cpu_time();

        cout << "時間:" << endtime -starttime << endl;
        // for(i=0;i<m;i++){
        //     cout << i << " " << DepotTime[i].get(GRB_DoubleAttr_X) << " "
        //     << DepotTime[i+m].get(GRB_DoubleAttr_X) << " " << 
        //     DepotTime[i+m].get(GRB_DoubleAttr_X)-DepotTime[i].get(GRB_DoubleAttr_X) << endl;
        // }
        for(i=0;i<bestroutelist.getRouteListSize();i++){
            for(j=0;j<bestroutelist.getRouteSize(i);j++){
                cout << bestroutelist.getRoute(i,j) << " ";
            }
            cout << endl;
        }

        
    // } catch (GRBException e) {
    //     cout << "Error code = " << e.getErrorCode() << endl;
    //     cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Error during optimization" << endl;
    }
  
}