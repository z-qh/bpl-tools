#ifndef _MINEMAP_H_
#define _MINEMAP_H_

#include "random"



//Random Generator
static std::default_random_engine dre;
static std::uniform_int_distribution<unsigned> uid(0, 100000000);


class MineMap{
private:
    std::vector<std::vector<int>> Map;//MineMap
    //DataMap:99-Mine 100-108-MineNums
    //ShowMap:-1-Mine 000-008-MineNums
    //ShowMap:-2-Flag 050-058-Flag
    //ShowMap:-3 Hide
    //More Than 99 Mean Not Visiable
    int WinFlag=1;//0-Failed 1-Trying 2-Win
    int RestMineNum=0;//Rest Mine Num
    bool FirstStep=true;//New Game
    int Times=0;//Time Used
    int MineNum=40;//Mine Num
    int MX=16;//Column
    int MY=16;//Row
    std::vector<std::pair<int,int>>HistoryStep;//History Step
    // Generate The MineMap
    void CreateMineMap(){
        //Generate A New MineMap With Invalid Value
        Map.clear();
        Map.resize(MX);
        for(auto& line : Map){
            line.resize(MY);
            std::fill(line.begin(), line.end(), 100);
        }
        //Generate The Mine
        for(int i = 0; i < MineNum; i++){
            int m = (int)uid(dre) % MX;
            int n = (int)uid(dre) % MY;
            if(Map[m][n] != 99) Map[m][n] = 99;// Push Mine
            else i--;// Try Again
        }
        //Check All Number
        for(int i = 0; i < MX; ++i){
            for(int j = 0; j < MY; ++j){
                if(Map[i][j] == 99) {
                    for (int m = -1; m < 2; ++m) {
                        for (int n = -1; n < 2; ++n) {
                            if ((i + m >= MX) || (j + n >= MY) || (i + m < 0) || (j + n < 0) ||
                                (Map[i + m][j + n] == 99))
                                continue;
                            Map[i + m][j + n]++;
                        }
                    }
                }
            }
        }
        // Adjust Data
        WinFlag = 1;
        RestMineNum = MineNum;
        FirstStep = true;
        Times = 0;
    }
    bool JudgeStatus(){
        for(auto&line:Map){
            for(auto&p:line){
                if(p>99)
                    return false;
            }
        }
        WinFlag = 2;
        return true;
    }
    //
    bool TriggerAStep(int m, int n){
        if(WinFlag == 0 || WinFlag == 2)
            return false;
        if( m >= MX || n >= MY || m < 0 || n < 0 || Map[m][n] < 99 )
            return false;
        if(Map[m][n] >= 101 && Map[m][n] <= 100){
            Map[m][n] -= 100;
            FirstStep = false;
            JudgeStatus();
            return true;
        }
        if(Map[m][n] == 100){
            Map[m][n] -= 100;
            TriggerAStep(m - 1, n);
            TriggerAStep(m + 1, n);
            TriggerAStep(m, n - 1);
            TriggerAStep(m, n + 1);
            TriggerAStep(m - 1, n - 1);
            TriggerAStep(m + 1, n - 1);
            TriggerAStep(m - 1, n + 1);
            TriggerAStep(m + 1, n + 1);
        }
        if(Map[m][n] == 99){
            // Avoid First To Trigger Mine
            if(FirstStep){
                RestMineNum--;
                int localAroundMineNum = 0;
                for(int a = -1; a < 2; ++a){
                    for(int b = -1; b < 2; ++b){
                        if ((a + m < MX) && (b + n < MY) && (a + m >= 0) && (b + n >= 0) && (a != 0 || b != 0 )){
                            if(Map[a + m][b + n] > 99)
                                Map[a + m][b + n] --;
                            if(Map[a + m][b + n] == 99)
                                localAroundMineNum++;
                        }
                    }
                }
                FirstStep = false;
                Map[m][n] = 100 + localAroundMineNum;
                TriggerAStep(m, n);
                return true;
            }
            // Game Over
            for(auto&line:Map)
                for(auto&p:line){
                    if(p==99) p = -1;
                    if(p > 49 && p < 60) p = -2;
                }
            WinFlag = 0;
        }
        return true;
    }
    // Check Around Flags If Equal To The Nums
    int CountAroundFlag(int m, int n)
    {
        int FlagNumAround = 0;
        if (m >= MX || m < 0 || n >= MY || n < 0)
            return -1;
        int a, b;
        for (a = -1; a < 2; a++)
            for (b = -1; b < 2; b++)
            {
                if ((m + a >= MX) || (n + b >= MY) || (m + a < 0) || (n + b < 0) || (Map[m + a][n + b] > 60) || (Map[m + a][n + b] < 40))
                    continue;
                FlagNumAround++;
            }
        return FlagNumAround;
    }
public:
    void tryAgain(int mx = 16, int my = 16, int minenum=40){
        MX = mx;
        MY = my;
        MineNum = minenum;
        CreateMineMap();
    }
    int restMineNum(){
        return RestMineNum;
    }
    // Return First Show Data
    // Return Second 1 OK, Return 0 Failed, Return 2 Win -1:Call Error
    std::pair<std::vector<std::vector<int>>,int> getAStep(int m, int n){
        if(TriggerAStep(m, n)){
            std::vector<std::vector<int>> visMap;
            getShowMap(visMap);
            return std::make_pair(visMap, WinFlag);
        }
        else{
            std::vector<std::vector<int>> empty;
            return std::make_pair(empty, -1);
        }
    }
    // Return Show Data
    void getShowMap(std::vector<std::vector<int>>& VisData){
        VisData = Map;
        for(auto&line:VisData)
            for(auto&p:line){
                if(p>=99) p = -2;
            }
    }
    // Set Flag
    bool SetFlag(int m, int n){
        if(WinFlag == 0 || WinFlag == 2)
            return false;
        if (m >= MX || m < 0 || n >= MY || n < 0 || Map[m][n] < 40)
            return false;
        if (Map[m][n] > 90)
        {
            Map[m][n] -= 50;
            RestMineNum--;
        }
        else if (Map[m][n] > 40 && Map[m][n] < 60)
        {
            RestMineNum++;
            Map[m][n] += 50;
        }
        return true;
    }
    // Auto Check
    bool AutoCheck(int m, int n){
        if (m >= MX || m < 0 || n >= MY || n < 0 || Map[m][n] > 40)
            return false;
        if (Map[m][n] == CountAroundFlag(m, n))
        {
            TriggerAStep(m - 1, n);
            TriggerAStep(m + 1, n);
            TriggerAStep(m, n - 1);
            TriggerAStep(m, n + 1);
            TriggerAStep(m - 1, n - 1);
            TriggerAStep(m + 1, n - 1);
            TriggerAStep(m - 1, n + 1);
            TriggerAStep(m + 1, n + 1);
        }
        return true;
    }
};


#endif //_MINEMAP_H_