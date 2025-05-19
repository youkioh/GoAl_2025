#ifndef SCHEDULER_H_
#define SCHEDULER_H_

// 드론 경로 시각화를 위한 매크로
// #define DRONE_PATH_VISUALIZATION

#include "simulator.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iomanip>

// 드론 경로 및 탐색 상태를 나타내는 구조체
struct DronePathInfo {
    vector<Coord> waypoints;           // 탐색을 위한 중간 웨이포인트 목록
    vector<Coord> currentPath;         // 현재 타겟 웨이포인트까지의 경로
    int currentWaypointIndex;          // 현재 웨이포인트 인덱스
    bool initialized;                  // 초기화 여부 확인 플래그
    int assignedRegion;                // 0: 왼쪽, 1: 오른쪽
};

class Scheduler
{
public:
    Scheduler() {
        initialized = false;
    }

    void on_info_updated(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const vector<vector<vector<int>>> &known_cost_map,
                         const vector<vector<OBJECT>> &known_object_map,
                         const vector<shared_ptr<TASK>> &active_tasks,
                         const vector<shared_ptr<ROBOT>> &robots);

    bool on_task_reached(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const vector<vector<vector<int>>> &known_cost_map,
                         const vector<vector<OBJECT>> &known_object_map,
                         const vector<shared_ptr<TASK>> &active_tasks,
                         const vector<shared_ptr<ROBOT>> &robots,
                         const ROBOT &robot,
                         const TASK &task);

    ROBOT::ACTION idle_action(const set<Coord> &observed_coords,
                              const set<Coord> &updated_coords,
                              const vector<vector<vector<int>>> &known_cost_map,
                              const vector<vector<OBJECT>> &known_object_map,
                              const vector<shared_ptr<TASK>> &active_tasks,
                              const vector<shared_ptr<ROBOT>> &robots,
                              const ROBOT &robot);

private:
    // 초기화가 완료되었는지 확인하는 플래그
    bool initialized;
    
    // 로봇 ID에 따른 경로 정보 맵
    unordered_map<int, DronePathInfo> dronePaths;
    
    // 드론의 평균 이동 비용 (계산 예정)
    int avgDroneCost;
    
    // 드론의 탐색 경로 초기화
    void initialize(const vector<vector<vector<int>>> &known_cost_map,
                   const vector<vector<OBJECT>> &known_object_map,
                   const vector<shared_ptr<ROBOT>> &robots);
    
    // 드론이 할당된 지역을 탐색하기 위한 웨이포인트 생성
    vector<Coord> generateWaypoints(int mapSize, int assignedRegion);
    
    // D* 알고리즘을 사용하여 목표까지의 경로 찾기
    vector<Coord> findPathToTarget(const Coord &start, 
                                  const Coord &target,
                                  const vector<vector<vector<int>>> &known_cost_map,
                                  const vector<vector<OBJECT>> &known_object_map);
    
    // 현재 위치와 경로를 기반으로 다음 행동 결정
    ROBOT::ACTION getNextAction(const Coord &current, const Coord &next);
    // 알려진 맵에서 드론의 평균 이동 비용 계산    
    int calculateAvgDroneCost(const vector<vector<vector<int>>> &known_cost_map);
    // 두 좌표가 인접해 있는지 확인
    bool areAdjacent(const Coord &a, const Coord &b);
    
    // 드론 경로 시각화 함수
    void visualizeDronePaths(const vector<vector<OBJECT>> &known_object_map, 
                            const vector<shared_ptr<ROBOT>> &robots);
};
#endif SCHEDULER_H_