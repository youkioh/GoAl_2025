#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <algorithm>
#include <random>
#include <unordered_set>
#include <set>
#include "simulator.h"
#include <unordered_map>

// 드론 경로 상태 구조체
// trajectory[0] = 현재 위치, trajectory[1..] = 이후 이동할 셀 (Beam Search 결과)
struct DronePathInfo {
    vector<Coord> trajectory;
    bool initialized;
};

class Scheduler
{
public:
    const int dx[4] = { -1, 1, 0, 0 };
    const int dy[4] = { 0, 0, -1, 1 };

    struct DijkNode {
        int cost;
        int x, y;
        bool operator>(const DijkNode& other) const {
            return cost > other.cost;
        }
    };

    typedef struct point_ {
        int x, y;
        int id, type;
        int cost[2];
        int energy;
    }Point;

    void on_info_updated(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots);

    bool on_task_reached(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots,
        const ROBOT& robot,
        const TASK& task);

    ROBOT::ACTION idle_action(const set<Coord>& observed_coords,
        const set<Coord>& updated_coords,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots,
        const ROBOT& robot);

private:
    /*Drone 탐색*/

    int max_time;
    int current_time;

    vector<vector<int>> last_observed_time_map;
    bool initialized;

    unordered_map<int, DronePathInfo> dronePaths;
    int initialDroneEnergy;
    int avgDroneCost;

    // 발견된 task 추적 (누적 카운터, EID unknown weight 조정용)
    int found_tasks_count;
    set<int> seen_task_ids;

    // EID 맵 (per-cell 정보 가치)
    vector<vector<double>> eid_map;

    // 초기화
    void initialize(const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<ROBOT>>& robots);

    // EID 맵 전체 재계산
    void buildEIDMap(const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<ROBOT>>& robots);

    // 위치 pos의 ±2 관측 범위를 인코딩된 정수 집합으로 반환
    unordered_set<int> observeRange(const Coord& pos, int mapSize);

    // Beam Search로 드론 궤적 결정
    vector<Coord> beamSearchTrajectory(
        const Coord& dronePos,
        const vector<vector<double>>& temp_eid,
        int budget,
        int beamWidth,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map);

    // 모든 드론의 궤적을 순차적으로 결정 (중복 탐색 방지)
    void planAllDroneTrajectories(
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<ROBOT>>& robots);

    // Dijkstra 기반 경로 탐색 (재사용)
    vector<Coord> findPathToTarget(const Coord& start,
        const Coord& target,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map);

    ROBOT::ACTION getNextAction(const Coord& current, const Coord& next);

    int calculateAvgDroneCost(const vector<vector<vector<int>>>& known_cost_map);

    bool areAdjacent(const Coord& a, const Coord& b);

    double calculateEuclideanDistance(const Coord& a, const Coord& b);

    bool pathHasWalls(const vector<Coord>& path, const vector<vector<OBJECT>>& known_object_map);

    void visualizeDronePaths(const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<ROBOT>>& robots);

    /*Task Scheduling*/

    vector<Scheduler::Point> sol_seq;
    vector<pair<int, vector<pair<int, int>>>> sol_path;
    bool scheduled;
    int n_sch;

    void schedule_tasks(const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots);

    int calculate_distance(const vector<vector<vector<int>>>& grid, pair<int, int> start, pair<int, int> end, int robotType, vector<pair<int, int>>& path);

    int calculate_cost(const vector<vector<vector<int>>>& known_cost_map, vector<Point> array, vector<pair<int, int>>& path);

    int cost_function(const vector<vector<vector<int>>>& known_cost_map, vector<Point> array, vector<pair<int, vector<pair<int, int>>>>& path);

    vector<Scheduler::Point> simulated_annealing(
        const vector<Scheduler::Point>& initial_points,
        const vector<vector<vector<int>>>& known_cost_map,
        vector<pair<int, vector<pair<int, int>>>>& path,
        int& cost,
        double start_temp = 10000.0,
        double end_temp = 1e-3,
        double cooling_rate = 0.999,
        int max_iterations = 10000
    );

    vector<Scheduler::Point> shuffle_point(vector<Scheduler::Point> array);
    vector<Scheduler::Point> two_opt_swap(const vector<Point>& route, int i, int k);
};
#endif SCHEDULER_H_
