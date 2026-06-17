#ifndef SCHEDULER_H_
#define SCHEDULER_H_

// #define DRONE_PATH_VISUALIZATION

#include <algorithm>
#include <random>
#include <unordered_set>
#include <set>
#include "simulator.h"
#include <unordered_map>
#include <map>

// 드론 경로 상태 구조체
// trajectory[0] = 현재 위치, trajectory[1..] = 이후 이동할 셀 (Beam Search 결과)
struct DronePathInfo {
    vector<Coord> trajectory;
    bool initialized;
};

struct Chromosome {
    vector<int> task_seq;      // Task ID의 순열 (예: [3, 1, 4, 2])
    vector<int> robot_assign;  // 각 Task 인덱스에 매핑되는 Robot ID (예: [1, 2, 3, 1])
    int cost;                // 해당 스케줄링의 총 비용
    double fitness;               // 평가된 Fitness (높을수록 좋음)
    vector<vector<pair<int, int>>> path; // 각 Robot이 Task를 수행하기 위해 이동하는 경로 (예: Robot 1의 경로, Robot 2의 경로, ...)

    bool operator<(const Chromosome& other) const {
        return this->fitness > other.fitness; // Fitness가 높을수록 우선순위가 높음
    }
};

class Scheduler
{
public:
    Scheduler() : g(rd()) {} 

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

    // 초기화가 완료되었는지 확인하는 플래그
    bool initialized = false;

    unordered_map<int, DronePathInfo> dronePaths;
    int initialDroneEnergy;
    int avgDroneCost;

    // 발견된 task 추적 (누적 카운터, EID unknown weight 조정용)
    int found_tasks_count;
    set<int> seen_task_ids;

    // EID 맵 (per-cell 정보 가치)
    vector<vector<double>> eid_map;

    // 동적 budget 계산용 (buildEIDMap에서 갱신)
    double unknown_ratio = 1.0;

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

    // Drone 제외 robot scheduling을 위한 solution sequence and path
    vector<Scheduler::Point> sol_seq;
    vector<pair<int, vector<pair<int, int>>>> sol_path;

    vector<Chromosome> population; // 유전 알고리즘을 위한 개체군
    vector<Chromosome> offspring; // 유전 알고리즘을 위한 자손
    Chromosome best_solution; // 최적 솔루션 저장
    vector<vector<pair<int, int>>> best_solution_path; // 최적 솔루션의 경로 저장

    vector<int> task_status; // 각 Task의 상태를 나타내는 벡터 (예: 0 = 미할당, 1 = 할당됨, 2 = 완료)

    std::unordered_map<int, Coord> previous_positions; // Robot ID별 이전 위치 저장

    // Drone 제외 robot scheduling이 되었는지 확인
    bool scheduled = false;

    // 주어진 맵 정보 따라 scheduling 수행
    void initial_scheduling(const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots);
    void update_scheduling(const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map,
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots);

    // Robot, Task간 Dijkstra 기반 cost 계산
    int calculate_distance(const vector<vector<vector<int>>>& grid, 
        pair<int, int> start, 
        pair<int, int> end, int robotType, 
        vector<pair<int, int>>& path);
    int cost_function(const vector<vector<vector<int>>>& known_cost_map, 
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots, 
        Chromosome& chromosome);
    int calculate_cost(const vector<vector<vector<int>>>& known_cost_map, 
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots, 
        const int robotId, 
        const vector<int>& sol_seq,
        vector<pair<int, int>>& path);
    int update_path(const vector<vector<vector<int>>>& known_cost_map, 
        const ROBOT& robot, 
        Chromosome& chromosome);

    // Genetic algorithm을 위한 함수들
    void InitializePopulation(const vector<shared_ptr<TASK>>& active_tasks, 
        vector<Chromosome>& population, 
        int population_size);
    void UpdatePopulation(const vector<shared_ptr<TASK>>& active_tasks, 
        vector<Chromosome>& population,
        const vector<int>& completed_tasks,
        const vector<int>& new_active_tasks);
    void Evaluate(const vector<vector<vector<int>>>& known_cost_map, 
        const vector<shared_ptr<TASK>>& active_tasks,
        const vector<shared_ptr<ROBOT>>& robots, 
        vector<Chromosome>& population);
    void Crossover(vector<Chromosome>& population, vector<Chromosome>& offspring, double crossover_rate = 0.6);
    void Mutate_Task(vector<Chromosome>& offspring, double mutation_rate = 0.1);
    void Mutate_Robot(vector<Chromosome>& population, double mutation_rate = 0.1);
    void Select(vector<Chromosome>& population, vector<Chromosome>& offspring);

    // Local exploration action 결정 (드론 제외)
    ROBOT::ACTION getLocalExplorationAction(const ROBOT& robot,
        const vector<vector<vector<int>>>& known_cost_map,
        const vector<vector<OBJECT>>& known_object_map);

    // Random number generator
    random_device rd;          
    mt19937 g;
};
#endif SCHEDULER_H_
