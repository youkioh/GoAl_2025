#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <algorithm>
#include <random>  
#include "simulator.h"


// 드론 경로 및 탐색 상태를 나타내는 구조체
struct DronePathInfo {
    Coord targetDestination;       // 탐색 목적지
    vector<Coord> currentPath;     // 현재 목적지까지의 경로
    bool initialized;              // 초기화 여부 확인 플래그
    int movesSinceLastDestUpdate;  // 마지막 목적지 갱신 이후 이동 횟수
    Coord lastMovement;            // 마지막 이동 방향 (0,0)은 처음 또는 방향 없음
};

// 셀 점수 계산을 위한 구조체
struct CellScore {
    Coord position;
    double score;
    
    CellScore(Coord pos, double s) : position(pos), score(s) {}
    
    bool operator<(const CellScore& other) const {
        return score < other.score;
    }
};

class Scheduler
{
public:
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};

    struct DijkNode {
        int cost;
        int x, y;
        bool operator>(const DijkNode& other) const {
            return cost > other.cost;
        }
    };

    typedef struct point_{
        int x, y;
        int id, type;
        int cost[2];
        int energy;
    }Point;

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
/*Drone 탐색*/

    // 내부시간 카운터
    int max_time;
    int current_time;

    // 각 셀에 마지막으로 방문한 시간 정보
    vector<vector<int>> last_observed_time_map;

    // 초기화가 완료되었는지 확인하는 플래그
    bool initialized;
    
    // 로봇 ID에 따른 경로 정보 맵
    unordered_map<int, DronePathInfo> dronePaths;
    
    // 드론의 평균 이동 비용 (계산 예정)
    int avgDroneCost;
    
    // 드론 초기화
    void initialize(const vector<vector<vector<int>>> &known_cost_map,
                   const vector<vector<OBJECT>> &known_object_map,
                   const vector<shared_ptr<ROBOT>> &robots);
    
    // 각 셀의 탐색 점수 계산
    double calculateBaseScore(const Coord &position, 
                           const Coord &dronePosition,
                           const vector<vector<vector<int>>> &known_cost_map,
                           const vector<vector<OBJECT>> &known_object_map);
    
    // 다른 드론과의 거리 점수 계산
    double calculateDroneDistanceScore(const Coord &position,
                                    const vector<shared_ptr<ROBOT>> &robots,
                                    int currentDroneId);
    
    // 이전 이동 방향과의 일치도 점수 계산
    double calculateDirectionAlignmentScore(const Coord &position,
                                         const Coord &dronePosition,
                                         const Coord &lastMovement);
    
    // 맵 중심에서의 확산 보너스 점수 계산
    double calculateSpreadBonusScore(const Coord &position,
                                  const vector<vector<OBJECT>> &known_object_map);

    double countCellsToReveal(const vector<Coord>& path,
                                const vector<vector<OBJECT>>& known_object_map);

    // 경로에서 오래 전에 방문한 셀에 대한 보너스 점수 계산
    double oldBonusInPath(const vector<Coord> &path,
                       const vector<vector<OBJECT>> &known_object_map);
    
    // 최적의 목적지 찾기
    Coord findBestDestination(const Coord &dronePosition,
                            const Coord &lastMovement,
                            const vector<vector<vector<int>>> &known_cost_map,
                            const vector<vector<OBJECT>> &known_object_map,
                            const vector<shared_ptr<ROBOT>> &robots,
                            int droneId);
    
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
    
    // 맨해튼 거리 계산
    int calculateManhattanDistance(const Coord &a, const Coord &b);
    
    // 유클리드 거리 계산
    double calculateEuclideanDistance(const Coord &a, const Coord &b);
      // 셀이 알려지지 않은 셀인지 확인
    bool isUnknownCell(const Coord &coord, const vector<vector<OBJECT>> &known_object_map);
    
    // 경로에 벽이 있는지 확인
    bool pathHasWalls(const vector<Coord> &path, const vector<vector<OBJECT>> &known_object_map);
    
    // D* 경로에서 알려지지 않은 셀 수 계산
    int countUnknownCellsInPath(const vector<Coord> &path, 
                               const vector<vector<OBJECT>> &known_object_map);
    
    // 드론의 목적지와 경로 업데이트
    void updateDroneDestination(const Coord &dronePosition,
                              DronePathInfo &pathInfo,
                              const vector<vector<vector<int>>> &known_cost_map,
                              const vector<vector<OBJECT>> &known_object_map,
                              const vector<shared_ptr<ROBOT>> &robots,
                              int droneId);
    
    // 드론 경로 시각화 함수
    void visualizeDronePaths(const vector<vector<OBJECT>> &known_object_map, 
                            const vector<shared_ptr<ROBOT>> &robots);

/*Task Scheduling*/

    // Drone 제외 robot scheduling을 위한 solution sequence and path
    vector<Scheduler::Point> sol_seq;
    vector<pair<int,vector<pair<int, int>>>> sol_path;

    // Drone 제외 robot scheduling이 되었는지 확인
    bool scheduled;
    int n_sch;

    // 주어진 맵 정보 따라 scheduling 수행
    void schedule_tasks(const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots);

    // Robot, Task간 Dijkstra 기반 최단거리 계산
    int calculate_distance(const vector<vector<vector<int>>>& grid, pair<int, int> start, pair<int, int> end, int robotType, vector<pair<int, int>>& path);

    // 각 Robot의 Cost 값 계산
    int calculate_cost(const vector<vector<vector<int>>>& known_cost_map, vector<Point> array, vector<pair<int, int>>& path);

    // 전체 solution sequence의 cost 계산
    int cost_function(const vector<vector<vector<int>>>& known_cost_map, vector<Point> array, vector<pair<int,vector<pair<int, int>>>>& path);

    // Simulated annealing 기반 최적 scheduling 탐색
    vector<Scheduler::Point> simulated_annealing(
        const vector<Scheduler::Point>& initial_points, const vector<vector<vector<int>>> &known_cost_map,vector<pair<int,vector<pair<int, int>>>> &path, int &cost,
        double start_temp = 10000.0,
        double end_temp = 1e-3,
        double cooling_rate = 0.999,
        int max_iterations = 10000
    );

    // Random한 solution sequence 생성
    vector<Scheduler::Point> shuffle_point(vector<Scheduler::Point> array); 
    vector<Scheduler::Point> two_opt_swap(const vector<Point>& route, int i, int k);
};
#endif SCHEDULER_H_