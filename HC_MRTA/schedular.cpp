#include "schedular.h"
#include "simulator.h"
#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <algorithm>
#include <limits>
#include <cmath>
#include <unordered_set>
#include <chrono>

// ---- 타이밍 누적 ----
struct TimingStats {
    double beam_ms        = 0; int beam_calls        = 0;
    double buildEID_ms    = 0; int buildEID_calls    = 0;
    double crossover_ms   = 0; int crossover_calls   = 0;
    double init_sched_ms  = 0;
    double upd_sched_ms   = 0; int upd_sched_calls   = 0;
};
static TimingStats g_tm;

void printTimingReport() {
    auto pct = [](double part, double total) {
        return total > 0 ? part / total * 100.0 : 0.0;
    };
    double total = g_tm.init_sched_ms + g_tm.upd_sched_ms
                 + g_tm.beam_ms + g_tm.buildEID_ms;
    fprintf(stderr, "\n=== TIMING REPORT (total: %.1fms) ===\n", total);
    fprintf(stderr, "  initial_sched   : %8.1fms  %5.1f%%\n",
            g_tm.init_sched_ms, pct(g_tm.init_sched_ms, total));
    fprintf(stderr, "  update_sched    : %8.1fms  %5.1f%%  calls=%d\n",
            g_tm.upd_sched_ms, pct(g_tm.upd_sched_ms, total), g_tm.upd_sched_calls);
    fprintf(stderr, "  Crossover       : %8.1fms  (within GA)\n", g_tm.crossover_ms);
    fprintf(stderr, "  beamSearch      : %8.1fms  %5.1f%%  calls=%d\n",
            g_tm.beam_ms,      pct(g_tm.beam_ms, total),      g_tm.beam_calls);
    fprintf(stderr, "  buildEIDMap     : %8.1fms  %5.1f%%  calls=%d\n",
            g_tm.buildEID_ms,  pct(g_tm.buildEID_ms, total),  g_tm.buildEID_calls);
    fprintf(stderr, "=====================================\n");
}
// ---- 타이밍 끝 ----

#define EID_UNKNOWN_WEIGHT    2.6286  // unknown 셀 기여 가중치
#define EID_STALENESS_WEIGHT  0.5881  // 오래된 known 셀 기여 가중치
#define EID_DISTANCE_WEIGHT   0.1338  // 로봇 거리 기여 가중치
#define DRONE_BUDGET_MIN  0.1508  // unknown 비율 0일 때 budget 비율
#define DRONE_BUDGET_MAX  0.3152  // unknown 비율 1일 때 budget 비율
#define BEAM_WIDTH            8     // Beam Search 너비

#define TASK_SCHEDULING_START_TIME_RATIO 0.2 // Time limit의 몇 %부터 task scheduling을 시작할지 결정하는 비율

using namespace std;

// findPathToTarget 내부에서 사용하는 노드 구조체
struct Node {
    Coord coord;
    int cost;

    Node(Coord c, int g) : coord(c), cost(g) {}

    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////General Function Section.//////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void Scheduler::on_info_updated(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots) {

    if (!initialized) {
        initialize(known_cost_map, known_object_map, robots);
    }
    else {
        current_time++;

        // 관측 셀 시간 업데이트
        for (const auto& coord : observed_coords) {
            if (coord.x >= 0 && coord.x < (int)last_observed_time_map.size() &&
                coord.y >= 0 && coord.y < (int)last_observed_time_map[0].size()) {
                last_observed_time_map[coord.x][coord.y] = current_time;
            }
        }

        // 새로 발견된 task 누적 추적
        for (const auto& task : active_tasks) {
            if (seen_task_ids.insert(task->id).second) {
                found_tasks_count++;
            }
        }
    }

    // Drone 제외 robot schedule 활성화
    if (current_time >= max_time * TASK_SCHEDULING_START_TIME_RATIO && !scheduled) {
        scheduled = true;
        //cout << "Robot Scheduling Start" << endl;
        initial_scheduling(known_cost_map, known_object_map, active_tasks, robots);
        //cout << "Robot Scheduling "<<n_sch++<<" Completed" << endl;
    }

    if (!updated_coords.empty()) {
        if (scheduled) {
            for (auto uc : updated_coords)
            {
                #ifdef VERBOSE
                if (known_object_map[uc.x][uc.y] == OBJECT::TASK)
                {
                    cout << "New task detected at (" << uc.x << ", " << uc.y << "). At time " << current_time << endl;
                }
                #endif
            }
            update_scheduling(known_cost_map, known_object_map, active_tasks, robots);
        }

        // 드론 궤적에 벽이 생긴 경우 전체 재계획
        bool needReplan = false;
        for (const auto& robot : robots) {
            if (robot->type != ROBOT::TYPE::DRONE) continue;
            auto& pathInfo = dronePaths[robot->id];
            if (!pathInfo.initialized) continue;
            if (pathHasWalls(pathInfo.trajectory, known_object_map)) {
                needReplan = true;
                break;
            }
        }
        if (needReplan) {
            planAllDroneTrajectories(known_cost_map, known_object_map, robots);
        }
    }
}

bool Scheduler::on_task_reached(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot,
    const TASK& task) {
    return robot.type != ROBOT::TYPE::DRONE;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot) {

    // 드론 제외 로봇: simulated annealing 결과에 따라 이동
    if (robot.type != ROBOT::TYPE::DRONE) {
        if (!scheduled) return ROBOT::ACTION::HOLD;

        Coord nextPos;
        if (scheduled) {
            auto& sol_path = best_solution_path[robot.id];
            if(!sol_path.empty()){
                #ifdef VERBOSE
                cout << "Robot " << robot.id << " (" << robot.type << ") best solution path: ";
                for (const auto& step : sol_path) {
                    cout << "(" << step.first << ", " << step.second << ") ";
                }
                #endif

                nextPos = Coord(sol_path[0].first, sol_path[0].second);
                sol_path.erase(sol_path.begin());

                return getNextAction(robot.get_coord(), nextPos);
            }
            else {
                int remaining_time = max_time - current_time;
                int energy_threshold = remaining_time * ROBOT::ROBOT_ENERGY_PER_TICK * 1.2;
                if (robot.get_energy() > energy_threshold) {
                    cout << "Robot " << robot.id << " (" << robot.type << ") has no path but sufficient energy. Exploring locally." << endl;
                    return getLocalExplorationAction(robot, known_cost_map, known_object_map);
                } 
                else{
                    return ROBOT::ACTION::HOLD; // 에너지가 낮으면 HOLD
                }
            }
        }
        else
            return ROBOT::ACTION::HOLD;
    }
    else {
        
        if (robot.get_energy() < initialDroneEnergy / 2  && current_time < max_time * 5 / 10) {
            // 대기 상태가 끝나고 다시 이동을 시작할 때 재계획하도록 남은 궤적을 비워둡니다
            dronePaths[robot.id].trajectory = { robot.get_coord() };
            return ROBOT::ACTION::HOLD; // 드론은 HOLD 액션
        }
    auto& pathInfo = dronePaths[robot.id];

    if (!pathInfo.initialized) return ROBOT::ACTION::HOLD;

    // 궤적이 소진된 경우 전체 재계획
    if (pathInfo.trajectory.size() <= 1) {

#ifdef DRONE_PATH_VISUALIZATION
        cout << "드론 " << robot.id << " 궤적 소진. 전체 재계획." << endl;
#endif
        planAllDroneTrajectories(known_cost_map, known_object_map, robots);
    }

    // 궤적을 따라 이동
    if (pathInfo.trajectory.size() >= 2) {
        Coord nextPos = pathInfo.trajectory[1];
        pathInfo.trajectory.erase(pathInfo.trajectory.begin());

#ifdef DRONE_PATH_VISUALIZATION
        cout << "드론 " << robot.id << " (" << robot.get_coord().x << "," << robot.get_coord().y
             << ") → (" << nextPos.x << "," << nextPos.y << ") 남은 궤적: "
             << pathInfo.trajectory.size() << endl;
#endif
        return getNextAction(robot.get_coord(), nextPos);
    }

    return ROBOT::ACTION::HOLD;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////Drone Section./////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void Scheduler::initialize(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {

    initialized = true;
    max_time = known_object_map.size() * 100;
    current_time = 0;
    found_tasks_count = 0;
    seen_task_ids.clear();

    // 첫 번째 드론의 초기 에너지 저장
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE) {
            initialDroneEnergy = robot->get_energy();
            break;
        }
    }

    int mapSize = known_object_map.size();

    last_observed_time_map.assign(mapSize, vector<int>(mapSize, -1));
    eid_map.assign(mapSize, vector<double>(mapSize, 0.0));

    for (int x = 0; x < mapSize; x++) {
        for (int y = 0; y < mapSize; y++) {
            if (known_object_map[x][y] != OBJECT::UNKNOWN) {
                last_observed_time_map[x][y] = 0;
            }
        }
    }

    avgDroneCost = calculateAvgDroneCost(known_cost_map);

    // 드론 경로 정보 초기화
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE) {
            DronePathInfo pathInfo;
            pathInfo.initialized = true;
            pathInfo.trajectory = { robot->get_coord() };
            dronePaths[robot->id] = pathInfo;
        }
    }

    // 초기 궤적 계획
    planAllDroneTrajectories(known_cost_map, known_object_map, robots);

    scheduled = false;
}

// EID 맵 전체 재계산
// EID[x][y]: 셀 (x,y) 자체의 정보 가치 (per-cell)
// 드론이 위치 p를 방문할 때의 정보 이득은 ±2 범위 합산으로 별도 계산 (beamSearchTrajectory 내)
void Scheduler::buildEIDMap(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {

    auto _t0 = chrono::high_resolution_clock::now();
    int mapSize = known_object_map.size();
    double maxDist = sqrt(2.0) * mapSize;

    for (int x = 0; x < mapSize; x++) {
        for (int y = 0; y < mapSize; y++) {
            if (known_object_map[x][y] == OBJECT::WALL) {
                eid_map[x][y] = 0.0;
                continue;
            }

            // 가장 가까운 로봇까지 유클리드 거리 (정규화)
            double minDist = numeric_limits<double>::max();
            for (const auto& r : robots) {
                double d = calculateEuclideanDistance(Coord(x, y), r->get_coord());
                minDist = min(minDist, d);
            }
            double distFactor = min(1.0, minDist / maxDist);

            if (known_object_map[x][y] == OBJECT::UNKNOWN) {
                // unknown 셀: 발견된 task 수가 늘어날수록 가치 감소
                double unknownVal = 1.0 / (1.0 + found_tasks_count);
                eid_map[x][y] = EID_UNKNOWN_WEIGHT * unknownVal
                               + EID_DISTANCE_WEIGHT * distFactor;
            }
            else {
                // known 셀: 오래 관측 안 할수록 가치 증가
                int lastObs = last_observed_time_map[x][y];
                double staleness = (lastObs >= 0)
                    ? static_cast<double>(current_time - lastObs) / max_time
                    : 0.0;
                eid_map[x][y] = EID_STALENESS_WEIGHT * staleness
                               + EID_DISTANCE_WEIGHT * distFactor;
            }
        }
    }
    // unknown_ratio 갱신 (동적 budget 계산용)
    int unknown_cells = 0, non_wall_cells = 0;
    for (int x = 0; x < mapSize; x++)
        for (int y = 0; y < mapSize; y++)
            if (known_object_map[x][y] != OBJECT::WALL) {
                non_wall_cells++;
                if (known_object_map[x][y] == OBJECT::UNKNOWN) unknown_cells++;
            }
    unknown_ratio = non_wall_cells > 0
        ? static_cast<double>(unknown_cells) / non_wall_cells
        : 0.0;

    g_tm.buildEID_ms += chrono::duration<double, milli>(chrono::high_resolution_clock::now() - _t0).count();
    g_tm.buildEID_calls++;
}

// 위치 pos의 ±2 관측 범위를 인코딩된 정수 집합으로 반환
// 인코딩: x * mapSize + y
unordered_set<int> Scheduler::observeRange(const Coord& pos, int mapSize) {
    unordered_set<int> result;
    for (int dx = -2; dx <= 2; dx++) {
        for (int dy = -2; dy <= 2; dy++) {
            int nx = pos.x + dx;
            int ny = pos.y + dy;
            if (nx >= 0 && nx < mapSize && ny >= 0 && ny < mapSize) {
                result.insert(nx * mapSize + ny);
            }
        }
    }
    return result;
}

// Beam Search로 드론 궤적 결정
// - trajectory[0] = dronePos (현재 위치)
// - 매 스텝 인접 셀로 확장, 에너지 예산 소진까지 탐색
// - 각 beam마다 observed set을 유지하여 5x5 관측 범위 EID 중복 집계 방지
vector<Coord> Scheduler::beamSearchTrajectory(
    const Coord& dronePos,
    const vector<vector<double>>& temp_eid,
    int budget,
    int beamWidth,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map) {

    auto beam_start = chrono::high_resolution_clock::now();

    int mapSize = known_cost_map.size();
    int droneIdx = static_cast<int>(ROBOT::TYPE::DRONE);

    struct Beam {
        vector<Coord> path;
        int energy;
        double eid_sum;
        unordered_set<int> observed;
    };

    Beam init;
    init.path     = { dronePos };
    init.energy   = 0;
    init.eid_sum  = 0.0;
    init.observed = observeRange(dronePos, mapSize);

    vector<Beam> beams = { move(init) };

    const int dirX[] = { 0, 1, 0, -1 };
    const int dirY[] = { -1, 0, 1, 0 };

    while (!beams.empty()) {
        vector<Beam> next;

        for (const auto& beam : beams) {
            const Coord& cur = beam.path.back();

            for (int d = 0; d < 4; d++) {
                int nx = cur.x + dirX[d];
                int ny = cur.y + dirY[d];

                if (nx < 0 || nx >= mapSize || ny < 0 || ny >= mapSize) continue;
                if (known_object_map[nx][ny] == OBJECT::WALL) continue;

                int moveCost;
                if (known_object_map[nx][ny] != OBJECT::UNKNOWN &&
                    known_cost_map[nx][ny][droneIdx] > 0 &&
                    known_cost_map[nx][ny][droneIdx] < INFINITE) {
                    moveCost = known_cost_map[nx][ny][droneIdx];
                }
                else {
                    moveCost = static_cast<int>(avgDroneCost * 1.2);
                }

                int newEnergy = beam.energy + moveCost;
                if (newEnergy > budget) continue;

                // ±2 관측 범위에서 새로 얻는 EID 계산 (중복 제거)
                auto newObs = observeRange(Coord(nx, ny), mapSize);
                double gain = 0.0;
                for (int code : newObs) {
                    if (beam.observed.find(code) == beam.observed.end()) {
                        gain += temp_eid[code / mapSize][code % mapSize];
                    }
                }

                Beam newBeam;
                newBeam.path     = beam.path;
                newBeam.path.push_back(Coord(nx, ny));
                newBeam.energy   = newEnergy;
                newBeam.eid_sum  = beam.eid_sum + gain;
                newBeam.observed = beam.observed;
                newBeam.observed.insert(newObs.begin(), newObs.end());

                next.push_back(move(newBeam));
            }
        }

        if (next.empty()) break;

        // 상위 beamWidth개만 유지
        if ((int)next.size() > beamWidth) {
            partial_sort(next.begin(), next.begin() + beamWidth, next.end(),
                [](const Beam& a, const Beam& b) { return a.eid_sum > b.eid_sum; });
            next.resize(beamWidth);
        }

        beams = move(next);
    }

    vector<Coord> result;
    if (beams.empty()) {
        result = { dronePos };
    } else {
        auto best = max_element(beams.begin(), beams.end(),
            [](const Beam& a, const Beam& b) { return a.eid_sum < b.eid_sum; });
        result = best->path;
    }

    g_tm.beam_ms += chrono::duration<double, milli>(chrono::high_resolution_clock::now() - beam_start).count();
    g_tm.beam_calls++;
    return result;
}

// 모든 드론의 궤적을 순차적으로 계획
// 한 드론의 궤적이 관측할 영역의 EID를 temp_eid에서 0으로 차감 →
// 다음 드론이 중복 탐색하지 않도록 유도
void Scheduler::planAllDroneTrajectories(
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {

    buildEIDMap(known_cost_map, known_object_map, robots);

    vector<vector<double>> temp_eid = eid_map;
    int mapSize = known_object_map.size();

    for (const auto& robot : robots) {
        if (robot->type != ROBOT::TYPE::DRONE) continue;

        auto& pathInfo = dronePaths[robot->id];
        if (!pathInfo.initialized) continue;

        // 동적 budget: unknown 비율에 따라 MIN~MAX 선형 보간
        // unknown 많을수록(초반) 크게, 적을수록(후반) 작게
        double budget_ratio = DRONE_BUDGET_MIN +
            (DRONE_BUDGET_MAX - DRONE_BUDGET_MIN) * unknown_ratio;
        int budget = min(
            static_cast<int>(initialDroneEnergy * budget_ratio),
            robot->get_energy()
        );

        vector<Coord> traj = beamSearchTrajectory(
            robot->get_coord(), temp_eid, budget, BEAM_WIDTH,
            known_cost_map, known_object_map
        );

        pathInfo.trajectory = traj;

        // 이 드론이 관측할 셀의 EID를 temp_eid에서 차감 (다음 드론용)
        for (const auto& pos : traj) {
            for (int dx = -2; dx <= 2; dx++) {
                for (int dy = -2; dy <= 2; dy++) {
                    int nx = pos.x + dx, ny = pos.y + dy;
                    if (nx >= 0 && nx < mapSize && ny >= 0 && ny < mapSize) {
                        temp_eid[nx][ny] = 0.0;
                    }
                }
            }
        }

#ifdef DRONE_PATH_VISUALIZATION
        cout << "드론 " << robot->id << " 궤적 계획 완료. 길이: "
             << traj.size() << ", 예산: " << budget << endl;
        visualizeDronePaths(known_object_map, robots);
#endif
    }
}

// 경로에 벽이 있는지 확인
bool Scheduler::pathHasWalls(const vector<Coord>& path,
    const vector<vector<OBJECT>>& known_object_map) {
    int mapSize = known_object_map.size();
    for (const auto& coord : path) {
        if (coord.x >= 0 && coord.x < mapSize &&
            coord.y >= 0 && coord.y < mapSize) {
            if (known_object_map[coord.x][coord.y] == OBJECT::WALL) {
                return true;
            }
        }
    }
    return false;
}

// 역방향 Dijkstra 기반 경로 탐색 (재사용)
vector<Coord> Scheduler::findPathToTarget(const Coord& start,
    const Coord& target,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map) {

    int mapSize = known_cost_map.size();

    if (start.x < 0 || start.y < 0 || start.x >= mapSize || start.y >= mapSize ||
        target.x < 0 || target.y < 0 || target.x >= mapSize || target.y >= mapSize) {
        return {};
    }

    const int dx[] = { 0, 1, 0, -1 };
    const int dy[] = { -1, 0, 1, 0 };

    priority_queue<Node, vector<Node>, greater<Node>> openSet;
    vector<vector<bool>> visited(mapSize, vector<bool>(mapSize, false));
    vector<vector<Coord>> parent(mapSize, vector<Coord>(mapSize, Coord(-1, -1)));
    vector<vector<int>> gScore(mapSize, vector<int>(mapSize, INFINITE));

    openSet.push(Node(target, 0));
    gScore[target.x][target.y] = 0;

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        Coord cur = current.coord;
        if (visited[cur.x][cur.y]) continue;
        visited[cur.x][cur.y] = true;

        if (cur.x == start.x && cur.y == start.y) break;

        for (int i = 0; i < 4; i++) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];

            if (nx < 0 || nx >= mapSize || ny < 0 || ny >= mapSize) continue;
            if (visited[nx][ny]) continue;
            if (known_object_map[nx][ny] == OBJECT::WALL) continue;

            int moveCost;
            if (known_object_map[nx][ny] != OBJECT::UNKNOWN &&
                known_cost_map[nx][ny][static_cast<int>(ROBOT::TYPE::DRONE)] > 0 &&
                known_cost_map[nx][ny][static_cast<int>(ROBOT::TYPE::DRONE)] < INFINITE) {
                moveCost = known_cost_map[nx][ny][static_cast<int>(ROBOT::TYPE::DRONE)];
            }
            else {
                moveCost = static_cast<int>(avgDroneCost * 1.5);
            }

            int newCost = gScore[cur.x][cur.y] + moveCost;
            if (newCost < gScore[nx][ny]) {
                gScore[nx][ny] = newCost;
                parent[nx][ny] = cur;
                openSet.push(Node(Coord(nx, ny), newCost));
            }
        }
    }

    vector<Coord> path;
    Coord current = start;
    if (parent[start.x][start.y].x != -1) {
        while (!(current.x == target.x && current.y == target.y)) {
            path.push_back(current);
            current = parent[current.x][current.y];
        }
        path.push_back(target);
    }
    return path;
}

ROBOT::ACTION Scheduler::getNextAction(const Coord& current, const Coord& next) {
    if (current.x < next.x) return ROBOT::ACTION::RIGHT;
    if (current.x > next.x) return ROBOT::ACTION::LEFT;
    if (current.y < next.y) return ROBOT::ACTION::UP;
    if (current.y > next.y) return ROBOT::ACTION::DOWN;
    return ROBOT::ACTION::HOLD;
}

int Scheduler::calculateAvgDroneCost(const vector<vector<vector<int>>>& known_cost_map) {
    int totalCost = 0;
    int count = 0;
    int droneType = static_cast<int>(ROBOT::TYPE::DRONE);

    for (const auto& row : known_cost_map) {
        for (const auto& cell : row) {
            if (cell[droneType] > 0 && cell[droneType] < INFINITE) {
                totalCost += cell[droneType];
                count++;
            }
        }
    }
    return (count > 0) ? (totalCost / count) : 100;
}

bool Scheduler::areAdjacent(const Coord& a, const Coord& b) {
    int dx = abs(a.x - b.x);
    int dy = abs(a.y - b.y);
    return (dx == 1 && dy == 0) || (dx == 0 && dy == 1);
}

double Scheduler::calculateEuclideanDistance(const Coord& a, const Coord& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

void Scheduler::visualizeDronePaths(const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {
#ifdef DRONE_PATH_VISUALIZATION
    cout << "현재 시뮬레이션 시간: " << current_time << endl;
    cout << "발견된 작업 개수: " << found_tasks_count << endl;
    cout << "드론 에너지 예산(비율): " << (DRONE_BUDGET_MIN + (DRONE_BUDGET_MAX - DRONE_BUDGET_MIN) * unknown_ratio) << endl;
    cout << "현재 드론의 에너지: ";
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE) {
            cout << "드론 " << robot->id << ": " << robot->get_energy() << " ";
        }
    }
    cout << endl;

    int mapSize = known_object_map.size();
    vector<vector<char>> visualMap(mapSize, vector<char>(mapSize, ' '));

    for (int y = 0; y < mapSize; y++) {
        for (int x = 0; x < mapSize; x++) {
            if (known_object_map[x][y] == OBJECT::WALL)    visualMap[x][y] = '#';
            else if (known_object_map[x][y] == OBJECT::UNKNOWN) visualMap[x][y] = '?';
            else if (known_object_map[x][y] == OBJECT::EMPTY)   visualMap[x][y] = '.';
            else if (known_object_map[x][y] == OBJECT::TASK)    visualMap[x][y] = 'T';
        }
    }

    for (const auto& robot : robots) {
        Coord pos = robot->get_coord();
        if (robot->type == ROBOT::TYPE::DRONE)
            cout << "드론 " << robot->id << " 위치: (" << pos.x << ", " << pos.y << ")" << endl;
        visualMap[pos.x][pos.y] = static_cast<char>('0' + robot->id);
    }

    for (const auto& pair : dronePaths) {
        const DronePathInfo& pathInfo = pair.second;
        if (!pathInfo.initialized || pathInfo.trajectory.empty()) continue;

        // 목적지 (궤적 마지막 셀): known('.') 또는 unknown('?') 모두 표시
        const Coord& dest = pathInfo.trajectory.back();
        char d = visualMap[dest.x][dest.y];
        if (d == '.' || d == '?') visualMap[dest.x][dest.y] = 'G';

        // 궤적 표시: unknown 셀('?')도 '*'로 표시하여 전체 궤적을 가시화
        for (const Coord& pt : pathInfo.trajectory) {
            char c = visualMap[pt.x][pt.y];
            if (c == '.' || c == '?') visualMap[pt.x][pt.y] = '*';
        }
    }

    cout << "맵 시각화:" << endl;
    for (int y = mapSize - 1; y >= 0; y--) {
        cout << setw(2) << y << " ";
        for (int x = 0; x < mapSize; x++) cout << visualMap[x][y] << " ";
        cout << endl;
    }
    cout << "   ";
    for (int x = 0; x < mapSize; x++) cout << setw(2) << x;
    cout << endl;
#endif
}


///////////////////////////////////////////////////////////////////////////////////////
//////////////////Task Scheduling Section./////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// 발견된 task를 scheduling
void Scheduler::initial_scheduling(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots)
{
    auto _t0 = chrono::high_resolution_clock::now();
    const int POPULATION_SIZE = 50;
    const int GENERATIONS = 400;
    const int MAX_NUM_TASKS = 20;

    task_status.assign(MAX_NUM_TASKS, 0);
    for(const auto& task: active_tasks) {
        task_status[task->id] = 1;
    }

    InitializePopulation(active_tasks, population, POPULATION_SIZE);
    for(int i = 0; i < GENERATIONS; i++) {
        Crossover(population, offspring);
        Mutate_Task(offspring);
        Mutate_Robot(offspring);
        Evaluate(known_cost_map, active_tasks, robots, population);
        Evaluate(known_cost_map, active_tasks, robots, offspring);
        Select(population, offspring);
    }

    sort(population.begin(), population.end());
    best_solution = population[0];
    best_solution_path = population[0].path;
    #ifdef VERBOSE
    cout << "Initial Scheduling Status" << endl;
    cout << "Task Chromosome: ";
    for(const auto& task_id: best_solution.task_seq) {
        cout << task_id << " ";
    }
    cout << endl;
    cout << "Robot Assignment: ";
    for(const auto& robot_id: best_solution.robot_assign) {
        cout << robot_id << " ";
    }
    cout << endl;

    cout << "Best Solution Cost: " << best_solution.cost << endl;
#endif
    g_tm.init_sched_ms += chrono::duration<double, milli>(chrono::high_resolution_clock::now() - _t0).count();
}

void Scheduler::update_scheduling(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots)
{
    auto _t0 = chrono::high_resolution_clock::now();
    int GENERATIONS = 10;
    vector<int> completed_tasks, new_active_tasks;

    // task_status 업데이트: 0 = 미발견, 1 = 활성, 2 = 완료
    for(int i = 0; i < task_status.size(); i++) {
        if(task_status[i] == 1) {
            if(active_tasks.end() == find_if(active_tasks.begin(), active_tasks.end(), [i](const shared_ptr<TASK>& task) {
                return task->id == i;
            })) {
                completed_tasks.push_back(i);
                task_status[i] = 2;
            }
        }
    }

    for(const auto& task: active_tasks) {
        if(task_status[task->id] == 0) {
            new_active_tasks.push_back(task->id);
            task_status[task->id] = 1;
        }
    }

    if(!completed_tasks.empty() || !new_active_tasks.empty()) {
        UpdatePopulation(active_tasks, population, completed_tasks, new_active_tasks);
        GENERATIONS = 200; // 새로운 task가 발견되면 더 많은 세대 동안 진화
    }

    for(int i = 0; i < GENERATIONS; i++) {
        Crossover(population, offspring);
        Mutate_Task(offspring);
        Mutate_Robot(offspring);
        Evaluate(known_cost_map, active_tasks, robots, population);
        Evaluate(known_cost_map, active_tasks, robots, offspring);
        Select(population, offspring);
    }

    sort(population.begin(), population.end());
    best_solution = population[0];
    best_solution_path = population[0].path;

#ifdef VERBOSE
    cout << "Scheduling Status" << endl;
    cout << "Task Chromosome: ";
    for(const auto& task_id: best_solution.task_seq) {
        cout << task_id << " ";
    }
    cout << endl;
    cout << "Robot Assignment: ";
    for(const auto& robot_id: best_solution.robot_assign) {
        cout << robot_id << " ";
    }
    cout << endl;

    cout << "Best Solution Cost: " << best_solution.cost << endl;
    for(int robot = 1; robot <= 5; robot++) {
        vector<pair<int, int>> sol_path = best_solution_path[robot];
        cout << "Robot " << robot << "best solution path: ";
                for (const auto& step : sol_path) {
                    cout << "(" << step.first << ", " << step.second << ") ";
                }
            cout << endl;
            }
#endif
    g_tm.upd_sched_ms += chrono::duration<double, milli>(chrono::high_resolution_clock::now() - _t0).count();
    g_tm.upd_sched_calls++;
}

void Scheduler::InitializePopulation(const vector<shared_ptr<TASK>> &active_tasks, vector<Chromosome>& population, int population_size)
{
    population.clear();
    int num_tasks = active_tasks.size();
    const int robot_pool[4] = {1, 2, 4, 5};
    for (int i = 0; i < population_size; i++) {
        Chromosome sol;
        for (const auto& task : active_tasks) {
            sol.task_seq.push_back(task->id);
            sol.robot_assign.push_back(robot_pool[rand() % 4]); // caterpilar, wheel 중에서 랜덤하게 로봇 할당
        }
        shuffle(sol.task_seq.begin(), sol.task_seq.end(), g);

        population.push_back(sol);
    }
}

void Scheduler::UpdatePopulation(const vector<shared_ptr<TASK>>& active_tasks, vector<Chromosome>& population, const vector<int>& completed_tasks, const vector<int>& new_active_tasks)
{
    const int robot_pool[4] = {1, 2, 4, 5};
    int population_size = population.size();
    const int new_population_size = population_size / 2;
    
    // 완료된 task를 solution에서 제거
    for(auto& sol: population) {
        for(const auto& task_id: completed_tasks) {
            int idx = find(sol.task_seq.begin(), sol.task_seq.end(), task_id) - sol.task_seq.begin();
            if(idx < sol.task_seq.size()) {
                sol.task_seq.erase(sol.task_seq.begin() + idx);
                sol.robot_assign.erase(sol.robot_assign.begin() + idx);
            }
        }
    }

    if(!new_active_tasks.empty()) {
    sort(population.begin(), population.end());
    // 새로운 task를 solution에 추가 : Last Insertion
        for(int i = 0; i < new_population_size; i++) {
            for(const auto& task_id: new_active_tasks) {
                population[i].task_seq.push_back(task_id);
                population[i].robot_assign.push_back(robot_pool[rand() % 4]); // caterpilar, wheel 중에서 랜덤하게 로봇 할당
            }
        }

        for(int i = new_population_size; i < population_size; i++) {
            population[i].task_seq.clear();
            population[i].robot_assign.clear();
            for(const auto& task: active_tasks) {
                population[i].task_seq.push_back(task->id);
                population[i].robot_assign.push_back(robot_pool[rand() % 4]); // caterpilar, wheel 중에서 랜덤하게 로봇 할당
            }
            shuffle(population[i].task_seq.begin(), population[i].task_seq.end(), g);
        }
    }
}

void Scheduler::Evaluate(const vector<vector<vector<int>>>& known_cost_map, const vector<shared_ptr<TASK>>& active_tasks, const vector<shared_ptr<ROBOT>>& robots, vector<Chromosome>& population) {
    for (auto& sol : population) {
        sol.cost = Scheduler::cost_function(known_cost_map, active_tasks, robots, sol);
        sol.fitness = 1.0 / (1.0 + (double)sol.cost); // 비용이 낮을수록 적합도가 높음
    }
}

void Scheduler::Crossover(vector<Chromosome>& population, vector<Chromosome>& offspring, double crossover_rate)
{
    auto _t0 = chrono::high_resolution_clock::now();
    int population_size = population.size();
    int num_tasks = population[0].task_seq.size();

    if(num_tasks < 2) {
        offspring = population; // 태스크가 1개 이하인 경우 교차 없이 그대로 복사
        return;
    }
    shuffle(population.begin(), population.end(), g);

    offspring.clear();
    for(int i = 0; i < population_size; i += 2) {
        offspring.push_back(population[i]); // 부모1
        offspring.push_back(population[i + 1]); // 부모2

        if((rand() / (double)RAND_MAX) >= crossover_rate) continue;

        int idx1, idx2;
        idx1 = rand() % (num_tasks - 1);
        idx2 = rand() % (num_tasks - idx1) + idx1;

        // Task sequence crossover
        for (int j = idx1; j <= idx2; j++) {
            offspring[i].task_seq[j] = population[i + 1].task_seq[j];
            offspring[i + 1].task_seq[j] = population[i].task_seq[j];
        }

        // Task sequence correction to ensure valid sequences (no duplicates)
        std::map<int, int> map1, map2;
        for (int j = idx1; j <= idx2; j++) {
            map1[offspring[i].task_seq[j]] = offspring[i + 1].task_seq[j];
            map2[offspring[i + 1].task_seq[j]] = offspring[i].task_seq[j];
        }

        for (int j = 0; j < num_tasks; j++) {
            if(j >= idx1 && j <= idx2)
                continue;

            int val1 = offspring[i].task_seq[j];
            while(map1.find(val1) != map1.end()) {
                val1 = map1[val1];
            }
            offspring[i].task_seq[j] = val1;

            int val2 = offspring[i + 1].task_seq[j];
            while(map2.find(val2) != map2.end()) {
                val2 = map2[val2];
            }
            offspring[i + 1].task_seq[j] = val2;
        }
    }
    g_tm.crossover_ms += chrono::duration<double, milli>(chrono::high_resolution_clock::now() - _t0).count();
    g_tm.crossover_calls++;
}

void Scheduler::Mutate_Task(vector<Chromosome>& population, double mutation_rate)
{
    int num_tasks = population[0].task_seq.size();
    if(num_tasks < 2) return; // 태스크가 1개 이하인 경우 돌연변이 없이 그대로 복사
    for (auto& sol : population) {
        if ((rand() / (double)RAND_MAX) < mutation_rate) {
            int idx1 = rand() % (num_tasks - 1);
            int idx2 = rand() % (num_tasks - idx1) + idx1;
            swap(sol.task_seq[idx1], sol.task_seq[idx2]); // Task sequence mutation (swap)
        }
    }
}

void Scheduler::Mutate_Robot(vector<Chromosome>& population, double mutation_rate)
{
    int num_tasks = population[0].task_seq.size();
    if(num_tasks < 1) return;
    const int robot_pool[4] = {1, 2, 4, 5};
    for (auto& sol : population) {
        if ((rand() / (double)RAND_MAX) < mutation_rate) {
            int idx = rand() % (num_tasks);
            sol.robot_assign[idx] = robot_pool[rand() % 4]; // 랜덤하게 로봇 할당
        }
    }
}


void Scheduler::Select(vector<Chromosome>& population, vector<Chromosome>& offspring)
{
    int population_size = population.size();

    vector<Chromosome> combined = population;
    combined.insert(combined.end(), offspring.begin(), offspring.end());
    population.clear();

    // 비용이 낮은 순으로 정렬
    sort(combined.begin(), combined.end());

    // Elitism
    population.push_back(combined[0]);

    // Roulette Wheel Selection
    double total_fitness = 0.0;
    for (const auto& sol : combined) {
        total_fitness += sol.fitness;
    }
    for (int i = 1; i < population_size; i++) {
        double pick = (rand() / (double)RAND_MAX) * total_fitness;
        double current = 0.0;
        for (const auto& sol : combined) {
            current += sol.fitness;
            if (current >= pick) {
                population.push_back(sol);
                break;
            }
        }
    }
}

int Scheduler::cost_function(const vector<vector<vector<int>>>& known_cost_map, const vector<shared_ptr<TASK>>& active_tasks, const vector<shared_ptr<ROBOT>>& robots, Chromosome& chromosome)
{
    int cost = 0;
    vector<vector<int>> sub_sol(robots.size()); // 각 로봇별로 할당된 태스크 시퀀스 저장
    chromosome.path.clear();
    chromosome.path.resize(robots.size());

    for (int i = 0; i < chromosome.task_seq.size(); i++) {
        sub_sol[chromosome.robot_assign[i]].push_back(chromosome.task_seq[i]);
    }

    for (int i = 0; i < sub_sol.size(); i++) {
        if (sub_sol[i].empty()) continue;
        // chromosome.path[i].push_back({robots[i]->get_coord().x, robots[i]->get_coord().y}); // 시작 위치 추가
        cost += Scheduler::calculate_cost(known_cost_map, active_tasks, robots, i, sub_sol[i], chromosome.path[i]);
    }
    return cost;
}

int Scheduler::calculate_cost(const vector<vector<vector<int>>>& known_cost_map, const vector<shared_ptr<TASK>>& active_tasks, const vector<shared_ptr<ROBOT>>& robots, const int robotId, const vector<int>& sol_seq, vector<pair<int, int>>& path)
{
    int cost = 0;
    pair<int, int> current_pos = { robots[robotId]->get_coord().x, robots[robotId]->get_coord().y };
    for (int taskId : sol_seq) {
        auto task = find_if(active_tasks.begin(), active_tasks.end(), [taskId](const shared_ptr<TASK>& t) {
            return t->id == taskId;
        });
        
        if (task == active_tasks.end()) continue; // (예외 처리) 태스크를 찾지 못한 경우
        
        pair<int, int> task_pos = { (*task)->coord.x, (*task)->coord.y };
        vector<pair<int, int>> sub_path;
        cost += Scheduler::calculate_distance(known_cost_map, current_pos, task_pos, static_cast<int>(robots[robotId]->type), sub_path);
        cost += (*task)->get_cost(robots[robotId]->type);

        if(!sub_path.empty()){
            path.insert(path.end(), sub_path.begin() + 1, sub_path.end());
        }
        current_pos = task_pos;
    }
    int penalty = 0;
    if(cost > robots[robotId]->get_energy()) penalty += (cost - robots[robotId]->get_energy()) * 1000;
    if(current_time + cost > max_time) penalty += (current_time + cost - max_time) * 100;
    return cost + penalty;
}



int Scheduler::calculate_distance(const vector<vector<vector<int>>>& grid, pair<int, int> start, pair<int, int> end, int robotType, vector<pair<int, int>>& path) {

    int n = grid.size();
    int m = grid[0].size();
    const int h_cost[3] = {0, 498, 796};

    vector<vector<int>> dist(n, vector<int>(m, numeric_limits<int>::max()));
    vector<vector<pair<int, int>>> parent(n, vector<pair<int, int>>(m, { -1, -1 }));
    priority_queue<DijkNode, vector<DijkNode>, greater<DijkNode>> pq;

    dist[start.first][start.second] = 0;
    pq.push({ 0, start.first, start.second });

    while (!pq.empty()) {
        DijkNode node = pq.top(); pq.pop();
        int x = node.x, y = node.y;

        if (x == end.first && y == end.second) break;

        for (int dir = 0; dir < 4; ++dir) {
            int nx = x + dx[dir], ny = y + dy[dir];
            if (nx >= 0 && ny >= 0 && nx < n && ny < m) {
                if (grid[nx][ny][robotType] == INFINITE) continue;

                int curr_cost = (grid[x][y][robotType] < 0) ? h_cost[robotType] : grid[x][y][robotType];
                int next_cost = (grid[nx][ny][robotType] < 0) ? h_cost[robotType] : grid[nx][ny][robotType];

                int cost = (curr_cost + next_cost) / 2;
                if (dist[x][y] + cost < dist[nx][ny]) {
                    dist[nx][ny] = dist[x][y] + cost;
                    parent[nx][ny] = { x, y };
                    pq.push({ dist[nx][ny], nx, ny });
                }
            }
        }
    }

    if (dist[end.first][end.second] == numeric_limits<int>::max()) {
        return 100000; // 경로 없음
    }

    path.clear();
    for (pair<int, int> at = end; at != make_pair(-1, -1); at = parent[at.first][at.second]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());
    return dist[end.first][end.second];
}

// MRTA/schedular.cpp 내부
// MRTA/schedular.cpp 내부

ROBOT::ACTION Scheduler::getLocalExplorationAction(
    const ROBOT& robot, 
    const vector<vector<vector<int>>>& known_cost_map, 
    const vector<vector<OBJECT>>& known_object_map) 
{
    Coord cur = robot.get_coord();
    int mapSize = known_object_map.size();
    int robotTypeIdx = static_cast<int>(robot.type);
    
    // 로봇 고유의 시야 범위와 형태 가져오기
    int viewRange = ROBOT::view_range_list[robotTypeIdx];
    ROBOT::VIEWTYPE viewType = ROBOT::view_type_list[robotTypeIdx];
    
    ROBOT::ACTION best_action = ROBOT::ACTION::HOLD;
    double best_score = -1.0; 

    // 핑퐁 방지를 위해 이전 위치 가져오기
    Coord prev_pos = {-1, -1};
    if (previous_positions.find(robot.id) != previous_positions.end()) {
        prev_pos = previous_positions[robot.id];
    }

    for (int i = 0; i < 4; i++) {
        int nx = cur.x + dx[i];
        int ny = cur.y + dy[i];
        Coord nextCoord(nx, ny);

        // 1. 당장 물리적으로 이동 가능한지 검사
        if (nx < 0 || nx >= mapSize || ny < 0 || ny >= mapSize) continue;
        if (nextCoord == prev_pos) continue;
        if (known_object_map[nx][ny] == OBJECT::WALL) continue;
        if (known_object_map[nx][ny] != OBJECT::UNKNOWN && 
            (known_cost_map[nx][ny][robotTypeIdx] < 0 || known_cost_map[nx][ny][robotTypeIdx] == INFINITE)) {
            continue;
        }

        // 2. nx, ny 위치로 이동했을 때 얻을 수 있는 시야의 총 가치(Information Gain) 평가
        double current_direction_score = 0.0;
        
        // 십자형(CROSS) 시야인 경우 (예: Wheel)
        if (viewType == ROBOT::VIEWTYPE::CROSS) {
            for (int vx = max(nx - viewRange, 0); vx <= min(nx + viewRange, mapSize - 1); ++vx) {
                int observed_time = last_observed_time_map[vx][ny];
                if (observed_time == -1) current_direction_score += 10000.0; // 미지(Unknown) 영역
                else current_direction_score += (current_time - observed_time); // 오래될수록 높은 점수
            }
            for (int vy = max(ny - viewRange, 0); vy <= min(ny + viewRange, mapSize - 1); ++vy) {
                if (vy == ny) continue; // 교차점 중복 합산 방지
                int observed_time = last_observed_time_map[nx][vy];
                if (observed_time == -1) current_direction_score += 10000.0;
                else current_direction_score += (current_time - observed_time);
            }
        } 
        // 정사각형(SQUARE) 시야인 경우 (예: Caterpillar)
        else if (viewType == ROBOT::VIEWTYPE::SQUARE) {
            for (int vx = max(nx - viewRange, 0); vx <= min(nx + viewRange, mapSize - 1); ++vx) {
                for (int vy = max(ny - viewRange, 0); vy <= min(ny + viewRange, mapSize - 1); ++vy) {
                    int observed_time = last_observed_time_map[vx][vy];
                    if (observed_time == -1) current_direction_score += 10000.0;
                    else current_direction_score += (current_time - observed_time);
                }
            }
        }

        // 가장 점수가 높은(즉, 가장 많은 Frontier와 미지 영역을 밝힐 수 있는) 방향 선택
        if (current_direction_score > best_score) {
            best_score = current_direction_score;
            best_action = getNextAction(cur, nextCoord);
        }
    }

    // 다음 이동을 위해 현재 위치 저장 (단, 갈 곳이 없어 HOLD인 경우 핑퐁 방지 초기화)
    if (best_action != ROBOT::ACTION::HOLD) {
        previous_positions[robot.id] = cur;
    } else {
        previous_positions[robot.id] = {-1, -1}; // 갇힘 방지 자가 치유
    }

    return best_action;
}