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

#define DRONE_PATH_VISUALIZATION

#define EID_UNKNOWN_WEIGHT    1.0   // unknown 셀 기여 가중치
#define EID_STALENESS_WEIGHT  0.5   // 오래된 known 셀 기여 가중치
#define EID_DISTANCE_WEIGHT   0.3   // 로봇 거리 기여 가중치
#define DRONE_ENERGY_BUDGET   0.2   // 궤적 계획 에너지 비율 (initialDroneEnergy 기준)
#define BEAM_WIDTH            5     // Beam Search 너비

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
    if (current_time >= max_time * 3 / 10 && !scheduled) {
        scheduled = true;
        schedule_tasks(known_cost_map, known_object_map, active_tasks, robots);
    }

    if (!updated_coords.empty()) {
        // 새 task 발견 시 robot re-scheduling
        if (scheduled) {
            for (auto uc : updated_coords) {
                if (known_object_map[uc.x][uc.y] == OBJECT::TASK) {
                    schedule_tasks(known_cost_map, known_object_map, active_tasks, robots);
                    break;
                }
            }
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
        for (auto& t : sol_path) {
            if (robot.id == t.first) {
                if (t.second.empty()) return ROBOT::ACTION::HOLD;
                nextPos = Coord(t.second.front().first, t.second.front().second);
                t.second.erase(t.second.begin());
            }
        }
        return getNextAction(robot.get_coord(), nextPos);
    }

    // 이하 드론만 실행
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
    n_sch = 1;
}

// EID 맵 전체 재계산
// EID[x][y]: 셀 (x,y) 자체의 정보 가치 (per-cell)
// 드론이 위치 p를 방문할 때의 정보 이득은 ±2 범위 합산으로 별도 계산 (beamSearchTrajectory 내)
void Scheduler::buildEIDMap(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {

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

    if (beams.empty()) return { dronePos };

    auto best = max_element(beams.begin(), beams.end(),
        [](const Beam& a, const Beam& b) { return a.eid_sum < b.eid_sum; });

    return best->path;
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

        // 에너지 예산: initialDroneEnergy * 20% 고정 목표, 실제 에너지로 캡핑
        int budget = min(
            static_cast<int>(initialDroneEnergy * DRONE_ENERGY_BUDGET),
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

vector<Scheduler::Point> Scheduler::shuffle_point(vector<Scheduler::Point> array)
{
    random_device rd;
    mt19937 g(rd());
    shuffle(array.begin(), array.end(), g);
    return array;
}

vector<Scheduler::Point> Scheduler::two_opt_swap(const vector<Point>& route, int i, int k) {
    vector<Point> new_route;
    new_route.insert(new_route.end(), route.begin(), route.begin() + i);
    for (int idx = k; idx >= i; --idx)
        new_route.push_back(route[idx]);
    new_route.insert(new_route.end(), route.begin() + k + 1, route.end());
    return new_route;
}

void Scheduler::schedule_tasks(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots)
{
    int cost;
    vector<Point> array;

    for (int i = 0; i < (int)active_tasks.size(); i++) {
        Point k;
        k.x = active_tasks[i]->coord.x; k.y = active_tasks[i]->coord.y;
        k.cost[0] = active_tasks[i]->get_cost(ROBOT::TYPE::CATERPILLAR);
        k.cost[1] = active_tasks[i]->get_cost(ROBOT::TYPE::WHEEL);
        k.type = 2;
        k.id = active_tasks[i]->id;
        k.energy = 0;
        array.push_back(k);
    }
    for (int i = 0; i < (int)robots.size(); i++) {
        if (robots[i]->type == ROBOT::TYPE::DRONE) continue;
        Point k;
        k.x = robots[i]->get_coord().x; k.y = robots[i]->get_coord().y;
        k.cost[0] = 0; k.cost[1] = 0;
        k.type = int(robots[i]->type) - 1;
        k.id = robots[i]->id;
        k.energy = robots[i]->get_energy();
        array.push_back(k);
    }
    while (true) {
        srand(time(nullptr));
        array = Scheduler::shuffle_point(array);
        if (array[0].type != 2) break;
    }

    int cnt = 0;
    do {
        sol_seq = simulated_annealing(array, known_cost_map, sol_path, cost);
        cnt++; if (cnt > 4) break;
    } while (cost >= 100000);
}

vector<Scheduler::Point> Scheduler::simulated_annealing(
    const vector<Scheduler::Point>& initial_points,
    const vector<vector<vector<int>>>& known_cost_map,
    vector<pair<int, vector<pair<int, int>>>>& path,
    int& cost,
    double start_temp,
    double end_temp,
    double cooling_rate,
    int max_iterations)
{
    vector<Scheduler::Point> current = initial_points;
    vector<Scheduler::Point> best = current;
    vector<pair<int, vector<pair<int, int>>>> current_path;
    int current_cost = Scheduler::cost_function(known_cost_map, current, current_path);
    int best_cost = current_cost;
    path = current_path;

    double T = start_temp;
    srand(time(nullptr));

    for (int iter = 0; iter < max_iterations && T > end_temp; ++iter) {
        vector<Point> next = current;
        current_path.clear();

        if (rand() % 2) {
            int i = rand() % next.size();
            int j = rand() % next.size();
            while (i == 0) i = rand() % next.size();
            while (i == j || j == 0) j = rand() % next.size();
            if (i > j) swap(i, j);
            next = two_opt_swap(next, i, j);
        }
        else {
            int i = rand() % (next.size() - 1);
            while (i == 0) i = rand() % (next.size() - 1);
            swap(next[i], next[i + 1]);
        }

        int next_cost = Scheduler::cost_function(known_cost_map, next, current_path);
        int delta = next_cost - current_cost;

        if (delta < 0 || exp(-delta / T) > (rand() / (double)RAND_MAX)) {
            current = next;
            current_cost = next_cost;

            if (current_cost < best_cost) {
                best = current;
                best_cost = current_cost;
                path = current_path;
            }
        }
        T *= cooling_rate;
    }

    cost = best_cost;
    return best;
}

int Scheduler::cost_function(const vector<vector<vector<int>>>& known_cost_map,
    vector<Point> array,
    vector<pair<int, vector<pair<int, int>>>>& path) {

    vector<Point> sub_array;
    vector<pair<int, int>> sub_path;
    int cost = 0;
    int robot_id = array[0].id;

    sub_array.push_back(array[0]);
    for (int i = 1; i < (int)array.size(); i++) {
        if (array[i].type < 2) {
            cost += Scheduler::calculate_cost(known_cost_map, sub_array, sub_path);
            path.push_back({ robot_id, sub_path });
            sub_array.clear();
            sub_array.push_back(array[i]);
            sub_path.clear();
            robot_id = array[i].id;
            continue;
        }
        sub_array.push_back(array[i]);
    }
    cost += Scheduler::calculate_cost(known_cost_map, sub_array, sub_path);
    path.push_back({ robot_id, sub_path });

    return cost;
}

int Scheduler::calculate_cost(const vector<vector<vector<int>>>& known_cost_map,
    vector<Point> array,
    vector<pair<int, int>>& path) {

    int cost = 0;
    vector<pair<int, int>> sub_path;
    for (int i = 0; i < (int)array.size() - 1; i++) {
        int dist = 0;
        dist = Scheduler::calculate_distance(known_cost_map,
            make_pair(array[i].x, array[i].y),
            make_pair(array[i + 1].x, array[i + 1].y),
            array[0].type, sub_path);
        cost += dist;
        cost += array[i + 1].cost[array[0].type];
        if (dist >= 100000) continue;

        if (i != (int)array.size() - 2) {
            sub_path.pop_back();
            path.insert(path.end(), sub_path.begin(), sub_path.end());
        }
        else path.insert(path.end(), sub_path.begin(), sub_path.end());
    }

    if (cost > array[0].energy) return 100000 + cost;
    if (cost <= 0) return 1000;
    return cost;
}

int Scheduler::calculate_distance(const vector<vector<vector<int>>>& grid,
    pair<int, int> start, pair<int, int> end,
    int robotType, vector<pair<int, int>>& path) {

    int n = grid.size();
    int m = grid[0].size();
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
                //cout<<nx<<", "<<ny<<endl;
                if (grid[nx][ny][robotType + 1] == INFINITE) continue;
                if (grid[nx][ny][robotType + 1] < 0) continue;
                int cost = (grid[x][y][robotType + 1] + grid[nx][ny][robotType + 1]) / 2;
                //cout<<"dist[x][y]: "<<dist[x][y]<<" cost: "<<cost<<" dist[nx][ny]: "<<dist[nx][ny]<<endl;
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