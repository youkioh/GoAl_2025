#include "schedular.h"
#include "simulator.h"
#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <algorithm>
#include <limits>

// #define DRONE_PATH_VISUALIZATION

#define DRONE_DISTANCE_WEIGHT 0.3
#define DRONE_DIRECTION_WEIGHT 0.2
#define DRONE_SPREAD_WEIGHT 0.1
#define DRONE_RANDOM_WEIGHT 0.001
#define DRONE_MODE_CHANGE_THRESHOLD 0.5 // 드론 모드 변경 임계값(%). 에너지가 임계값 이하면 drone의 모드 변경경
#define DROND_OLD_BONUS_WEIGHT 1 // drone이 이전 방문한 cell을 다시 방문할 때의 보너스 가중치

using namespace std;

// D* 탐색 알고리즘을 위한 간단한 구조체
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

    // 아직 초기화되지 않았으면 초기화
    if (!initialized) {
        initialize(known_cost_map, known_object_map, robots);
    }

    else {
        // 초기화 이후에는 매 호출마다 시간 증가
        current_time++;
        // 로봇 시야 내 모든 셀들의 시간 정보를 현재 시간으로 업데이트 (최근 방문 시간)
        for (const auto& coord : observed_coords) {  // updated_coords 대신 observed_coords 사용
            // 맵 경계 확인
            if (coord.x >= 0 && coord.x < last_observed_time_map.size() &&
                coord.y >= 0 && coord.y < last_observed_time_map.size()) {

                // 로봇 시야 내의 모든 셀의 마지막 발견 시간을 현재 시간으로 업데이트
                last_observed_time_map[coord.x][coord.y] = current_time;
            }
        }
    }

    // Drone 제외 robot schedule 활성화
    if (current_time >= max_time * 3 / 10 && !scheduled) {
        scheduled = true;
        //cout << "Robot Scheduling Start" << endl;
        schedule_tasks(known_cost_map, known_object_map, active_tasks, robots);
        //cout << "Robot Scheduling "<<n_sch++<<" Completed" << endl;
    }

    // 맵 정보가 업데이트된 경우 드론의 경로 재계산 (필요한 경우)
    if (!updated_coords.empty()) {
        // 업데이트 된 맵 정보에 task가 있는 경우 schedule 재진행
        if (scheduled) {
            for (auto uc : updated_coords)
            {
                if (known_object_map[uc.x][uc.y] == OBJECT::TASK)
                {
                    //cout << "Robot Scheduling Start" << endl;
                    schedule_tasks(known_cost_map, known_object_map, active_tasks, robots);
                    //cout << "Robot Scheduling "<<n_sch++<<" Completed" << endl;
                    break;
                }
            }
        }

        for (const auto& robot : robots) {
            if (robot->type == ROBOT::TYPE::DRONE) {
                auto& pathInfo = dronePaths[robot->id];

                // 드론이 초기화되었는지 확인
                if (!pathInfo.initialized) {
                    continue;
                }

                // 경로 재계산이 필요한지 확인
                bool needPathRecalculation = false;

                // 1. 목적지가 벽인지 확인
                if (known_object_map[pathInfo.targetDestination.x][pathInfo.targetDestination.y] == OBJECT::WALL) {
#ifdef DRONE_PATH_VISUALIZATION
                    cout << "목적지가 벽으로 확인됨. 새 목적지 설정." << endl;
#endif
                    needPathRecalculation = true;
                }
                // 2. 경로에 벽이 있는지 확인
                else if (!pathInfo.currentPath.empty() && pathHasWalls(pathInfo.currentPath, known_object_map)) {
#ifdef DRONE_PATH_VISUALIZATION
                    cout << "경로에 벽 발견: 목적지를 업데이트합니다." << endl;
#endif
                    needPathRecalculation = true;
                }

                // 경로 재계산이 필요한 경우
                if (needPathRecalculation) {
                    updateDroneDestination(
                        robot->get_coord(),
                        pathInfo,
                        known_cost_map,
                        known_object_map,
                        robots,
                        robot->id
                    );
#ifdef DRONE_PATH_VISUALIZATION
                    cout << "새 목적지 설정: ("
                        << pathInfo.targetDestination.x << ", "
                        << pathInfo.targetDestination.y << ")" << endl;
                    cout << "새 경로 길이: " << pathInfo.currentPath.size() << endl;
                    visualizeDronePaths(known_object_map, robots);
#endif
                }
            }
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
    // 원래 동작 유지 - 드론은 작업 수행 안 함
    return robot.type != ROBOT::TYPE::DRONE;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots,
    const ROBOT& robot) {

    // 드론 제외 로봇 simulated annealing 결과에 따라 action 할당
    if (robot.type != ROBOT::TYPE::DRONE) {
        Coord nextPos;
        if (scheduled) {
            for (auto& t : sol_path) {
                if (robot.id == t.first) {
                    if (t.second.empty()) return ROBOT::ACTION::HOLD;
                    nextPos = Coord(t.second.front().first, t.second.front().second);
                    t.second.erase(t.second.begin());
                }
            }
            return getNextAction(robot.get_coord(), nextPos);
        }
        else
            return ROBOT::ACTION::HOLD;
    }
    else {
        
        if (robot.get_energy() < initialDroneEnergy / 2  && current_time < max_time * 5 / 10) {
            return ROBOT::ACTION::HOLD; // 드론은 HOLD 액션
        }

        // if (robot.get_energy() < initialDroneEnergy / 2 && active_tasks.size() >= 4) {
        //     int totalEnergy = 0;
        //     int count = 0;
        //     for (auto& robot : robots) {
        //         if (robot->type != ROBOT::TYPE::DRONE) {
        //             count++;
        //             totalEnergy += robot->get_energy();
        //         }
        //     }
        //     int avgEnergy = totalEnergy / count;
            
        //     if (robot.get_energy() < avgEnergy) {
        //         return ROBOT::ACTION::HOLD;
        // }

// #ifdef DRONE_PATH_VISUALIZATION
//             cout << "드론 HOLD. 활성화된 작업 개수: " << active_tasks.size()
//                 << ", 에너지: " << robot.get_energy() << endl;
// #endif //DRONE_PATH_VISUALIZATION
//             return ROBOT::ACTION::HOLD; // 에너지가 절반 이하이고, 활성화된 작업이 4개 이상인 경우 대기
        // }

        // 이하 코드는 드론만 실행
        auto& pathInfo = dronePaths[robot.id];
        bool destinationReached = false;
        bool updateDestination = false;

        // 1. 목적지 업데이트가 필요한 상황인지 확인
        // 1-1. 현재 위치가 목적지인 경우
        if (robot.get_coord().x == pathInfo.targetDestination.x &&
            robot.get_coord().y == pathInfo.targetDestination.y) {
            destinationReached = true;
            updateDestination = true;
#ifdef DRONE_PATH_VISUALIZATION
            cout << "드론이 목적지에 도달했습니다. 새 목적지를 설정합니다." << endl;
#endif
        }
        // 1-2. 일정 횟수(10번) 이동 후 목적지 재설정
        else if (pathInfo.movesSinceLastDestUpdate >= 3) {
            updateDestination = true;
#ifdef DRONE_PATH_VISUALIZATION
            cout << "3번 이동 후 목적지를 재설정합니다." << endl;
#endif
        }
        // 1-3. 경로가 비어있는 경우
        else if (pathInfo.currentPath.empty()) {
            updateDestination = true;
#ifdef DRONE_PATH_VISUALIZATION
            cout << "경로가 비어있어 목적지를 재설정합니다." << endl;
#endif
        }

        // 2. 목적지 업데이트가 필요한 경우
        if (updateDestination) {
            updateDroneDestination(robot.get_coord(), pathInfo, known_cost_map, known_object_map, robots, robot.id);

#ifdef DRONE_PATH_VISUALIZATION
            cout << "새 목적지 설정: ("
                << pathInfo.targetDestination.x << ", "
                << pathInfo.targetDestination.y << ")" << endl;
            cout << "새 경로 길이: " << pathInfo.currentPath.size() << endl;
            visualizeDronePaths(known_object_map, robots);
#endif
        }

        // 3. 경로가 비어있거나 현재 위치만 포함하는 경우
        bool invalidPath = pathInfo.currentPath.empty() ||
            (pathInfo.currentPath.size() == 1 &&
                pathInfo.currentPath[0].x == robot.get_coord().x &&
                pathInfo.currentPath[0].y == robot.get_coord().y);

        if (invalidPath) {
            // 목적지에 도달하지 않았다면 경로만 재계산
            if (!destinationReached) {
                pathInfo.currentPath = findPathToTarget(
                    robot.get_coord(),
                    pathInfo.targetDestination,
                    known_cost_map,
                    known_object_map
                );
            }

            // 여전히 경로가 유효하지 않다면 제자리에 있기
            invalidPath = pathInfo.currentPath.empty() ||
                (pathInfo.currentPath.size() == 1 &&
                    pathInfo.currentPath[0].x == robot.get_coord().x &&
                    pathInfo.currentPath[0].y == robot.get_coord().y);

            if (invalidPath) {
                return ROBOT::ACTION::HOLD;
            }
        }
        // 4. 경로가 유효한 경우 다음 위치로 이동
        if (pathInfo.currentPath.size() >= 2) {
            Coord nextPos = pathInfo.currentPath[1];

            // 참고: on_info_updated 함수에서 이미 경로에 벽이 있는지 확인하고 재계산하므로
            // 여기서 다시 확인할 필요가 없습니다.
            // 이동 방향 계산 및 이동 실행
            Coord direction(nextPos.x - robot.get_coord().x, nextPos.y - robot.get_coord().y);
            pathInfo.lastMovement = direction;

            // 이동 횟수 증가
            pathInfo.movesSinceLastDestUpdate++;

            // 경로에서 현재 위치 제거
            pathInfo.currentPath.erase(pathInfo.currentPath.begin());

#ifdef DRONE_PATH_VISUALIZATION
            cout << "드론 " << robot.id << "가 " << robot.get_coord()
                << "에서 " << nextPos << "로 이동. 남은 경로 길이: "
                << pathInfo.currentPath.size() << ", 목적지까지 이동 횟수: "
                << pathInfo.movesSinceLastDestUpdate << endl;
#endif

            // 다음 위치로 이동하는 액션 반환
            return getNextAction(robot.get_coord(), nextPos);
        }

        // 기본 동작: 제자리에 있기
        return ROBOT::ACTION::HOLD;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////Drone Section./////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void Scheduler::initialize(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {
    // 초기화 완료 표시
    initialized = true;

    // known_object_map의 가로 길이 * 100 = max_time
    max_time = known_object_map.size() * 100;
    initialDroneEnergy = robots[0]->get_energy(); // 드론의 초기 에너지 설정

    // 현재 시간 초기화
    current_time = 0;

    // 맵 크기 가져오기
    int mapSize = known_object_map.size();

    // last_observed_time_map 초기화 (-1: 아직 방문하지 않음)
    last_observed_time_map.resize(mapSize, vector<int>(mapSize, -1));

    // 초기에 이미 알려진 셀들의 시간을 0으로 설정
    for (int x = 0; x < mapSize; x++) {
        for (int y = 0; y < mapSize; y++) {
            if (known_object_map[x][y] != OBJECT::UNKNOWN) {
                last_observed_time_map[x][y] = 0;
            }
        }
    }

    // 드론의 평균 이동 비용 계산
    avgDroneCost = calculateAvgDroneCost(known_cost_map);

    // 각 드론의 경로 초기화
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE) {
            DronePathInfo pathInfo;
            pathInfo.initialized = true;
            pathInfo.movesSinceLastDestUpdate = 0;
            pathInfo.lastMovement = Coord(0, 0); // 초기에는 이동방향 없음

            // 드론의 경로 정보 초기화
            updateDroneDestination(
                robot->get_coord(),
                pathInfo,
                known_cost_map,
                known_object_map,
                robots,
                robot->id
            );

            // 드론의 경로 정보 저장
            dronePaths[robot->id] = pathInfo;
        }
    }

    // 초기 드론 경로 시각화
    visualizeDronePaths(known_object_map, robots);

    // Robot scheduling 비활성화
    scheduled = false;
    n_sch = 1;
}

// 맨해튼 거리 계산
int Scheduler::calculateManhattanDistance(const Coord& a, const Coord& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// 유클리드 거리 계산
double Scheduler::calculateEuclideanDistance(const Coord& a, const Coord& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

// 셀이 알려지지 않은 셀인지 확인
bool Scheduler::isUnknownCell(const Coord& coord, const vector<vector<OBJECT>>& known_object_map) {
    int mapSize = known_object_map.size();
    if (coord.x < 0 || coord.x >= mapSize || coord.y < 0 || coord.y >= mapSize) {
        return false; // 맵 바깥은 알려지지 않은 셀이 아님
    }
    return known_object_map[coord.x][coord.y] == OBJECT::UNKNOWN;
}

// 경로에 벽이 있는지 확인
bool Scheduler::pathHasWalls(const vector<Coord>& path, const vector<vector<OBJECT>>& known_object_map) {
    for (const auto& coord : path) {
        // 경로 상의 각 좌표가 맵 범위 내에 있고 벽인지 확인
        int mapSize = known_object_map.size();
        if (coord.x >= 0 && coord.x < mapSize && coord.y >= 0 && coord.y < mapSize) {
            if (known_object_map[coord.x][coord.y] == OBJECT::WALL) {
                return true; // 벽을 발견하면 즉시 true 반환
            }
        }
    }    return false; // 벽이 없으면 false 반환
}

// 드론의 목적지와 경로 업데이트 헬퍼 함수
void Scheduler::updateDroneDestination(const Coord& dronePosition,
    DronePathInfo& pathInfo,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots,
    int droneId) {
    // 새 목적지 찾기
    pathInfo.targetDestination = findBestDestination(
        dronePosition,
        pathInfo.lastMovement,
        known_cost_map,
        known_object_map,
        robots,
        droneId
    );

    // 새 경로 찾기
    pathInfo.currentPath = findPathToTarget(
        dronePosition,
        pathInfo.targetDestination,
        known_cost_map,
        known_object_map
    );

    // 이동 횟수 초기화
    pathInfo.movesSinceLastDestUpdate = 0;
}

// D* 경로에서 알려지지 않은 셀 수 계산
int Scheduler::countUnknownCellsInPath(const vector<Coord>& path,
    const vector<vector<OBJECT>>& known_object_map) {
    int count = 0;
    for (const auto& coord : path) {
        if (isUnknownCell(coord, known_object_map)) {
            count++;
        }
    }
    return count;
}

double Scheduler::countCellsToReveal(const vector<Coord>& path,
    const vector<vector<OBJECT>>& known_object_map) {
    double score = 0.0;

    set<Coord> CellsToObserve;

    // path를 순회하며 observe하게 되는 모든 cell을 CellsToObserve에 추가
    for (const auto& coord : path) {
        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                Coord neighbor(coord.x + dx, coord.y + dy);

                // 맵 범위 내에 있는지 확인
                if (neighbor.x >= 0 && neighbor.x < known_object_map.size() &&
                    neighbor.y >= 0 && neighbor.y < known_object_map.size()) {
                    // 알려지지 않은 셀인 경우
                    if (isUnknownCell(neighbor, known_object_map)) {
                        CellsToObserve.insert(neighbor);
                    }
                }
            }
        }
    }

    // CellsToObserve에 있는 셀들에서 unknown 셀의 개수를 세어 점수 계산
    for (const auto& coord : CellsToObserve) {
        if (isUnknownCell(coord, known_object_map)) {
            score += 1.0;
        }
    }

    return score;
}

double Scheduler::oldBonusInPath(const vector<Coord>& path,
    const vector<vector<OBJECT>>& known_object_map) {
    double bonus = 0.0;
    set<Coord> CellsToObserve;

    // path를 순회하며 observe하게 되는 모든 cell을 CellsToObserve에 추가
    for (const auto& coord : path) {
        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                Coord neighbor(coord.x + dx, coord.y + dy);

                // 맵 범위 내에 있는지 확인
                if (neighbor.x >= 0 && neighbor.x < known_object_map.size() &&
                    neighbor.y >= 0 && neighbor.y < known_object_map.size()) {
                    // 알려지지 않은 셀인 경우
                    if (isUnknownCell(neighbor, known_object_map)) {
                        CellsToObserve.insert(neighbor);
                    }
                }
            }
        }
    }

    // CellsToObserve에 있는 셀들의 점수 계산
    for (const auto& coord : CellsToObserve) {
        // 셀의 마지막 관찰 시간 가져오기
        int lastObservedTime = last_observed_time_map[coord.x][coord.y];

        // 아직 관찰하지 않은 셀은 0점으로 처리
        if (lastObservedTime == -1) {
            bonus += 0.0;
        }
        else {
            // 보너스 점수 계산 (현재 시간과 마지막 관찰 시간 차이)
            bonus += (current_time - lastObservedTime) / static_cast<double>(max_time);
        }
    }

    return bonus;
}

// 각 셀의 기본 탐색 점수 계산 (새로 밝히는 셀 수 / 거리)
double Scheduler::calculateBaseScore(const Coord& position,
    const Coord& dronePosition,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map) {
    // 목적지까지의 경로 찾기
    vector<Coord> path = findPathToTarget(dronePosition, position, known_cost_map, known_object_map);

    // 경로가 없으면 점수는 0
    if (path.empty()) {
        return 0.0;
    }

    // 경로에서 알려지지 않은 셀 수 계산
    double newCells = countCellsToReveal(path, known_object_map);

    double oldBonus = 0.0;

    // drone의 모드가 변경되면, 추가적인 점수 부여.
    if (current_time >= max_time / DRONE_MODE_CHANGE_THRESHOLD) {
        oldBonus += oldBonusInPath(path, known_object_map);
    }

    // 경로의 길이
    double pathLength = static_cast<double>(path.size()) + oldBonus;

    // 경로 길이가 0이면 점수는 0
    if (pathLength == 0) {
        return 0.0;
    }

    // 알려지지 않은 셀 수를 경로 길이로 나눔
    return (newCells + DROND_OLD_BONUS_WEIGHT * oldBonus) / pathLength;
}

// 다른 드론과의 거리 점수 계산
double Scheduler::calculateDroneDistanceScore(const Coord& position,
    const vector<shared_ptr<ROBOT>>& robots,
    int currentDroneId) {
    double minDistance = INFINITE;
    double mapSize = 0;

    // 맵 크기 추정 (첫 번째 드론의 위치를 기준으로)
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE) {
            Coord pos = robot->get_coord();
            mapSize = max(mapSize, static_cast<double>(pos.x));
            mapSize = max(mapSize, static_cast<double>(pos.y));
        }
    }
    mapSize = max(mapSize * 2, 20.0); // 맵 크기를 대략적으로 추정

    // 현재 드론을 제외한 다른 드론과의 최소 거리 찾기
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE && robot->id != currentDroneId) {
            double distance = calculateEuclideanDistance(position, robot->get_coord());
            minDistance = min(minDistance, distance);
        }
    }

    // 거리가 무한대면 다른 드론이 없거나 모두 현재 드론과 같은 위치
    if (minDistance == INFINITE) {
        return 0.0;
    }

    // 최소 거리를 맵 크기로 정규화 (0~1 사이의 값으로)
    return min(1.0, minDistance / mapSize);
}

// 이전 이동 방향과의 일치도 점수 계산
double Scheduler::calculateDirectionAlignmentScore(const Coord& position,
    const Coord& dronePosition,
    const Coord& lastMovement) {
    // 이전 이동이 없으면 점수는 0.5 (중립)
    if (lastMovement.x == 0 && lastMovement.y == 0) {
        return 0.5;
    }

    // 현재 드론 위치에서 목표 위치로 가는 맨해튼 거리
    int currentManhattan = calculateManhattanDistance(dronePosition, position);

    // 이전 이동 방향으로 한 칸 이동했을 때의 위치 계산
    Coord nextPos = Coord(dronePosition.x + lastMovement.x, dronePosition.y + lastMovement.y);

    // 이동 후 위치에서 목표 위치로 가는 맨해튼 거리
    int nextManhattan = calculateManhattanDistance(nextPos, position);

    // 거리가 줄어들면 1, 아니면 0
    return (nextManhattan < currentManhattan) ? 1.0 : 0.0;
}

// 맵 중심에서의 확산 보너스 점수 계산
double Scheduler::calculateSpreadBonusScore(const Coord& position,
    const vector<vector<OBJECT>>& known_object_map) {
    int mapSize = known_object_map.size();

    // 맵 중심 좌표 계산
    Coord center = Coord(mapSize / 2, mapSize / 2);

    // 중심에서의 유클리드 거리 계산
    double distance = calculateEuclideanDistance(position, center);

    // 맵 대각선 길이의 절반 (정규화를 위한 최대 거리)
    double maxDistance = sqrt(2.0) * mapSize / 2.0;

    // 거리를 0~1 사이의 값으로 정규화 (중심에서 멀수록 높은 점수)
    return min(1.0, distance / maxDistance);
}

// 최적의 목적지 찾기
Coord Scheduler::findBestDestination(const Coord& dronePosition,
    const Coord& lastMovement,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots,
    int droneId) {
    int mapSize = known_cost_map.size();
    vector<CellScore> candidateCells;

    // 랜덤 요소를 위한 난수 생성기 초기화
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);

    // 스캔 범위는 드론 기준 상하좌우 5칸씩 총 11x11 범위
    int scanRange = 5;
    int startX = max(2, dronePosition.x - scanRange);
    int startY = max(2, dronePosition.y - scanRange);
    int endX = min(mapSize - 2, dronePosition.x + scanRange);
    int endY = min(mapSize - 2, dronePosition.y + scanRange);

    // 맵 전체를 스캔하면서 후보 셀 선정
    for (int x = startX; x < endX; x++) {
        for (int y = startY; y < endY; y++) {
            Coord position(x, y);

            // 벽이나 작업인 셀은 제외
            if (known_object_map[x][y] == OBJECT::WALL ||
                known_object_map[x][y] == OBJECT::TASK) {
                continue;
            }

            // 현재 위치는 제외
            if (x == dronePosition.x && y == dronePosition.y) {
                continue;
            }

            // 목적지까지의 경로 찾기
            vector<Coord> path = findPathToTarget(dronePosition, position, known_cost_map, known_object_map);

            // 경로가 없거나 경로에 알려지지 않은 셀이 5개 이상이면 제외
            if (path.empty() || countUnknownCellsInPath(path, known_object_map) >= 5) {
                continue;
            }

            // 각 요소별 점수 계산
            double baseScore = calculateBaseScore(position, dronePosition, known_cost_map, known_object_map);
            double droneDistanceScore = calculateDroneDistanceScore(position, robots, droneId);
            double directionAlignmentScore = calculateDirectionAlignmentScore(position, dronePosition, lastMovement);
            double spreadBonusScore = calculateSpreadBonusScore(position, known_object_map);

            // 랜덤 요소 (0~1 사이의 값)
            double randomFactor = dis(gen);

            // 최종 점수 계산
            double finalScore = baseScore +
                DRONE_DISTANCE_WEIGHT * droneDistanceScore +
                DRONE_DIRECTION_WEIGHT * directionAlignmentScore +
                DRONE_SPREAD_WEIGHT * spreadBonusScore +
                DRONE_RANDOM_WEIGHT * randomFactor;

            // 후보 셀에 추가
            candidateCells.emplace_back(position, finalScore);
        }
    }

    // 후보 셀이 없으면 현재 위치 반환
    if (candidateCells.empty()) {
        return dronePosition;
    }

    // 점수가 가장 높은 셀 찾기
    auto bestCell = max_element(candidateCells.begin(), candidateCells.end(),
        [](const CellScore& a, const CellScore& b) {
            return a.score < b.score;
        });

#ifdef DRONE_PATH_VISUALIZATION
    // bestCell의 점수 표시
    cout << "Best cell: " << bestCell->position << ", Score: " << bestCell->score << endl;
#endif

    return bestCell->position;
}

vector<Coord> Scheduler::findPathToTarget(const Coord& start,
    const Coord& target,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map) {
    int mapSize = known_cost_map.size();

    // 시작점과 목표점이 유효한지 확인
    if (start.x < 0 || start.y < 0 || start.x >= mapSize || start.y >= mapSize ||
        target.x < 0 || target.y < 0 || target.x >= mapSize || target.y >= mapSize) {
        return {};
    }

    // 방향: 위, 오른쪽, 아래, 왼쪽
    const int dx[] = { 0, 1, 0, -1 };
    const int dy[] = { -1, 0, 1, 0 };

    // D* 검색을 위한 우선순위 큐
    priority_queue<Node, vector<Node>, greater<Node>> openSet;

    // 방문한 노드 및 경로 재구성을 위한 부모 포인터
    vector<vector<bool>> visited(mapSize, vector<bool>(mapSize, false));
    vector<vector<Coord>> parent(mapSize, vector<Coord>(mapSize, Coord(-1, -1)));
    vector<vector<int>> gScore(mapSize, vector<int>(mapSize, INFINITE));

    // 목표 노드부터 시작 (D*는 역방향으로 검색)
    openSet.push(Node(target, 0));
    gScore[target.x][target.y] = 0;

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        Coord currentCoord = current.coord;

        // 이미 방문한 경우 건너뛰기
        if (visited[currentCoord.x][currentCoord.y]) {
            continue;
        }

        // 방문했다고 표시
        visited[currentCoord.x][currentCoord.y] = true;

        // 시작점에 도달했는지 확인
        if (currentCoord.x == start.x && currentCoord.y == start.y) {
            break;
        }

        // 이웃 탐색
        for (int i = 0; i < 4; i++) {
            int nx = currentCoord.x + dx[i];
            int ny = currentCoord.y + dy[i];
            // 이웃이 유효한지 확인
            if (nx >= 0 && nx < mapSize && ny >= 0 && ny < mapSize &&
                !visited[nx][ny] && known_object_map[nx][ny] != OBJECT::WALL) {

                int moveCost;
                // 셀이 알려진 경우 실제 비용 사용 (비용이 유효한지도 확인)
                if (known_object_map[nx][ny] != OBJECT::UNKNOWN &&
                    known_cost_map[nx][ny][static_cast<int>(ROBOT::TYPE::DRONE)] > 0 &&
                    known_cost_map[nx][ny][static_cast<int>(ROBOT::TYPE::DRONE)] < INFINITE) {
                    moveCost = known_cost_map[nx][ny][static_cast<int>(ROBOT::TYPE::DRONE)];
                }
                else {
                    // 알려지지 않은 영역 또는 비용이 유효하지 않은 경우에는 평균 드론 이동 비용의 1.5배 사용
                    moveCost = static_cast<int>(avgDroneCost * 1.5);
                }

                int newCost = gScore[currentCoord.x][currentCoord.y] + moveCost;

                if (newCost < gScore[nx][ny]) {
                    gScore[nx][ny] = newCost;
                    parent[nx][ny] = currentCoord;
                    openSet.push(Node(Coord(nx, ny), newCost));
                }
            }
        }
    }

    // 시작점에서 목표까지의 경로 재구성
    vector<Coord> path;
    Coord current = start;

    // 경로를 찾았는지 확인
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
    // 현재 위치와 다음 위치를 기반으로 이동 방향 결정
    if (current.x < next.x) {
        return ROBOT::ACTION::RIGHT;
    }
    else if (current.x > next.x) {
        return ROBOT::ACTION::LEFT;
    }
    else if (current.y < next.y) {
        return ROBOT::ACTION::UP;    // y가 증가하면 위로 이동
    }
    else if (current.y > next.y) {
        return ROBOT::ACTION::DOWN;  // y가 감소하면 아래로 이동
    }

    // 유효한 이동이 없으면 제자리에 있기
#ifdef DRONE_PATH_VISUALIZATION
    cout << "Drone is holding at (" << current.x << ", " << current.y << ")" << endl;
#endif
    return ROBOT::ACTION::HOLD;
}

int Scheduler::calculateAvgDroneCost(const vector<vector<vector<int>>>& known_cost_map) {
    // 알려진 셀에서 드론 평균 이동 비용 계산
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

    // 평균 반환, 유효한 비용이 없으면 기본값 사용
    return (count > 0) ? (totalCost / count) : 100;
}

bool Scheduler::areAdjacent(const Coord& a, const Coord& b) {
    // 두 좌표가 인접해 있는지 확인
    int dx = abs(a.x - b.x);
    int dy = abs(a.y - b.y);
    return (dx == 1 && dy == 0) || (dx == 0 && dy == 1);
}

void Scheduler::visualizeDronePaths(const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {
#ifdef DRONE_PATH_VISUALIZATION
    // 현재 시간 출력
    cout << "현재 시뮬레이션 시간: " << current_time << endl;

    int mapSize = known_object_map.size();

    // 맵 복사본 생성 (시각화를 위해)
    vector<vector<char>> visualMap(mapSize, vector<char>(mapSize, ' '));

    // 알려진 객체 맵 기본 시각화
    for (int y = 0; y < mapSize; y++) {
        for (int x = 0; x < mapSize; x++) {
            if (known_object_map[x][y] == OBJECT::WALL) {
                visualMap[x][y] = '#';  // 벽
            }
            else if (known_object_map[x][y] == OBJECT::UNKNOWN) {
                visualMap[x][y] = '?';  // 미지 지역
            }
            else if (known_object_map[x][y] == OBJECT::EMPTY) {
                visualMap[x][y] = '.';  // 빈 공간
            }
            else if (known_object_map[x][y] == OBJECT::TASK) {
                visualMap[x][y] = 'T';  // 작업
            }
        }
    }

    // 드론 위치와 번호 표시
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE) {
            Coord pos = robot->get_coord();
            visualMap[pos.x][pos.y] = static_cast<char>(robot->id);  // 드론 위치

            // 드론 번호도 표시
            cout << "드론 " << robot->id << " 위치: (" << pos.x << ", " << pos.y << ")" << endl;
        }
        if (robot->type == ROBOT::TYPE::CATERPILLAR) {
            Coord pos = robot->get_coord();
            visualMap[pos.x][pos.y] = static_cast<char>(robot->id);  // 캐터필러 로봇 위치
        }
        if (robot->type == ROBOT::TYPE::WHEEL) {
            Coord pos = robot->get_coord();
            visualMap[pos.x][pos.y] = static_cast<char>(robot->id);  // 휠 로봇 위치
        }
    }

    // 각 드론의 경로와 목적지 표시
    for (const auto& pair : dronePaths) {
        int droneId = pair.first;
        const DronePathInfo& pathInfo = pair.second;

        // 목적지 표시
        if (pathInfo.initialized) {
            const Coord& dest = pathInfo.targetDestination;
            // 목적지는 'G'로 표시 (드론이 아닌 경우)
            if (visualMap[dest.x][dest.y] != 'D') {
                visualMap[dest.x][dest.y] = 'G';  // 목적지
            }
        }

        // 현재 경로 표시
        for (const Coord& pathPoint : pathInfo.currentPath) {
            // 경로상의 지점이 드론 또는 목적지가 아닌 경우만 표시
            if (visualMap[pathPoint.x][pathPoint.y] != 'D' &&
                visualMap[pathPoint.x][pathPoint.y] != 'G') {
                visualMap[pathPoint.x][pathPoint.y] = '*';  // 경로 지점
            }
        }

        // // 드론의 정보 출력
        // cout << "드론 " << droneId << " 정보:" << endl;

        // // 현재 목표 목적지 출력
        // if (pathInfo.initialized) {
        //     cout << " - 목적지: ("
        //         << pathInfo.targetDestination.x << ", "
        //         << pathInfo.targetDestination.y << ")" << endl;

        //     // 마지막 이동 방향 출력 (있는 경우)
        //     if (pathInfo.lastMovement.x != 0 || pathInfo.lastMovement.y != 0) {
        //         cout << " - 마지막 이동 방향: ("
        //             << pathInfo.lastMovement.x << ", "
        //             << pathInfo.lastMovement.y << ")" << endl;
        //     }

        //     // 현재 이동 횟수 출력
        //     cout << " - 마지막 목적지 갱신 이후 이동 횟수: "
        //         << pathInfo.movesSinceLastDestUpdate << endl;
        // }

        // // 현재 경로 길이 출력
        // cout << " - 현재 경로 길이: " << pathInfo.currentPath.size() << endl;
    }

    // 맵 시각화 출력
    cout << "맵 시각화:" << endl;

    // 맵 격자 및 행 번호 출력 (y축 역순으로 - 위에서 아래로)
    for (int y = mapSize - 1; y >= 0; y--) {
        cout << setw(2) << y << " ";
        for (int x = 0; x < mapSize; x++) {
            cout << visualMap[x][y] << " ";
        }
        cout << endl;
    }

    // 열 번호 출력
    cout << "  ";
    for (int x = 0; x < mapSize; x++) {
        cout << setw(2) << x;
    }
    cout << endl;

    // // 범례 출력
    // cout << "범례: " << endl;
    // cout << " - D: 드론" << endl;
    // cout << " - G: 목적지" << endl;
    // cout << " - *: 경로 지점" << endl;
    // cout << " - #: 벽" << endl;
    // cout << " - ?: 미지 지역" << endl;
    // cout << " - .: 빈 공간" << endl;
    // cout << " - T: 작업" << endl;
    // cout << endl;
#endif // DRONE_PATH_VISUALIZATION
}


///////////////////////////////////////////////////////////////////////////////////////
//////////////////Task Scheduling Section./////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// solution sequence mixing
vector<Scheduler::Point> Scheduler::shuffle_point(vector<Scheduler::Point> array)
{
    random_device rd;                  // OS로부터 시드 받기
    mt19937 g(rd());                   // Mersenne Twister 엔진 생성
    shuffle(array.begin(), array.end(), g);
    return array;
}

// two-opt swap 사용
vector<Scheduler::Point> Scheduler::two_opt_swap(const vector<Point>& route, int i, int k) {
    vector<Point> new_route;

    // 1. i 이전 구간
    new_route.insert(new_route.end(), route.begin(), route.begin() + i);

    // 2. i ~ k 구간 (역순)
    for (int idx = k; idx >= i; --idx)
        new_route.push_back(route[idx]);

    // 3. k 이후 구간
    new_route.insert(new_route.end(), route.begin() + k + 1, route.end());

    return new_route;
}

// 발견된 task를 scheduling
void Scheduler::schedule_tasks(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots)
{
    int cost;
    vector<Point> array;

    //if(sol_seq.empty()){
    // active tasks, robots 기반으로 초기 solution sequence 생성
    for (int i = 0; i < active_tasks.size(); i++) {
        Point k; k.x = active_tasks[i]->coord.x; k.y = active_tasks[i]->coord.y;
        k.cost[0] = active_tasks[i]->get_cost(ROBOT::TYPE::CATERPILLAR);
        k.cost[1] = active_tasks[i]->get_cost(ROBOT::TYPE::WHEEL);
        k.type = 2;
        k.id = active_tasks[i]->id;
        k.energy = 0;
        array.push_back(k);
    }
    for (int i = 0; i < robots.size(); i++) {
        if (robots[i]->type == ROBOT::TYPE::DRONE) continue;
        Point k; k.x = robots[i]->get_coord().x; k.y = robots[i]->get_coord().y;
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
    //}
    /*
    else{
        for(int i = 0 ; i < active_tasks.size() ; i++){
            bool new_task = true;
            for(auto item : sol_seq){
                if(active_tasks[i]->id == item.id && item.type == 2){
                    new_task = false;
                }
            }
            if(!new_task) continue;
            Point k; k.x=active_tasks[i]->coord.x; k.y=active_tasks[i]->coord.y;
            k.cost[0]=active_tasks[i]->get_cost(ROBOT::TYPE::CATERPILLAR);
            k.cost[1]=active_tasks[i]->get_cost(ROBOT::TYPE::WHEEL);
            k.type=2;
            k.id=active_tasks[i]->id;
            k.energy=0;

            srand(time(nullptr));
            int index = rand() % (sol_seq.size() - 1) + 1;
            sol_seq.insert(sol_seq.begin()+ index, k);
        }
    }*/
    int cnt = 0;
    // simulated annealing 수행
    do
    {
        sol_seq = simulated_annealing(array, known_cost_map, sol_path, cost);
        //cout<<"best cost: "<<cost<<endl;
        cnt++; if (cnt > 4) break;
    } while (cost >= 100000); // 일정 코스트 이상인 경우 simulated annealing 재실행
}

vector<Scheduler::Point> Scheduler::simulated_annealing(
    const vector<Scheduler::Point>& initial_points, const vector<vector<vector<int>>>& known_cost_map, vector<pair<int, vector<pair<int, int>>>>& path, int& cost,
    double start_temp,
    double end_temp,
    double cooling_rate,
    int max_iterations
) {
    vector<Scheduler::Point> current = initial_points;
    vector<Scheduler::Point> best = current;
    vector<pair<int, vector<pair<int, int>>>> current_path;
    int current_cost = Scheduler::cost_function(known_cost_map, current, current_path);
    int best_cost = current_cost;
    path = current_path;
    /*
    for(int i = 0 ; i < current.size() ; i++){
        cout<<"Type: "<<current[i].type<<" ID: "<< current[i].id<<endl;
    }*/
    double T = start_temp;
    srand(time(nullptr));

    for (int iter = 0; iter < max_iterations && T > end_temp; ++iter) {
        vector<Point> next = current;
        current_path.clear();

        // two-opt swap
        if (rand() % 2) {
            int i = rand() % next.size();
            int j = rand() % next.size();
            while (i == 0) i = rand() % next.size();
            while (i == j || j == 0) j = rand() % next.size();
            if (i > j) swap(i, j);
            //swap(next[i], next[j]);
            next = two_opt_swap(next, i, j);
        }
        // 인접 원소 swap
        else {
            int i = rand() % (next.size() - 1);
            while (i == 0) i = rand() % (next.size() - 1);
            swap(next[i], next[i + 1]);
        }

        int next_cost = Scheduler::cost_function(known_cost_map, next, current_path);
        int delta = next_cost - current_cost;

        if (delta < 0 || exp(-delta / T) >(rand() / (double)RAND_MAX)) {
            current = next;
            current_cost = next_cost;

            if (current_cost < best_cost) {
                best = current;
                best_cost = current_cost;
                path = current_path;
                //cout<<"current cost ("<<iter<<"): "<<best_cost<<endl;
            }
        }
        T *= cooling_rate;
    }

    /*
    for(int i = 0 ; i < best.size() ; i++){
        cout<<"Type: "<<best[i].type<<" ID: "<<best[i].id<<endl;
    }

    for(auto i : path)
    {
        cout<<"Path "<<i.first<<endl;
        for(auto j : i.second){
            cout<<"("<<j.first<<","<<j.second<<")";
        }
        cout<<endl;
    }*/
    cost = best_cost;
    return best;
}

// sub-array별로 cost 계산 함수 호출
int Scheduler::cost_function(const vector<vector<vector<int>>>& known_cost_map, vector<Point> array, vector<pair<int, vector<pair<int, int>>>>& path) {
    vector<Point> sub_array;
    vector<pair<int, int>> sub_path;
    int cost = 0;
    int robot_id = array[0].id;

    sub_array.push_back(array[0]);
    for (int i = 1; i < array.size(); i++) {
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

// sub-array의 cost 계산
int Scheduler::calculate_cost(const vector<vector<vector<int>>>& known_cost_map, vector<Point> array, vector<pair<int, int>>& path) {
    int cost = 0;
    vector<pair<int, int>> sub_path;
    for (int i = 0; i < array.size() - 1; i++) {
        int dist = 0;
        dist = Scheduler::calculate_distance(known_cost_map, make_pair(array[i].x, array[i].y), make_pair(array[i + 1].x, array[i + 1].y), array[0].type, sub_path);
        cost += dist;
        cost += array[i + 1].cost[array[0].type];
        if (dist >= 100000) continue;

        if (i != array.size() - 2) {
            sub_path.pop_back();
            path.insert(path.end(), sub_path.begin(), sub_path.end());
        }
        else path.insert(path.end(), sub_path.begin(), sub_path.end());
    }

    if (cost > array[0].energy) return 100000 + cost;
    if (cost <= 0) return 1000;
    return cost;
}

// Dijkstra 거리 계산
int Scheduler::calculate_distance(const vector<vector<vector<int>>>& grid, pair<int, int> start, pair<int, int> end, int robotType, vector<pair<int, int>>& path) {
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