#include "schedular.h"

// D* 탐색 알고리즘을 위한 간단한 구조체
struct Node {
    Coord coord;
    int cost;

    Node(Coord c, int g) : coord(c), cost(g) {}

    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

void Scheduler::initialize(const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {
    // 초기화 완료 표시
    initialized = true;

    // 맵 크기 계산
    int mapSize = known_cost_map.size();

    // 드론의 평균 이동 비용 계산
    avgDroneCost = calculateAvgDroneCost(known_cost_map);

    // 드론에 지역 할당 (좌측과 우측 번갈아가며)
    int regionAssignment = 0;
    // 각 드론의 경로 초기화
    for (const auto& robot : robots) {
        if (robot->type == ROBOT::TYPE::DRONE) {
            DronePathInfo pathInfo;
            pathInfo.initialized = true;
            pathInfo.currentWaypointIndex = 0;
            pathInfo.assignedRegion = regionAssignment;

            // 드론을 위한 탐색 웨이포인트 생성
            pathInfo.waypoints = generateWaypoints(mapSize, regionAssignment);

            // 벽인 곳에 있는 웨이포인트 제거
            vector<Coord> validWaypoints;
            for (const Coord& wp : pathInfo.waypoints) {
                if (known_object_map[wp.x][wp.y] != OBJECT::WALL) {
                    validWaypoints.push_back(wp);
                }
#ifdef DRONE_PATH_VISUALIZATION
                else {
                    cout << "웨이포인트 제거 (벽): (" << wp.x << ", " << wp.y << ")" << endl;
                }
#endif
            }
            pathInfo.waypoints = validWaypoints;

            // 첫 웨이포인트까지의 경로 찾기
            if (!pathInfo.waypoints.empty()) {
                pathInfo.currentPath = findPathToTarget(
                    robot->get_coord(),
                    pathInfo.waypoints[0],
                    known_cost_map,
                    known_object_map
                );
            }            // 드론의 경로 정보 저장
            dronePaths[robot->id] = pathInfo;

            // 다음 드론을 위한 지역 할당 번갈아가며 설정
            regionAssignment = (regionAssignment + 1) % 2;
        }
    }

    // 초기 드론 경로 시각화
    visualizeDronePaths(known_object_map, robots);
}

vector<Coord> Scheduler::generateWaypoints(int mapSize, int assignedRegion) {
    vector<Coord> waypoints;

    // 맵을 5x5 그리드로 나눔
    const int grid_divisions = 5; // 5x5 그리드
    const int cell_size = mapSize / grid_divisions;

    // 할당된 지역에 따라 경계 결정 (왼쪽/오른쪽 반으로 나누기)
    int startGridX = (assignedRegion == 0) ? 0 : grid_divisions / 2;
    int endGridX = (assignedRegion == 0) ? grid_divisions / 2 - 1 : grid_divisions - 1;

    // 지그재그 패턴으로 웨이포인트 생성
    // 홀수 행은 왼쪽->오른쪽, 짝수 행은 오른쪽->왼쪽
    for (int gridY = 0; gridY < grid_divisions; gridY++) {
        // 왼쪽->오른쪽 또는 오른쪽->왼쪽 결정
        bool leftToRight = (gridY % 2 == 0);
        
        // 현재 행의 x 범위 설정
        int startX = leftToRight ? startGridX : endGridX;
        int endX = leftToRight ? endGridX : startGridX;
        int step = leftToRight ? 1 : -1;
        
        // 이 행의 웨이포인트 추가
        for (int gridX = startX; leftToRight ? (gridX <= endX) : (gridX >= endX); gridX += step) {
            // 각 그리드 셀의 중심점 계산
            int centerX = gridX * cell_size + cell_size / 2;
            int centerY = gridY * cell_size + cell_size / 2;

            // 맵 경계 내에 있는지 확인
            if (centerX < mapSize && centerY < mapSize) {
                waypoints.emplace_back(centerX, centerY);
            }
        }
    }

#ifdef DRONE_PATH_VISUALIZATION
    cout << "생성된 웨이포인트 (지역 " << assignedRegion << "):" << endl;
    for (const auto& wp : waypoints) {
        cout << " - (" << wp.x << ", " << wp.y << ")" << endl;
    }
#endif

    return waypoints;
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
    cout << "Drone " << current.x << " is holding at (" << current.x << ", " << current.y << ")" << endl;
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

void Scheduler::on_info_updated(const set<Coord>& observed_coords,
    const set<Coord>& updated_coords,
    const vector<vector<vector<int>>>& known_cost_map,
    const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<TASK>>& active_tasks,
    const vector<shared_ptr<ROBOT>>& robots) {
    // 맵 정보가 업데이트된 경우 드론의 경로 재계산
    if (!updated_coords.empty()) {
        for (const auto& robot : robots) {
            if (robot->type == ROBOT::TYPE::DRONE && robot->get_status() == ROBOT::STATUS::IDLE) {
                auto& pathInfo = dronePaths[robot->id];                // 드론이 초기화되었고 현재 웨이포인트가 있는 경우에만 업데이트
                if (pathInfo.initialized && pathInfo.currentWaypointIndex < pathInfo.waypoints.size()) {
                    // 현재 웨이포인트가 벽인지 확인하고 필요하면 웨이포인트 목록 갱신
                    if (!pathInfo.waypoints.empty()) {
                        // 현재 및 남은 웨이포인트들 중 벽인 것들을 제거
                        vector<Coord> validWaypoints;
                        bool currentWaypointRemoved = false;
                        
                        for (size_t i = pathInfo.currentWaypointIndex; i < pathInfo.waypoints.size(); i++) {
                            const Coord& wp = pathInfo.waypoints[i];
                            if (known_object_map[wp.x][wp.y] != OBJECT::WALL) {
                                validWaypoints.push_back(wp);
                            } else {
#ifdef DRONE_PATH_VISUALIZATION
                                cout << "웨이포인트 제거 (벽 발견): (" << wp.x << ", " << wp.y << ")" << endl;
#endif
                                if (i == pathInfo.currentWaypointIndex) {
                                    currentWaypointRemoved = true;
                                }
                            }
                        }
                        
                        // 이전 웨이포인트는 그대로 유지
                        vector<Coord> newWaypoints;
                        for (size_t i = 0; i < pathInfo.currentWaypointIndex; i++) {
                            newWaypoints.push_back(pathInfo.waypoints[i]);
                        }
                        
                        // 유효한 웨이포인트 추가
                        for (const auto& wp : validWaypoints) {
                            newWaypoints.push_back(wp);
                        }
                        
                        // 웨이포인트 목록 갱신
                        pathInfo.waypoints = newWaypoints;
                        
                        // 현재 웨이포인트가 제거되었으면 인덱스 조정 불필요 (아래 로직에서 empty 체크하므로)
                        // 그렇지 않으면 현재 인덱스 유지
                        if (currentWaypointRemoved && !pathInfo.waypoints.empty() && 
                            pathInfo.currentWaypointIndex >= pathInfo.waypoints.size()) {
                            pathInfo.currentWaypointIndex = pathInfo.waypoints.size() - 1;
                        }
                    }
                    
                    // 웨이포인트가 남아있는지 확인
                    if (pathInfo.waypoints.empty() || pathInfo.currentWaypointIndex >= pathInfo.waypoints.size()) {
#ifdef DRONE_PATH_VISUALIZATION
                        cout << "모든 웨이포인트가 벽으로 판명됨. 탐색 중단." << endl;
#endif
                        // 웨이포인트가 없으면 빈 경로 설정
                        pathInfo.currentPath.clear();
                    } else {
                        // 현재 웨이포인트까지의 경로 재계산                    
                        pathInfo.currentPath = findPathToTarget(
                            robot->get_coord(),
                            pathInfo.waypoints[pathInfo.currentWaypointIndex],
                            known_cost_map,
                            known_object_map
                        );
                        
                        // 경로를 찾지 못했다면 다음 웨이포인트로 넘어감
                        if (pathInfo.currentPath.empty() && pathInfo.currentWaypointIndex < pathInfo.waypoints.size() - 1) {
#ifdef DRONE_PATH_VISUALIZATION
                            cout << "경로를 찾을 수 없음: 웨이포인트("
                                << pathInfo.waypoints[pathInfo.currentWaypointIndex].x << ", "
                                << pathInfo.waypoints[pathInfo.currentWaypointIndex].y << ")는 도달할 수 없음. 다음 웨이포인트로 이동합니다." << endl;
#endif
                            // 다음 웨이포인트로 이동
                            pathInfo.currentWaypointIndex++;
                            
                            // 새 웨이포인트까지의 경로 찾기
                            pathInfo.currentPath = findPathToTarget(
                                robot->get_coord(),
                                pathInfo.waypoints[pathInfo.currentWaypointIndex],
                                known_cost_map,
                                known_object_map
                            );
                        }
                    }

                    // 경로에 벽이 포함되어 있는지 확인
                    if (!pathInfo.currentPath.empty() && pathInfo.currentPath.size() >= 2) {
                        Coord nextPos = pathInfo.currentPath[1];
                        if (known_object_map[nextPos.x][nextPos.y] == OBJECT::WALL) {
#ifdef DRONE_PATH_VISUALIZATION
                            cout << "경로에 벽 발견: 경로를 재계산합니다." << endl;
#endif
                            // 경로 재계산
                            pathInfo.currentPath = findPathToTarget(
                                robot->get_coord(),
                                pathInfo.waypoints[pathInfo.currentWaypointIndex],
                                known_cost_map,
                                known_object_map
                            );
                        }
                    }
                }
            }
        }

        // 경로가 업데이트된 후 시각화
#ifdef DRONE_PATH_VISUALIZATION
//         cout << "\n==================== MAP UPDATED ====================\n" << endl;
//         visualizeDronePaths(known_object_map, robots);
#endif
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
    // 아직 초기화되지 않았으면 초기화
    if (!initialized) {
        initialize(known_cost_map, known_object_map, robots);
    }

    // 드론의 경우, 탐색 경로 따르기
    if (robot.type == ROBOT::TYPE::DRONE) {
        auto& pathInfo = dronePaths[robot.id];
        bool waypointReached = false;

        // 드론에 경로가 없거나 현재 웨이포인트에 도달한 경우
        if (pathInfo.currentPath.empty() ||
            (pathInfo.currentPath.size() == 1 &&
                pathInfo.currentPath[0].x == robot.get_coord().x &&
                pathInfo.currentPath[0].y == robot.get_coord().y)) {

            // 웨이포인트 도달 플래그 설정
            waypointReached = true;            // 다음 웨이포인트로 이동 (있는 경우)
            pathInfo.currentWaypointIndex++;

            // 아직 웨이포인트가 남아있으면
            if (pathInfo.currentWaypointIndex < pathInfo.waypoints.size()) {
#ifdef DRONE_PATH_VISUALIZATION
                cout << "\n============= WAYPOINT REACHED =============\n" << endl;
                cout << "Drone " << robot.id << " reached waypoint "
                    << pathInfo.currentWaypointIndex - 1 << " of "
                    << pathInfo.waypoints.size() - 1 << endl;
                cout << "Moving to next waypoint: ("
                    << pathInfo.waypoints[pathInfo.currentWaypointIndex].x << ", "
                    << pathInfo.waypoints[pathInfo.currentWaypointIndex].y << ")\n" << endl;
#endif

                // 다음 웨이포인트까지의 경로 찾기
                pathInfo.currentPath = findPathToTarget(
                    robot.get_coord(),
                    pathInfo.waypoints[pathInfo.currentWaypointIndex],
                    known_cost_map,
                    known_object_map
                );
            }
            else {
#ifdef DRONE_PATH_VISUALIZATION
                cout << "\n============= EXPLORATION COMPLETE =============\n" << endl;
                cout << "Drone " << robot.id << " completed all waypoints. Holding position." << endl;
#endif
                // 모든 웨이포인트 탐색 완료, 제자리에 있기
                return ROBOT::ACTION::HOLD;
            }
        }
        // 경로가 최소 두 지점 이상 있는 경우 유효
        if (pathInfo.currentPath.size() >= 2) {
            // 현재 위치는 경로의 첫 번째, 다음 위치는 두 번째
            Coord nextPos = pathInfo.currentPath[1];

            // 다음 위치가 벽인지 확인
            if (known_object_map[nextPos.x][nextPos.y] == OBJECT::WALL) {
#ifdef DRONE_PATH_VISUALIZATION
                cout << "Drone " << robot.id << " 경로상 벽 발견: (" << nextPos.x << ", " << nextPos.y
                    << "). 경로 재계산 중..." << endl;
#endif                // 경로 재계산
                pathInfo.currentPath = findPathToTarget(
                    robot.get_coord(),
                    pathInfo.waypoints[pathInfo.currentWaypointIndex],
                    known_cost_map,
                    known_object_map
                );                // 경로를 찾지 못했다면 다음 웨이포인트로 넘어감
                if (pathInfo.currentPath.empty() || pathInfo.currentPath.size() < 2) {
#ifdef DRONE_PATH_VISUALIZATION
                    cout << "경로를 찾을 수 없음: 다음 웨이포인트로 넘어갑니다." << endl;
#endif
                    pathInfo.currentWaypointIndex++;

                    // 모든 웨이포인트를 탐색했는지 확인
                    if (pathInfo.currentWaypointIndex >= pathInfo.waypoints.size()) {
#ifdef DRONE_PATH_VISUALIZATION
                        cout << "\n============= EXPLORATION COMPLETE =============\n" << endl;
                        cout << "Drone " << robot.id << " completed all waypoints. Holding position." << endl;
#endif
                        return ROBOT::ACTION::HOLD;
                    }

                    // 다음 웨이포인트까지의 경로 찾기
                    pathInfo.currentPath = findPathToTarget(
                        robot.get_coord(),
                        pathInfo.waypoints[pathInfo.currentWaypointIndex],
                        known_cost_map,
                        known_object_map
                    );

                    // 새 경로도 유효하지 않다면 제자리에 머무름
                    if (pathInfo.currentPath.size() < 2) {
                        return ROBOT::ACTION::HOLD;
                    }

                    // 새 경로의 다음 위치 설정
                    nextPos = pathInfo.currentPath[1];
                }
                else {
                    // 새 경로의 다음 위치 설정
                    nextPos = pathInfo.currentPath[1];
                }
            }

            // 경로에서 현재 위치 제거
            pathInfo.currentPath.erase(pathInfo.currentPath.begin());

#ifdef DRONE_PATH_VISUALIZATION
            // 웨이포인트에 도달한 경우 또는 맵이 업데이트된 경우에만 전체 시각화
            if (waypointReached || !updated_coords.empty()) {
                visualizeDronePaths(known_object_map, robots);
            }
            else {
                // 움직임만 표시
                cout << "Drone " << robot.id << " moving from " << robot.get_coord()
                    << " to " << nextPos << ", remaining path length: "
                    << pathInfo.currentPath.size() << endl;
            }
#endif

            // 다음 위치로 이동하는 액션 반환
            return getNextAction(robot.get_coord(), nextPos);
        }
        else {
            // 경로가 비어있거나 현재 위치만 포함하는 경우 제자리에 있기
            return ROBOT::ACTION::HOLD;
        }
    }
    else {
        // 드론이 아닌 로봇은 무작위로 이동
        // ROBOT::ACTION action = static_cast<ROBOT::ACTION>(rand() % 5);
        ROBOT::ACTION action = ROBOT::ACTION::HOLD;
        return action;
    }
}

void Scheduler::visualizeDronePaths(const vector<vector<OBJECT>>& known_object_map,
    const vector<shared_ptr<ROBOT>>& robots) {
#ifdef DRONE_PATH_VISUALIZATION
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
            visualMap[pos.x][pos.y] = 'D';  // 드론 위치

            // 드론 번호도 표시
            cout << "Drone " << robot->id << " position: (" << pos.x << ", " << pos.y << ")" << endl;
        }
    }

    // 각 드론의 경로와 웨이포인트 표시
    for (const auto& pair : dronePaths) {
        int droneId = pair.first;
        const DronePathInfo& pathInfo = pair.second;

        // 웨이포인트 위치 표시
        for (size_t i = 0; i < pathInfo.waypoints.size(); i++) {
            const Coord& wp = pathInfo.waypoints[i];

            // 현재 목표 웨이포인트는 'W'로, 나머지는 'w'로 표시
            if (i == pathInfo.currentWaypointIndex) {
                visualMap[wp.x][wp.y] = 'W';  // 현재 웨이포인트
            }
            else if (visualMap[wp.x][wp.y] != 'D' && visualMap[wp.x][wp.y] != 'W') {
                visualMap[wp.x][wp.y] = 'w';  // 다른 웨이포인트
            }
        }

        // 현재 경로 표시 (웨이포인트로 가는 경로)
        for (const Coord& pathPoint : pathInfo.currentPath) {
            // 경로상의 지점이 드론, 현재 웨이포인트, 다른 웨이포인트가 아닌 경우만 표시
            if (visualMap[pathPoint.x][pathPoint.y] != 'D' &&
                visualMap[pathPoint.x][pathPoint.y] != 'W' &&
                visualMap[pathPoint.x][pathPoint.y] != 'w') {
                visualMap[pathPoint.x][pathPoint.y] = '*';  // 경로 지점
            }
        }

        // 드론의 정보 출력
        cout << "Drone " << droneId << " (Region " << pathInfo.assignedRegion << "):" << endl;
        cout << " - Current waypoint index: " << pathInfo.currentWaypointIndex
            << " of " << pathInfo.waypoints.size() << endl;

        // 현재 목표 웨이포인트 출력
        if (pathInfo.currentWaypointIndex < pathInfo.waypoints.size()) {
            cout << " - Target waypoint: ("
                << pathInfo.waypoints[pathInfo.currentWaypointIndex].x << ", "
                << pathInfo.waypoints[pathInfo.currentWaypointIndex].y << ")" << endl;
        }

        // 현재 경로 길이 출력
        cout << " - Current path length: " << pathInfo.currentPath.size() << endl;
    }

    // 맵 시각화 출력
    cout << "Map Visualization:" << endl;


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

    // 범례 출력
    cout << "Legend: " << endl;
    cout << " - D: Drone" << endl;
    cout << " - W: Current waypoint" << endl;
    cout << " - w: Other waypoint" << endl;
    cout << " - *: Path point" << endl;
    cout << " - #: Wall" << endl;
    cout << " - ?: Unknown area" << endl;
    cout << " - .: Empty space" << endl;
    cout << " - T: Task" << endl;
    cout << endl;
#endif // DRONE_PATH_VISUALIZATION
}