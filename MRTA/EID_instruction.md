# EID 기반 드론 탐색 전략 구현 지침

PDF 출처: `260513_이기종컴퓨터_아이디어_안성수_양동원.pdf` (pp. 3~7)

---

## 개요

기존 드론의 greedy scoring 방식을 **EID(Expected Information Density) 맵 기반 Beam Search 궤적 계획**으로 교체한다.

드론 2대가 EID가 높은 영역을 체계적으로 관측하며, 중복 탐색을 최소화하고 조건 충족 시 전체 재계획(Replanning)을 수행한다.

---

## 전체 흐름 (3단계)

```
Step 1: EID Map Generation        → 맵 전체 셀의 정보 가치를 수치화
Step 2: Drone Trajectory Planning → Beam Search로 모든 드론 궤적 순차 결정
Step 3: Periodic Replanning       → 벽 충돌 or 궤적 완수 시 전체 드론 재계획
```

---

## Step 1: EID Map Generation

### 핵심 원칙

EID[x][y]는 **셀 (x,y) 자체의 정보 가치**를 나타낸다 (per-cell 고유값).  
드론이 위치 p를 방문할 때 얻는 정보 이득은 Beam Search 단계에서 ±2 범위 합산으로 별도 계산한다.  
(EID를 per-cell 값으로 유지해야 5x5 관측 범위를 Beam Search에서 이중 집계 없이 정확히 반영 가능)

### EID[cell] 계산 — 셀 상태에 따른 분기

```
if WALL or unreachable:
    EID[x][y] = 0

elif UNKNOWN:
    EID[x][y] = w_unknown * unknownValue(found_tasks_count)
              + w_distance * minDistFromRobots(x, y)

else (known cell):
    staleness = (current_time - last_observed_time_map[x][y]) / max_time
    EID[x][y] = w_staleness * staleness
              + w_distance * minDistFromRobots(x, y)
```

**이유**: unknown 셀은 staleness가 의미 없고(한 번도 관측 안 됨), known 셀은 unknown factor가 없다. 두 상태를 섞으면 수식이 부자연스러워진다.

### 각 항 상세

#### `unknownValue(found_tasks_count)`
- PDF: "찾은 task의 개수에 따라 감소"
- 정확한 expected_total_tasks를 알 수 없으므로, 지금까지 발견된 task 누적 수로 근사:
  ```
  unknownValue(n) = 1.0 / (1.0 + n)
  ```
  - task를 하나도 못 찾았으면 1.0 (unknown 탐색 최우선)
  - task를 많이 찾을수록 unknown 셀의 가치가 감소
- `found_tasks_count`: `on_task_reached()`에서 드론 외 로봇이 완료한 task마다 +1 누적 (별도 멤버 변수)

#### `minDistFromRobots(x, y)`
- 가장 가까운 로봇(드론 포함)까지의 **유클리드 거리**를 맵 대각선 길이로 정규화 (0~1)
- BFS 비용 대비 충분히 빠르고 기존 `calculateEuclideanDistance` 재사용 가능

#### `staleness`
- `last_observed_time_map`은 기존 코드 그대로 재사용
- 한 번도 관측 안 된 셀(`last_observed_time_map[x][y] == -1`)은 UNKNOWN 분기에서 처리되므로 이 항에서는 발생하지 않음

### EID 맵 재계산 시점
- `on_info_updated()` 내에서 **`updated_coords`가 비어있지 않을 때만** 재계산
- 기존 `on_info_updated()`에서 `updated_coords` 분기 로직 참고

---

## Step 2: Drone Trajectory Planning

### 핵심 아이디어
드론이 **에너지 예산** 내에서 Beam Search로 궤적을 탐색하여, 5x5 관측 범위 기준 EID 합산이 최대가 되는 경로를 결정한다.

### 에너지 예산
```
budget = min(initialDroneEnergy * DRONE_ENERGY_BUDGET, robot.get_energy())
```

**재계획 시 trajectory 길이 축소 문제 해결**:  
`current_energy * ratio` 방식은 재계획마다 예산이 지수적으로 줄어든다 (예: 1000→200→40→8).  
`initialDroneEnergy * BUDGET_RATIO`를 고정 목표값으로 삼되, 실제 에너지가 부족하면 남은 에너지 전체를 쓰는 구조로 자연스럽게 해결된다.

### 알고리즘: Beam Search

- **Beam width** k: 매 스텝마다 상위 k개 궤적 유지
- **EID gain 계산 (5x5 관측 범위 반영)**:
  - 드론이 위치 p로 이동하면 ±2 범위 내 셀들을 관측
  - EID gain = 해당 범위 내 셀들의 EID 합산 (이미 이전 스텝에서 카운트한 셀 제외)
  - 각 beam은 `unordered_set<Coord> observed_set`을 유지하여 중복 제거 (정확한 방식)
- **종료 조건**: 에너지 예산 초과 or 유효 이웃 없음

```
BeamSearch(drone_pos, eid_map, budget, beam_width):
    // 각 beam: (path, energy_used, eid_sum, observed_set)
    beams = [([drone_pos], 0, 0, observe(drone_pos))]

    result_beams = []

    while beams not empty:
        next_beams = []
        for each (path, energy_used, eid_sum, observed) in beams:
            expanded = false
            for each neighbor of path[-1] (4방향, non-WALL):
                move_cost = known_cost_map[neighbor][DRONE]  // unknown이면 avgDroneCost * 1.5
                new_energy = energy_used + move_cost
                if new_energy > budget: continue
                expanded = true

                new_obs = observe_range(neighbor)  // ±2 범위 셀 집합
                gain = sum(eid_map[c] for c in new_obs if c not in observed)
                new_observed = observed ∪ new_obs

                next_beams.append((path + [neighbor], new_energy, eid_sum + gain, new_observed))

            if not expanded:
                result_beams.append((path, eid_sum))  // 더 이상 확장 불가

        if next_beams is empty: break
        beams = top-k next_beams by eid_sum
        // 마지막 라운드 최종 후보도 저장
        result_beams.extend([(b.path, b.eid_sum) for b in beams])

    return argmax(result_beams, key=eid_sum).path
```

**`observe(pos)`**: 위치 pos 한 곳의 ±2 범위 셀 집합 반환 (기존 `countCellsToReveal` 내 범위 로직 재사용)

### 순차적 드론 계획 (중복 탐색 방지)

```
buildEIDMap(...)                          // Step 1: EID 맵 계산
temp_eid_map = eid_map                    // 임시 맵 복사

for each drone in drones (순서대로):
    trajectory[drone] = BeamSearch(drone.pos, temp_eid_map, budget[drone], k)
    for each pos in trajectory[drone]:
        for each cell in observe_range(pos):
            temp_eid_map[cell] = 0        // 관측 예정 셀 EID 차감 → 다음 드론 중복 제거
```

---

## Step 3: Periodic Trajectory Replanning

### 재계획 트리거 조건 (OR) — **모든 드론 동시 재계획**
1. **벽 충돌**: 어떤 드론이든 궤적 상의 다음 셀이 벽으로 밝혀진 경우
2. **궤적 완수**: 어떤 드론이든 결정된 trajectory를 전부 완료한 경우

### 재계획 절차
1. EID 맵 재계산 (현재 맵 상태 기준)
2. 모든 드론의 현재 위치에서 Step 2 (순차 Beam Search) 다시 수행
3. 모든 드론의 trajectory 교체

---

## 기존 코드 변경 사항

### 제거 대상 (불필요)

| 함수/변수 | 이유 |
|-----------|------|
| `findBestDestination()` | Beam Search로 완전 대체 |
| `calculateBaseScore()` | EID 맵으로 흡수 |
| `oldBonusInPath()` | EID staleness 항으로 흡수 |
| `countCellsToReveal()` | Beam Search 내 `observe_range()` 로직으로 대체 |
| `calculateDroneDistanceScore()` | EID의 `minDistFromRobots` 항으로 흡수 |
| `calculateDirectionAlignmentScore()` | 제거 (방향 편향 불필요) |
| `calculateSpreadBonusScore()` | 제거 (EID distance 항이 대체) |
| `DronePathInfo::movesSinceLastDestUpdate` | 3스텝 재계획 트리거 제거 |
| `DronePathInfo::lastMovement` | 방향 점수 제거에 따라 불필요 |
| `DRONE_DISTANCE_WEIGHT` 등 기존 가중치 상수 | 새 파라미터로 교체 |
| HOLD 조건 (`energy < initialDroneEnergy/2`) | 제거 |

### 재사용 대상

| 함수/변수 | 재사용 방식 |
|-----------|------------|
| `last_observed_time_map` | EID staleness 항 계산에 그대로 사용 |
| `calculateEuclideanDistance()` | `minDistFromRobots` 계산에 재사용 |
| `findPathToTarget()` | 드론이 trajectory 각 waypoint까지 이동 경로 계산에 유지 |
| `pathHasWalls()` | 재계획 트리거 조건 감지에 유지 |
| `calculateAvgDroneCost()` | Beam Search의 unknown 셀 이동 비용 추정에 재사용 |
| `areAdjacent()`, `getNextAction()` | 그대로 유지 |
| `countCellsToReveal` 내 ±2 범위 순회 로직 | `observe_range()` 구현에 재사용 |

### `DronePathInfo` 구조체 변경

```cpp
struct DronePathInfo {
    vector<Coord> trajectory;   // Beam Search로 결정된 전체 궤적 (waypoints)
    vector<Coord> currentPath;  // 현재 waypoint까지의 세부 이동 경로 (findPathToTarget 결과)
    bool initialized;
    // 제거: movesSinceLastDestUpdate, lastMovement, targetDestination
};
```

- `trajectory`: 드론이 순서대로 방문할 waypoint 목록 (Beam Search 출력)
- `currentPath`: 현재 waypoint까지의 실제 이동 경로 (`findPathToTarget` 결과), `idle_action`에서 한 스텝씩 소비

---

## 추가 멤버 변수 (schedular.h)

```cpp
vector<vector<double>> eid_map;   // EID 맵 (per-cell 정보 가치)
int found_tasks_count;             // 완료된 task 누적 수 (on_task_reached에서 +1)
```

---

## 추가/수정 함수 목록

| 함수 | 신규/수정 | 설명 |
|------|-----------|------|
| `buildEIDMap(known_cost_map, known_object_map, robots)` | 신규 | EID 맵 전체 재계산 |
| `observeRange(pos)` | 신규 | 위치 pos의 ±2 범위 셀 집합 반환 |
| `beamSearchTrajectory(drone_pos, temp_eid, budget, k)` | 신규 | Beam Search → trajectory 반환 |
| `planAllDroneTrajectories(known_cost_map, known_object_map, robots)` | 신규 | 전체 드론 순차 계획 (temp_eid 공유) |
| `on_info_updated()` | 수정 | updated_coords 있을 때 EID 재계산 + 재계획 트리거 |
| `idle_action()` (drone 분기) | 수정 | trajectory 순서대로 이동, HOLD 조건 제거 |
| `initialize()` | 수정 | EID 맵 초기화, found_tasks_count = 0, HOLD 관련 코드 제거 |
| `on_task_reached()` | 수정 | 드론 외 로봇이 task 완료 시 found_tasks_count++ |

---

## 파라미터

```cpp
#define EID_UNKNOWN_WEIGHT    1.0    // unknown 셀 기여 가중치
#define EID_STALENESS_WEIGHT  0.5    // 오래된 known 셀 기여 가중치
#define EID_DISTANCE_WEIGHT   0.3    // 로봇 거리 기여 가중치
#define DRONE_ENERGY_BUDGET   0.2    // 궤적 계획 에너지 비율 (고정 기준: initialDroneEnergy * 0.2)
#define BEAM_WIDTH            5      // Beam Search 너비
```
