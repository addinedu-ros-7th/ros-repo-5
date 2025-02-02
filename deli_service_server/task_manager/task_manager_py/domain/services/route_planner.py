# task_manager_py/domain/services/route_planner.py

import math
import random
import math

# ------------------------------------------------------------
# 거리 행렬 (예시)
# "출발지", "목적지"를 포함해 각 스테이션 간 거리를 정의
# ------------------------------------------------------------
distance_matrix = {
    "냉동":   {"냉동": 0, "신선": 4, "일반": 1, "목적지": 2, "출발지": 4},
    "신선":   {"냉동": 4, "신선": 0, "일반": 2, "목적지": 4, "출발지": 2},
    "일반":   {"냉동": 1, "신선": 2, "일반": 0, "목적지": 3, "출발지": 4},
    "목적지": {"냉동": 2, "신선": 4, "일반": 3, "목적지": 0, "출발지": 1},
    "출발지": {"냉동": 4, "신선": 2, "일반": 4, "목적지": 1, "출발지": 0},
}

# ------------------------------------------------------------
# 스테이션 방문 가중치 (예시)
# ------------------------------------------------------------
order_weight = {
    "냉동": [5, 3, 1],
    "신선": [3, 2, 1],
    "일반": [1, 2, 3],
}

# ------------------------------------------------------------
# 기타 패널티 상수
# ------------------------------------------------------------
FIRST_VISIT_OCCUPIED_PENALTY = 40       # 스테이션 점유 중인데 remain_time이 0이면 부여되는 큰 패널티
BATTERY_PENALTY_FACTOR = 5.0            # 배터리 잔량이 적을수록 비용 증가시키는 계수

def calculate_route_cost(stations_to_visit, occupied_info, battery_level):
    """
    스테이션 방문 순서에 따른 총 비용(cost)을 계산하는 함수.
    비용이 낮을수록 좋은 경로.

    :param stations_to_visit: 실제로 방문해야 하는 스테이션들의 리스트 (예: ["냉동", "신선", "일반"]).
    :param occupied_info: 매대(스테이션) 점유 정보 딕셔너리
                          예) {
                              "냉동": {"robot": (False, 0), "person": (False, 0)},
                              "신선": {"robot": (False, 0), "person": (False, 0)},
                              ...
                          }
    :param battery_level: 로봇 배터리 잔량 (0.0 ~ 1.0)
    :return: float 형태의 비용 값
    """
    start_station = "출발지"
    total_cost = 0.0

    # 1) 방문할 스테이션이 없다면: 출발지->목적지 거리 + 배터리 페널티만 적용
    if not stations_to_visit:
        total_cost += distance_matrix[start_station]["목적지"]
        total_cost += (1.0 - battery_level) * BATTERY_PENALTY_FACTOR
        return total_cost

    # 2) 출발지 -> 첫 스테이션 이동 비용
    first_st = stations_to_visit[0]
    total_cost += distance_matrix[start_station][first_st]

    # 3) 첫 스테이션 점유 여부 체크
    r_occ, r_time = occupied_info[first_st]["robot"]
    p_occ, p_time = occupied_info[first_st]["person"]
    total_occ = (r_occ or p_occ)  # 로봇 혹은 사람이 점유중인지
    total_remain = max(r_time, p_time)

    # remain_time > 0 이면 대기 비용, remain_time == 0 이지만 점유중이면 큰 패널티
    if total_occ:
        if total_remain > 0:
            total_cost += total_remain
        else:
            total_cost += FIRST_VISIT_OCCUPIED_PENALTY

    # 4) 각 스테이션을 방문할 때 부여되는 가중치
    for i, st in enumerate(stations_to_visit):
        if st in order_weight:
            idx = min(i, len(order_weight[st]) - 1)
            total_cost += order_weight[st][idx]

    # 5) 스테이션들 간 이동 비용
    for i in range(len(stations_to_visit) - 1):
        cur_st = stations_to_visit[i]
        nxt_st = stations_to_visit[i+1]
        total_cost += distance_matrix[cur_st][nxt_st]

    # 6) 마지막 스테이션 -> 목적지
    last_st = stations_to_visit[-1]
    total_cost += distance_matrix[last_st]["목적지"]

    # 7) 배터리 페널티
    total_cost += (1.0 - battery_level) * BATTERY_PENALTY_FACTOR

    return total_cost


def create_initial_population_unique(stations, population_size):
    """
    무작위로 섞은 순열을 생성하되, '중복 없는' 유일한 순열만 사용하여
    초기 개체군(population)을 만든다.

    다만, 스테이션 개수가 작을 때 factorial(len(stations))보다 큰 population_size를
    요구하면, 절대 만들 수 없는 만큼 고유 순열을 요구하게 되므로 무한루프 발생.

    여기서 factorial(len(stations))을 구해, population_size가 그 이상이면
    population_size를 자동으로 축소하여 해결한다.
    """
    # 1) 만들 수 있는 최대 순열 개수 = factorial(스테이션 수)
    max_permutations = math.factorial(len(stations))

    # 2) population_size가 최대 순열 개수를 초과하면, 자동 조정
    if population_size > max_permutations:
        print(f"[Warning] Requested population_size={population_size} "
              f"but only {max_permutations} unique permutations are possible. "
              f"Adjusting population_size to {max_permutations}.")
        population_size = max_permutations

    unique_individuals = set()
    while len(unique_individuals) < population_size:
        candidate = stations[:]
        random.shuffle(candidate)
        unique_individuals.add(tuple(candidate))

    return [list(ind) for ind in unique_individuals]


def ga_optimize_order(
    stations_to_visit,
    occupied_info,
    battery_level,
    population_size=30,   # 초기 개체군 크기
    generations=80,       # 세대 수
    mutation_rate=0.1     # 돌연변이 확률
):
    """
    유전 알고리즘을 활용하여 스테이션 방문 순서를 최적화한다.
    스테이션 수가 많아(예: 15개 이상) 전수조사가 어렵더라도,
    GA를 통해 근사해를 찾을 수 있다.

    :param stations_to_visit: 방문해야 할 스테이션 목록 (예: ["냉동", "신선", "일반"] 등)
    :param occupied_info: {"냉동": {"robot": (False,0), "person": (False,0)}, ...}
    :param battery_level: 로봇의 배터리 잔량 (0.0 ~ 1.0)
    :param population_size: 초기 개체군 크기
    :param generations: 총 진화 세대 수
    :param mutation_rate: 돌연변이 확률
    :return: (best_route, best_cost) 형태
    """
    # 0) 예외 처리: 스테이션이 없거나 1개 이하
    if len(stations_to_visit) <= 1:
        cost = calculate_route_cost(stations_to_visit, occupied_info, battery_level)
        return list(stations_to_visit), cost

    # --------------------------------------------------
    # 1) 초기 개체군: 중복 없는 고유 순열
    #    (population_size가 factorial보다 큰 경우 자동 조정)
    # --------------------------------------------------
    population = create_initial_population_unique(stations_to_visit, population_size)

    # --------------------------------------------------
    # 2) 적합도(Fitness) 함수
    #    비용(cost)이 낮을수록 좋으므로, fitness = 1/(cost+ε)
    # --------------------------------------------------
    def fitness(route):
        c = calculate_route_cost(route, occupied_info, battery_level)
        return 1.0 / (c + 1e-6)

    # --------------------------------------------------
    # 3) 부모 선택(Selection)
    #    - 간단히 정렬 후 상위 절반을 부모로 사용(랭킹 선택)
    # --------------------------------------------------
    def select_parents(pop):
        scored = [(ind, fitness(ind)) for ind in pop]
        scored.sort(key=lambda x: x[1], reverse=True)
        cutoff = len(scored) // 2
        parents = [x[0] for x in scored[:cutoff]]
        # 베스트 1개 (route, fitness)
        best_individual, best_fitness = scored[0]
        return parents, (best_individual, best_fitness)

    # --------------------------------------------------
    # 4) 교차(Crossover)
    #    - 순열에 특화된 간단한 교차 연산 예시
    # --------------------------------------------------
    def crossover(p1, p2):
        size = len(p1)
        # start~end 구간 무작위
        start = random.randint(0, size - 2)
        end = random.randint(start+1, size - 1)

        child = [None] * size
        # (1) p1 구간 복사
        for i in range(start, end+1):
            child[i] = p1[i]

        # (2) p2의 나머지 원소를 순서대로 채움
        ptr = 0
        for elem in p2:
            if elem not in child:
                while child[ptr] is not None:
                    ptr += 1
                child[ptr] = elem
        return child

    # --------------------------------------------------
    # 5) 돌연변이(Mutation): 일정 확률로 swap
    # --------------------------------------------------
    def mutate(ind, rate=0.1):
        if random.random() < rate:
            a, b = random.sample(range(len(ind)), 2)
            ind[a], ind[b] = ind[b], ind[a]

    # --------------------------------------------------
    # 6) 세대 반복
    # --------------------------------------------------
    best_route = None
    best_cost = math.inf

    for gen in range(generations):
        # (1) 부모 집단 선택
        parents, (top_ind, top_fit) = select_parents(population)
        current_best_cost = 1.0 / (top_fit + 1e-6)  # fitness->cost 역변환

        # (2) 세대 최고 해 갱신
        if current_best_cost < best_cost:
            best_cost = current_best_cost
            best_route = top_ind[:]

        # (3) 자식 세대 생성
        new_pop = []
        while len(new_pop) < population_size:
            p1 = random.choice(parents)
            p2 = random.choice(parents)
            child = crossover(p1, p2)
            mutate(child, mutation_rate)
            new_pop.append(child)

        population = new_pop

    return list(best_route), best_cost
