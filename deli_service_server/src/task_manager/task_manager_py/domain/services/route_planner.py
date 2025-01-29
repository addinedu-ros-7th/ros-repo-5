import itertools
import random
import math

distance_matrix = {
    "냉동":   {"냉동":0, "신선":4, "일반":1, "목적지":2, "출발지":4},
    "신선":   {"냉동":4, "신선":0, "일반":2, "목적지":4, "출발지":2},
    "일반":   {"냉동":1, "신선":2, "일반":0, "목적지":3, "출발지":4},
    "목적지": {"냉동":2, "신선":4, "일반":3, "목적지":0, "출발지":1},
    "출발지": {"냉동":4, "신선":2, "일반":4, "목적지":1, "출발지":0},
}

order_weight = {
    "냉동": [5,3,1],
    "신선": [3,2,1],
    "일반": [1,2,3],
}

FIRST_VISIT_OCCUPIED_PENALTY = 10
BATTERY_PENALTY_FACTOR = 5.0

def calculate_route_cost(stations_to_visit, occupied_info, battery_level):
    start_station = "출발지"
    total_cost = 0.0

    if not stations_to_visit:
        total_cost += distance_matrix[start_station]["목적지"]
        total_cost += (1.0 - battery_level) * BATTERY_PENALTY_FACTOR
        return total_cost

    # 출발지 -> 첫 스테이션
    first_st = stations_to_visit[0]
    total_cost += distance_matrix[start_station][first_st]

    # --------------------------
    # 1) 첫 스테이션의 실제 점유
    # --------------------------
    # occupied_info[first_st] -> { "robot":(bool,int), "person":(bool,int) }
    r_occ, r_time = occupied_info[first_st]["robot"]
    p_occ, p_time = occupied_info[first_st]["person"]

    # 최종 점유 여부 = r_occ or p_occ
    # 최종 remain_time = max(r_time, p_time) (또는 sum 등 정책에 맞게)
    total_occ = (r_occ or p_occ)
    total_remain = max(r_time, p_time)

    if total_occ:
        if total_remain > 0:
            total_cost += total_remain
        else:
            total_cost += FIRST_VISIT_OCCUPIED_PENALTY

    # 스테이션 방문 가중치
    for i, st in enumerate(stations_to_visit):
        idx = min(i, 2)
        total_cost += order_weight[st][idx]

    # 스테이션들 간 이동
    for i in range(len(stations_to_visit) - 1):
        f = stations_to_visit[i]
        t = stations_to_visit[i+1]
        total_cost += distance_matrix[f][t]

    # 마지막 스테이션 -> 목적지
    last_st = stations_to_visit[-1]
    total_cost += distance_matrix[last_st]["목적지"]

    # 배터리 페널티
    total_cost += (1.0 - battery_level) * BATTERY_PENALTY_FACTOR
    return total_cost


def ga_optimize_order(stations_to_visit, occupied_info, battery_level,
                      population_size=20, generations=50, mutation_rate=0.1):
    if len(stations_to_visit) <= 1:
        cost = calculate_route_cost(stations_to_visit, occupied_info, battery_level)
        return stations_to_visit, cost

    all_perms = list(itertools.permutations(stations_to_visit))
    random.shuffle(all_perms)

    if len(all_perms) < population_size:
        population = all_perms
    else:
        population = all_perms[:population_size]

    def fitness(order):
        c = calculate_route_cost(order, occupied_info, battery_level)
        return 1.0 / (c + 1e-6)

    for _ in range(generations):
        scored_pop = [(p, fitness(p)) for p in population]
        scored_pop.sort(key=lambda x: x[1], reverse=True)

        cutoff = len(scored_pop) // 2
        parents = [sp[0] for sp in scored_pop[:cutoff]]

        new_pop = []
        while len(new_pop) < population_size:
            p1 = random.choice(parents)
            p2 = random.choice(parents)

            idx = random.randint(0, len(p1) - 1)
            child = list(p1[:idx])
            for g in p2:
                if g not in child:
                    child.append(g)

            # 변이
            if random.random() < mutation_rate:
                a, b = random.sample(range(len(child)), 2)
                child[a], child[b] = child[b], child[a]

            new_pop.append(tuple(child))

        population = new_pop

    best_cost = math.inf
    best_order = None
    for p in population:
        c = calculate_route_cost(p, occupied_info, battery_level)
        if c < best_cost:
            best_cost = c
            best_order = p

    return list(best_order), best_cost

# replan_route 도 동일하게 robot+person 점유를 합산해서 계산
