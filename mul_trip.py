import json
import math
import time
from docplex.mp.model import Model
from docplex.mp.utils import DOcplexException

# =========================
# HÀM/CÔNG CỤ TIỆN ÍCH CƠ BẢN
# =========================

# tính quãng đường euclid giữa 2 điểm tọa độ
def calculate_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)

# =========================
# MÔ PHỎNG THỜI GIAN ĐI LẠI CỦA KỸ THUẬT VIÊN (TỐC ĐỘ THEO KHUNG GIỜ)
# =========================
# tính thời gian đến nơi của kỹ thuật viên
def calculate_technician_arrival_time(start_node_idx, end_node_idx, start_time_milestone_idx,
                                      all_coords, dist_matrix, V_base,
                                      time_intervals_T, congestion_factors_theta, L_prime_indices):
    """
    Tính thời gian đến (theo mốc thời gian tuyệt đối, đơn vị giây) của kỹ thuật viên
    khi di chuyển từ nút start_node_idx đến end_node_idx với tốc độ phụ thuộc khoảng thời gian.
    """
    dist_to_travel = dist_matrix[start_node_idx, end_node_idx]
    current_time = time_intervals_T[start_time_milestone_idx]
    time_elapsed = 0.0

    if dist_to_travel == 0:
        return current_time

    current_interval_idx = -1
    for l in range(len(time_intervals_T) - 1):
        if time_intervals_T[l] <= current_time < time_intervals_T[l + 1]:
            current_interval_idx = l
            break
    if current_interval_idx == -1:
        current_interval_idx = len(congestion_factors_theta) - 1
        if current_time > time_intervals_T[-1]:
            print(
                f"Warning: Start time {current_time} is beyond the last milestone {time_intervals_T[-1]}. Using last congestion factor.")

    dist_covered = 0.0

    while dist_covered < dist_to_travel:
        if current_interval_idx >= len(congestion_factors_theta):
            print(f"Warning: Reached interval index {current_interval_idx} beyond defined factors. Using last factor.")
            current_speed = congestion_factors_theta[-1] * V_base
            if current_speed <= 1e-9:
                print(f"ERROR: Calculated speed is zero or negative in fallback for interval {current_interval_idx}.")
                return float('inf')
            time_to_finish = (dist_to_travel - dist_covered) / current_speed
            time_elapsed += time_to_finish
            dist_covered = dist_to_travel
            break

        current_speed = congestion_factors_theta[current_interval_idx] * V_base
        if current_speed <= 1e-9:
            print(f"ERROR: Calculated speed is zero or negative for interval {current_interval_idx}. Cannot proceed.")
            return float('inf')

        time_to_next_milestone = float('inf')
        if current_interval_idx < len(time_intervals_T) - 1:
            time_to_next_milestone = time_intervals_T[current_interval_idx + 1] - (current_time + time_elapsed)

        dist_can_cover_in_interval = current_speed * time_to_next_milestone
        dist_remaining = dist_to_travel - dist_covered

        if dist_remaining <= dist_can_cover_in_interval + 1e-9:
            time_needed = dist_remaining / current_speed
            time_elapsed += time_needed
            dist_covered = dist_to_travel
        else:
            time_elapsed += time_to_next_milestone
            dist_covered += dist_can_cover_in_interval
            current_interval_idx += 1

    arrival_time = current_time + time_elapsed
    return arrival_time


# =========================
# NẠP DỮ LIỆU CẤU HÌNH VÀ INSTANCE
# =========================
def load_data(config_path, instance_path):
    """Loads configuration and instance data."""
    print(f"Loading config from: {config_path}")
    with open(config_path, 'r') as f:
        config = json.load(f)

    print(f"Loading instance from: {instance_path}")
    with open(instance_path, 'r') as f:
        lines = f.readlines()

    # Đọc số lượng kỹ thuật viên, drone, khách hàng từ file instance
    num_technicians = int(lines[0].split()[1])
    num_drones = int(lines[1].split()[1])
    num_customers = int(lines[3].split()[1])

    # Đọc dữ liệu từng khách hàng (tọa độ, trọng lượng hàng, ràng buộc chỉ nhân viên, thời gian phục vụ tech/drone, Wi_limit optional)
    customer_data = []
    for i in range(5, 5 + num_customers):
        parts = lines[i].split()
        Wi_limit = None
        if len(parts) >= 7:
            Wi_limit = float(parts[6])
        customer_data.append({
            'id': i - 4,
            'x': float(parts[0]),
            'y': float(parts[1]),
            'w': float(parts[2]),
            'only_staff': int(parts[3]) == 1,
            'sigma_tech': float(parts[4]),
            'sigma_drone': float(parts[5]),
            'Wi_limit': Wi_limit  # per-customer waiting cap
        })

    print(f"Loaded {num_technicians} technicians, {num_drones} drones, {num_customers} customers.")
    return config, num_technicians, num_drones, customer_data


# =========================
# CHƯƠNG TRÌNH CHÍNH
# =========================
if __name__ == "__main__":
    start_run_time = time.time()

    # --- Configuration and Parameters ---
    CONFIG_FILE = 'config.json'  # đường dẫn file cấu hình
    INSTANCE_FILE = 'test_instances/SUBTOUR_TECH_DEMO.txt'  # file instance
    BIG_M = 1e7                  # hằng số Big-M cho ràng buộc tuyến tính hóa
    EPSILON = 1e-6               # ngưỡng số học nhỏ
    CPLEX_TIME_LIMIT = 300       # giới hạn thời gian chạy (giây)

    # Nạp dữ liệu
    config, num_technicians, num_drones, customer_raw_data = load_data(CONFIG_FILE, INSTANCE_FILE)
    num_customers = len(customer_raw_data)

    # Tập chỉ số nút: 0 là depot, 1..n là khách hàng
    N_nodes_idx = list(range(num_customers + 1))
    C_customers_idx = list(range(1, num_customers + 1))
    K_techs_idx = list(range(num_technicians))
    D_drones_idx = list(range(num_drones))

    # Phân loại khách hàng: chỉ kỹ thuật viên (C_T) hay cả hai phương tiện (C_A)
    C_T_idx = [c['id'] for c in customer_raw_data if c['only_staff']]
    C_A_idx = [c['id'] for c in customer_raw_data if not c['only_staff']]
    print(f"Technician-only customers (C_T): {C_T_idx}")
    print(f"Any-vehicle customers (C_A): {C_A_idx}")

    # Tọa độ: depot (0) đặt tại (0,0); khách hàng theo file
    coords = {0: (0, 0)}
    for c in customer_raw_data:
        coords[c['id']] = (c['x'], c['y'])

    # Tham số trọng lượng giao hàng và thời gian phục vụ
    weights = {c['id']: c['w'] for c in customer_raw_data}
    sigma_tech = {c['id']: c['sigma_tech'] for c in customer_raw_data}
    sigma_drone = {c['id']: c['sigma_drone'] for c in customer_raw_data if c['id'] in C_A_idx}

    # Hạn chờ riêng từng khách; nếu thiếu thì lấy từ config['cold_chain']['Wi_max_default'] hoặc 3600s
    default_Wi = config.get('cold_chain', {}).get('Wi_max_default', 3600.0)
    Wi_limit = {c['id']: (c['Wi_limit'] if c['Wi_limit'] is not None else default_Wi)
                for c in customer_raw_data}

    # =========================
    # THÔNG SỐ KỸ THUẬT VIÊN: TỐC ĐỘ THAY ĐỔI THEO KHUNG GIỜ
    # =========================
    V_base_tech = config['technician']['V_max (m/s)']         # tốc độ cơ sở
    time_congestion_dict = config['technician']['T (hour)']   # dict khung giờ: "start-end" -> hệ số theta
    L = len(time_congestion_dict)
    L_intervals_idx = list(range(L))
    L_prime_milestones_idx = list(range(L + 1))  # 0..L (mốc thời gian)

    # Chuyển khung giờ theo giờ sang mốc thời gian giây và mảng hệ số theta
    time_intervals_T = [0.0] * (L + 1)
    congestion_factors_theta = [0.0] * L
    sorted_intervals = sorted(time_congestion_dict.keys(), key=lambda x: int(x.split('-')[0]))

    last_end_hour = 0
    for idx, interval_key in enumerate(sorted_intervals):
        start_hour, end_hour = map(int, interval_key.split('-'))
        if start_hour != last_end_hour:
            print(f"WARNING: Gap or overlap in technician time intervals detected at hour {start_hour}!")
        time_intervals_T[idx + 1] = end_hour * 3600.0
        congestion_factors_theta[idx] = time_congestion_dict[interval_key]
        last_end_hour = end_hour
    print(f"Technician Milestones T_l (sec): {time_intervals_T}")
    print(f"Technician Congestion Factors theta_l: {congestion_factors_theta}")

    # =========================
    # THÔNG SỐ DRONE: ĐỘ CAO, VẬN TỐC, NĂNG LƯỢNG, TẢI TRỌNG
    # =========================
    h_drone = config['drone']['cruiseAlt [m]']
    vt_drone = config['drone']['takeoffSpeed [m/s]']
    vc_drone = config['drone']['cruiseSpeed [m/s]']
    vl_drone = config['drone']['landingSpeed [m/s]']
    Wmax_drone = config['drone']['capacity [kg]']
    E_drone = config['drone']['batteryPower [Joule]']
    beta_drone = config['drone']['beta(w/kg)']
    gamma_drone = config['drone']['gama(w)']

    # Số chuyến tối đa mỗi drone (đặt bằng số khách hàng có thể phục vụ bởi drone, tối đa)
    Rmax = len(C_A_idx) if C_A_idx else 1
    R_trips_idx = list(range(1, Rmax + 1))
    print(f"Max drone trips per drone (Rmax): {Rmax}")

    # >>> Technician multi-trip
    RTmax = len(C_customers_idx) if C_customers_idx else 1
    RT_trips_idx = list(range(1, RTmax + 1))
    print(f"Max technician trips per technician (RTmax): {RTmax}")

    # =========================
    # TIỀN XỬ LÝ: MA TRẬN KHOẢNG CÁCH, THỜI GIAN BAY DRONE, AT_tech
    # =========================
    print("Starting pre-computations...")
    precomp_start_time = time.time()

    # Ma trận khoảng cách Euclid giữa tất cả cặp nút
    d_matrix = {(i, j): calculate_distance(coords[i], coords[j])
                for i in N_nodes_idx for j in N_nodes_idx}

    # Thời gian cất/hạ cánh theo vận tốc thẳng đứng
    t_takeoff = h_drone / vt_drone if vt_drone > 0 else float('inf')
    t_landing = h_drone / vl_drone if vl_drone > 0 else float('inf')

    # Thời gian bay drone T_drone[i,j] = cất cánh + bay ngang + hạ cánh
    T_drone = {}
    for i in N_nodes_idx:
        for j in N_nodes_idx:
            if i == j:
                T_drone[i, j] = 0
                continue
            if i == 0 and j in C_A_idx:
                t_cruise = d_matrix[0, j] / vc_drone if vc_drone > 0 else float('inf')
                T_drone[0, j] = t_takeoff + t_cruise + t_landing
            elif i in C_A_idx and j not in C_T_idx and i != 0 and j != 0:
                t_cruise = d_matrix[i, j] / vc_drone if vc_drone > 0 else float('inf')
                T_drone[i, j] = t_takeoff + t_cruise + t_landing
            elif i in C_A_idx and j == 0:
                t_cruise = d_matrix[i, 0] / vc_drone if vc_drone > 0 else float('inf')
                T_drone[i, 0] = t_takeoff + t_cruise + t_landing

    # Bổ sung các cặp (i,j) bắt buộc phải có trong T_drone (đảm bảo đầy đủ)
    required_drone_pairs = set()
    for i in C_A_idx:
        required_drone_pairs.add((i, 0))
    for j in C_A_idx:
        required_drone_pairs.add((0, j))
    for i in C_A_idx:
        for j in N_nodes_idx:
            if j not in C_T_idx and i != j:
                required_drone_pairs.add((i, j))

    missing_drone_times = False
    for pair in required_drone_pairs:
        if pair not in T_drone:
            i, j = pair
            if i in C_A_idx and j == 0 and (i, 0) not in T_drone:
                t_cruise = d_matrix[i, 0] / vc_drone if vc_drone > 0 else float('inf')
                T_drone[i, 0] = t_takeoff + t_cruise + t_landing
                print(f"Calculated potentially missing T_drone({i},{j}) = {T_drone[i, j]}")
            elif i in C_A_idx and j in C_A_idx and i != j and (i, j) not in T_drone:
                t_cruise = d_matrix[i, j] / vc_drone if vc_drone > 0 else float('inf')
                T_drone[i, j] = t_takeoff + t_cruise + t_landing
                print(f"Calculated potentially missing T_drone({i},{j}) = {T_drone[i, j]}")
            else:
                print(f"ERROR: Required drone travel time T_drone{pair} is missing and not defined by formulas!")
                missing_drone_times = True
    if missing_drone_times:
        raise ValueError("Missing required drone travel times.")

    # Bảng thời điểm đến của kỹ thuật viên AT_tech[i,j,l] với mọi mốc xuất phát l
    AT_tech = {}
    print("Calculating Technician Arrival Times (AT)... this may take time.")
    for i in N_nodes_idx:
        for j in N_nodes_idx:
            if i == j: continue
            for l in L_prime_milestones_idx:
                AT_tech[i, j, l] = calculate_technician_arrival_time(
                    i, j, l, coords, d_matrix, V_base_tech,
                    time_intervals_T, congestion_factors_theta, L_prime_milestones_idx
                )
                if AT_tech[i, j, l] == float('inf'):
                    print(f"WARNING: Technician travel from {i} to {j} starting at T_{l} is infeasible (zero speed).")

    print(f"Pre-computation finished in {time.time() - precomp_start_time:.2f} seconds.")

    # =========================
    # MÔ HÌNH TỐI ƯU DOCPLEX
    # =========================
    mdl = Model(name='MSSVTDE')

    # ----- Biến quyết định -----
    print("Defining variables...")
    # Technician đa-trip: X[i,j,k,r]
    X_vars = mdl.binary_var_dict(
        ((i, j, k, r) for i in N_nodes_idx for j in N_nodes_idx if i != j
         for k in K_techs_idx for r in RT_trips_idx),
        name="X"
    )

    # Drone
    N_minus_CT_idx = [idx for idx in N_nodes_idx if idx not in C_T_idx]
    Y_vars = mdl.binary_var_dict(((i, j, d, r) for i in N_minus_CT_idx for j in N_minus_CT_idx if i != j
                                  for d in D_drones_idx for r in R_trips_idx), name="Y")

    # Thời điểm đến/đi/return của technician có thêm r
    S_tech_vars = mdl.continuous_var_dict(
        ((i, k, r) for i in N_nodes_idx for k in K_techs_idx for r in RT_trips_idx), lb=0, name="S_tech")
    S_dep_tech_vars = mdl.continuous_var_dict(
        ((i, k, r) for i in N_nodes_idx for k in K_techs_idx for r in RT_trips_idx), lb=0, name="S_dep_tech")
    A_ret_tech_vars = mdl.continuous_var_dict(
        ((k, r) for k in K_techs_idx for r in RT_trips_idx), lb=0, name="A_ret_tech")

    # Lambda nội suy cho technician có thêm r
    Lambda_tech_vars = mdl.continuous_var_dict(
        ((i, j, k, r, l) for i in N_nodes_idx for j in N_nodes_idx if i != j
         for k in K_techs_idx for r in RT_trips_idx for l in L_prime_milestones_idx),
        lb=0, name="Lambda"
    )

    # Drone variables
    S_drone_vars = mdl.continuous_var_dict(
        ((i, d, r) for i in N_minus_CT_idx for d in D_drones_idx for r in R_trips_idx), lb=0, name="S_drone")
    A_ret_drone_vars = mdl.continuous_var_dict(((d, r) for d in D_drones_idx for r in R_trips_idx), lb=0,
                                               name="A_ret_drone")

    L_drone_vars = mdl.continuous_var_dict(
        ((i, d, r) for i in N_minus_CT_idx for d in D_drones_idx for r in R_trips_idx), lb=0, name="L_drone")
    E_drone_vars = mdl.continuous_var_dict(
        ((i, d, r) for i in N_minus_CT_idx for d in D_drones_idx for r in R_trips_idx), lb=0, name="E_drone")
    E_ret_drone_vars = mdl.continuous_var_dict(((d, r) for d in D_drones_idx for r in R_trips_idx), lb=0,
                                               name="E_ret_drone")

    # Thời gian chờ tại khách hàng và biến makespan
    W_vars = mdl.continuous_var_dict(C_customers_idx, lb=0, name="W")
    Cmax_var = mdl.continuous_var(lb=0, name="Cmax")
    print("Variables defined.")

    # ----- Hàm mục tiêu (đơn mục tiêu) -----
    print("Defining objective function...")
    mdl.minimize(Cmax_var)
    print("Objective defined.")

    # ----- Ràng buộc -----
    print("Defining constraints...")

    # Mỗi khách hàng C_A được phục vụ đúng 1 lần (bởi kỹ thuật viên hoặc drone)
    for i in C_A_idx:
        tech_arrival_sum = mdl.sum(X_vars[j, i, k, r]
                                   for j in N_nodes_idx if i != j
                                   for k in K_techs_idx for r in RT_trips_idx
                                   if (j, i, k, r) in X_vars)
        drone_arrival_sum = mdl.sum(Y_vars[j, i, d, r]
                                    for j in N_minus_CT_idx if i != j
                                    for d in D_drones_idx for r in R_trips_idx
                                    if (j, i, d, r) in Y_vars)
        mdl.add_constraint(tech_arrival_sum + drone_arrival_sum == 1,
                           ctname=f"assign_CA_{i}")

    # Khách hàng C_T chỉ được kỹ thuật viên phục vụ đúng 1 lần
    for i in C_T_idx:
        tech_arrival_sum = mdl.sum(X_vars[j, i, k, r]
                                   for j in N_nodes_idx if i != j
                                   for k in K_techs_idx for r in RT_trips_idx
                                   if (j, i, k, r) in X_vars)
        mdl.add_constraint(tech_arrival_sum == 1, ctname=f"assign_CT_{i}")

    # Luồng technician theo từng trip và xâu chuỗi các trip
    for k in K_techs_idx:
        # mốc rời depot của trip 1 = 0
        mdl.add_constraint(S_dep_tech_vars[0, k, 1] == 0, ctname=f"tech_depart_depot_trip1_{k}")
        # (tùy chọn) đặt S_tech tại depot trip 1 = 0 cho rõ ràng
        mdl.add_constraint(S_tech_vars[0, k, 1] == 0, ctname=f"tech_start_depot_trip1_{k}")

        for r in RT_trips_idx:
            # Mỗi trip rời depot tối đa 1 lần
            mdl.add_constraint(mdl.sum(X_vars[0, j, k, r] for j in C_customers_idx if (0, j, k, r) in X_vars) <= 1,
                               ctname=f"tech_depot_leave_{k}_{r}")

            # Số lần về depot = số lần rời depot (trip-wise)
            mdl.add_constraint(
                mdl.sum(X_vars[i, 0, k, r] for i in C_customers_idx if (i, 0, k, r) in X_vars) ==
                mdl.sum(X_vars[0, j, k, r] for j in C_customers_idx if (0, j, k, r) in X_vars),
                ctname=f"tech_depot_flow_{k}_{r}"
            )

            # Bảo toàn luồng tại từng khách hàng cho mỗi trip
            for h in C_customers_idx:
                mdl.add_constraint(
                    mdl.sum(X_vars[j, h, k, r] for j in N_nodes_idx if j != h and (j, h, k, r) in X_vars) ==
                    mdl.sum(X_vars[h, j, k, r] for j in N_nodes_idx if j != h and (h, j, k, r) in X_vars),
                    ctname=f"tech_customer_flow_{h}_{k}_{r}"
                )

        # Xâu chuỗi: trip r+1 bắt đầu sau khi trip r về depot
        for r in RT_trips_idx:
            if r < RTmax:
                mdl.add_constraint(S_dep_tech_vars[0, k, r + 1] >= A_ret_tech_vars[k, r],
                                   ctname=f"tech_trip_sequence_{k}_{r}")

    # Luồng vào/ra depot và bảo toàn luồng cho drone theo từng chuyến r
    for d in D_drones_idx:
        # Start time trip 1 = 0
        if (0, d, 1) in S_drone_vars:
            mdl.add_constraint(S_drone_vars[0, d, 1] == 0, ctname=f"drone_trip1_start_{d}")

        for r in R_trips_idx:
            # Mỗi chuyến rời depot tối đa 1 lần
            mdl.add_constraint(mdl.sum(Y_vars[0, j, d, r] for j in C_A_idx if (0, j, d, r) in Y_vars) <= 1,
                               ctname=f"drone_depot_leave_{d}_{r}")

            # Số lần về depot = số lần rời depot (mỗi chuyến)
            mdl.add_constraint(mdl.sum(Y_vars[i, 0, d, r] for i in C_A_idx if (i, 0, d, r) in Y_vars) ==
                               mdl.sum(Y_vars[0, j, d, r] for j in C_A_idx if (0, j, d, r) in Y_vars),
                               ctname=f"drone_depot_flow_{d}_{r}")

            # Bảo toàn luồng tại từng khách hàng cho drone
            for h in C_A_idx:
                mdl.add_constraint(
                    mdl.sum(Y_vars[j, h, d, r] for j in N_minus_CT_idx if j != h and (j, h, d, r) in Y_vars) ==
                    mdl.sum(Y_vars[h, j, d, r] for j in N_minus_CT_idx if j != h and (h, j, d, r) in Y_vars),
                    ctname=f"drone_customer_flow_{h}_{d}_{r}")

            # Thứ tự chuyến: chuyến r+1 chỉ có thể bắt đầu sau khi kết thúc chuyến r
            if r < Rmax:
                next_trip_start_var = S_drone_vars.get((0, d, r + 1))
                current_trip_return_var = A_ret_drone_vars.get((d, r))
                if next_trip_start_var is not None and current_trip_return_var is not None:
                    mdl.add_constraint(next_trip_start_var >= current_trip_return_var,
                                       ctname=f"drone_trip_sequence_{d}_{r}")

    # Ràng buộc thời gian phục vụ và nội suy Lambda (kỹ thuật viên) theo (k,r)
    for k in K_techs_idx:
        for r in RT_trips_idx:
            for i in C_customers_idx:
                tech_enters_i_sum = mdl.sum(X_vars[j, i, k, r] for j in N_nodes_idx if i != j and (j, i, k, r) in X_vars)
                mdl.add_constraint(
                    S_dep_tech_vars[i, k, r] >= S_tech_vars[i, k, r] + sigma_tech[i] - BIG_M * (1 - tech_enters_i_sum),
                    ctname=f"tech_depart_after_service_{i}_{k}_{r}")
                mdl.add_constraint(S_dep_tech_vars[i, k, r] >= S_tech_vars[i, k, r],
                                   ctname=f"tech_depart_after_arrival_{i}_{k}_{r}")

            # Nội suy thời gian đến dựa trên Lambda và AT_tech (SOS2 theo mốc l)
            for i in N_nodes_idx:
                for j in C_customers_idx:
                    if i == j: continue
                    if (i, j, k, r) in X_vars:
                        mdl.add_constraint(mdl.sum(Lambda_tech_vars[i, j, k, r, l] for l in L_prime_milestones_idx if
                                                   (i, j, k, r, l) in Lambda_tech_vars) == X_vars[i, j, k, r],
                                           ctname=f"tech_lambda_sum_{i}_{j}_{k}_{r}")

                        mdl.add_constraint(S_tech_vars[j, k, r] >= mdl.sum(
                            Lambda_tech_vars[i, j, k, r, l] * AT_tech[i, j, l] for l in L_prime_milestones_idx if
                            (i, j, k, r, l) in Lambda_tech_vars)
                                           - BIG_M * (1 - X_vars[i, j, k, r]),
                                           ctname=f"tech_arrival_lb_{i}_{j}_{k}_{r}")
                        mdl.add_constraint(S_tech_vars[j, k, r] <= mdl.sum(
                            Lambda_tech_vars[i, j, k, r, l] * AT_tech[i, j, l] for l in L_prime_milestones_idx if
                            (i, j, k, r, l) in Lambda_tech_vars)
                                           + BIG_M * (1 - X_vars[i, j, k, r]),
                                           ctname=f"tech_arrival_ub_{i}_{j}_{k}_{r}")

                        mdl.add_constraint(S_dep_tech_vars[i, k, r] >= mdl.sum(
                            Lambda_tech_vars[i, j, k, r, l] * time_intervals_T[l] for l in L_prime_milestones_idx if
                            (i, j, k, r, l) in Lambda_tech_vars)
                                           - BIG_M * (1 - X_vars[i, j, k, r]),
                                           ctname=f"tech_depart_lambda_lb_{i}_{j}_{k}_{r}")

                        mdl.add_constraint(S_dep_tech_vars[i, k, r] <= mdl.sum(
                            Lambda_tech_vars[i, j, k, r, l] * time_intervals_T[l] for l in L_prime_milestones_idx if
                            (i, j, k, r, l) in Lambda_tech_vars)
                                           + BIG_M * (1 - X_vars[i, j, k, r]),
                                           ctname=f"tech_depart_lambda_ub_{i}_{j}_{k}_{r}")

                        # Thêm SOS2 để Lambda chọn tối đa 2 mốc liên tiếp
                        try:
                            lambda_vars_for_sos = [Lambda_tech_vars[i, j, k, r, l] for l in L_prime_milestones_idx if
                                                   (i, j, k, r, l) in Lambda_tech_vars]
                            if len(lambda_vars_for_sos) >= 2:
                                mdl.add_sos2(lambda_vars_for_sos, name=f"tech_sos2_{i}_{j}_{k}_{r}")
                        except DOcplexException as e:
                            print(f"Warning: Could not add SOS2 for tech arc ({i},{j},{k},{r}). Error: {e}")
                        except KeyError as e:
                            print(f"Warning: KeyError adding SOS2 for tech arc ({i},{j},{k},{r}). Error: {e}")

            # Xử lý cung quay về depot (i->0) cho kỹ thuật viên
            for i in C_customers_idx:
                if (i, 0, k, r) in X_vars:
                    mdl.add_constraint(mdl.sum(
                        Lambda_tech_vars[i, 0, k, r, l] for l in L_prime_milestones_idx if (i, 0, k, r, l) in Lambda_tech_vars) ==
                                       X_vars[i, 0, k, r],
                                       ctname=f"tech_lambda_sum_ret_{i}_{k}_{r}")

                    mdl.add_constraint(A_ret_tech_vars[k, r] >= mdl.sum(
                        Lambda_tech_vars[i, 0, k, r, l] * AT_tech[i, 0, l] for l in L_prime_milestones_idx if
                        (i, 0, k, r, l) in Lambda_tech_vars)
                                       - BIG_M * (1 - X_vars[i, 0, k, r]),
                                       ctname=f"tech_return_lb_{i}_{k}_{r}")
                    mdl.add_constraint(A_ret_tech_vars[k, r] <= mdl.sum(
                        Lambda_tech_vars[i, 0, k, r, l] * AT_tech[i, 0, l] for l in L_prime_milestones_idx if
                        (i, 0, k, r, l) in Lambda_tech_vars)
                                       + BIG_M * (1 - X_vars[i, 0, k, r]),
                                       ctname=f"tech_return_ub_{i}_{k}_{r}")

                    mdl.add_constraint(S_dep_tech_vars[i, k, r] >= mdl.sum(
                        Lambda_tech_vars[i, 0, k, r, l] * time_intervals_T[l] for l in L_prime_milestones_idx if
                        (i, 0, k, r, l) in Lambda_tech_vars)
                                       - BIG_M * (1 - X_vars[i, 0, k, r]),
                                       ctname=f"tech_depart_lambda_ret_lb_{i}_{k}_{r}")

                    mdl.add_constraint(S_dep_tech_vars[i, k, r] <= mdl.sum(
                        Lambda_tech_vars[i, 0, k, r, l] * time_intervals_T[l] for l in L_prime_milestones_idx if
                        (i, 0, k, r, l) in Lambda_tech_vars)
                                       + BIG_M * (1 - X_vars[i, 0, k, r]),
                                       ctname=f"tech_depart_lambda_ret_ub_{i}_{k}_{r}")

                    try:
                        lambda_vars_for_sos_ret = [Lambda_tech_vars[i, 0, k, r, l] for l in L_prime_milestones_idx if
                                                   (i, 0, k, r, l) in Lambda_tech_vars]
                        if len(lambda_vars_for_sos_ret) >= 2:
                            mdl.add_sos2(lambda_vars_for_sos_ret, name=f"tech_sos2_ret_{i}_{k}_{r}")
                    except DOcplexException as e:
                        print(f"Warning: Could not add SOS2 for tech return arc ({i},0,{k},{r}). Error: {e}")
                    except KeyError as e:
                        print(f"Warning: KeyError adding SOS2 for tech return arc ({i},0,{k},{r}). Error: {e}")

    # Ràng buộc thời gian di chuyển của drone theo từng cung/chuyến
    for d in D_drones_idx:
        for r in R_trips_idx:

            # Depot -> khách hàng
            for j in C_A_idx:
                if (0, j, d, r) in Y_vars and (0, j) in T_drone and T_drone[0, j] != float('inf') and (
                0, d, r) in S_drone_vars and (j, d, r) in S_drone_vars:
                    mdl.add_constraint(S_drone_vars[j, d, r] >= S_drone_vars[0, d, r] + T_drone[0, j] - BIG_M * (
                                1 - Y_vars[0, j, d, r]),
                                       ctname=f"drone_time_depot_cust_{j}_{d}_{r}")

            # Khách hàng -> khách hàng
            for i in C_A_idx:
                for j in N_minus_CT_idx:
                    if i == j: continue
                    if j == 0: continue
                    if (i, j, d, r) in Y_vars and (i, j) in T_drone and T_drone[i, j] != float(
                            'inf') and i in sigma_drone and (i, d, r) in S_drone_vars and (j, d, r) in S_drone_vars:
                        mdl.add_constraint(
                            S_drone_vars[j, d, r] >= S_drone_vars[i, d, r] + sigma_drone[i] + T_drone[i, j] - BIG_M * (
                                        1 - Y_vars[i, j, d, r]),
                            ctname=f"drone_time_cust_cust_{i}_{j}_{d}_{r}")

            # Khách hàng -> depot (kết thúc chuyến)
            for i in C_A_idx:
                if (i, 0, d, r) in Y_vars and (i, 0) in T_drone and T_drone[i, 0] != float(
                        'inf') and i in sigma_drone and (i, d, r) in S_drone_vars and (d, r) in A_ret_drone_vars:
                    mdl.add_constraint(
                        A_ret_drone_vars[d, r] >= S_drone_vars[i, d, r] + sigma_drone[i] + T_drone[i, 0] - BIG_M * (
                                    1 - Y_vars[i, 0, d, r]),
                        ctname=f"drone_time_cust_depot_{i}_{d}_{r}")

    # Ràng buộc tải trọng (L_drone) theo cung
    for d in D_drones_idx:
        for r in R_trips_idx:
            if (0, d, r) in L_drone_vars:
                mdl.add_constraint(L_drone_vars[0, d, r] == 0, ctname=f"drone_payload_start_{d}_{r}")

            # Depot -> khách hàng
            for j in C_A_idx:
                if (0, j, d, r) in Y_vars and j in weights and (j, d, r) in L_drone_vars:
                    mdl.add_constraint(L_drone_vars[j, d, r] >= weights[j] - BIG_M * (1 - Y_vars[0, j, d, r]),
                                       ctname=f"drone_payload_depot_cust_{j}_{d}_{r}")

            # Khách -> khách: cập nhật tải tích lũy
            for i in C_A_idx:
                for j in C_A_idx:
                    if i == j: continue
                    if (i, j, d, r) in Y_vars and j in weights and (i, d, r) in L_drone_vars and (
                    j, d, r) in L_drone_vars:
                        mdl.add_constraint(L_drone_vars[j, d, r] >= L_drone_vars[i, d, r] + weights[j] - BIG_M * (
                                    1 - Y_vars[i, j, d, r]),
                                           ctname=f"drone_payload_cust_cust_{i}_{j}_{d}_{r}")

            # Giới hạn tải tối đa theo cung rời i
            for i in N_minus_CT_idx:
                if (i, d, r) in L_drone_vars:
                    sum_leaving_y = mdl.sum(
                        Y_vars[i, j, d, r] for j in N_minus_CT_idx if i != j and (i, j, d, r) in Y_vars)
                    mdl.add_constraint(L_drone_vars[i, d, r] <= Wmax_drone * sum_leaving_y,
                                       ctname=f"drone_payload_max_{i}_{d}_{r}")

    # Ràng buộc năng lượng (E_drone_vars) theo cung
    for d in D_drones_idx:
        for r in R_trips_idx:
            if (0, d, r) in E_drone_vars:
                mdl.add_constraint(E_drone_vars[0, d, r] == 0, ctname=f"drone_energy_start_{d}_{r}")

            # Depot -> khách hàng: tiêu hao năng lượng theo T_drone[0,j] với tải 0 (khi rời depot)
            for j in C_A_idx:
                if (0, j, d, r) in Y_vars and (0, j) in T_drone and T_drone[0, j] != float('inf') and (
                0, d, r) in E_drone_vars and (j, d, r) in E_drone_vars:
                    power_0j = beta_drone * 0 + gamma_drone
                    energy_consumed_0j = power_0j * T_drone[0, j]
                    mdl.add_constraint(E_drone_vars[j, d, r] >= E_drone_vars[0, d, r] + energy_consumed_0j - BIG_M * (
                                1 - Y_vars[0, j, d, r]),
                                       ctname=f"drone_energy_depot_cust_{j}_{d}_{r}")

            # Khách -> khách: tiêu hao theo tải hiện tại L_drone[i,d,r]
            for i in C_A_idx:
                for j in C_A_idx:
                    if i == j: continue
                    if (i, j, d, r) in Y_vars and (i, j) in T_drone and T_drone[i, j] != float('inf') and (
                    i, d, r) in L_drone_vars and (i, d, r) in E_drone_vars and (
                    j, d, r) in E_drone_vars:
                        power_ij = beta_drone * L_drone_vars[i, d, r] + gamma_drone
                        energy_consumed_ij = power_ij * T_drone[i, j]
                        mdl.add_constraint(
                            E_drone_vars[j, d, r] >= E_drone_vars[i, d, r] + energy_consumed_ij - BIG_M * (
                                        1 - Y_vars[i, j, d, r]),
                            ctname=f"drone_energy_cust_cust_{i}_{j}_{d}_{r}")

            # Khách -> depot: cập nhật năng lượng tiêu hao và ràng buộc năng lượng quay về <= E_drone
            for i in C_A_idx:
                if (i, 0, d, r) in Y_vars and (i, 0) in T_drone and T_drone[i, 0] != float('inf') and (
                i, d, r) in L_drone_vars and (i, d, r) in E_drone_vars and (d, r) in E_ret_drone_vars:
                    power = beta_drone * L_drone_vars[i, d, r] + gamma_drone
                    energy_consumed = power * T_drone[i, 0]
                    mdl.add_constraint(E_ret_drone_vars[d, r] >= E_drone_vars[i, d, r] + energy_consumed - BIG_M * (
                                1 - Y_vars[i, 0, d, r]),
                                       ctname=f"drone_energy_cust_depot_{i}_{d}_{r}")

            if (d, r) in E_ret_drone_vars:
                mdl.add_constraint(E_ret_drone_vars[d, r] <= E_drone, ctname=f"drone_energy_max_{d}_{r}")

    # Makespan: Cmax ≥ thời điểm quay về muộn nhất của kỹ thuật viên (mọi trip) và drone
    for k in K_techs_idx:
        for r in RT_trips_idx:
            if (k, r) in A_ret_tech_vars:
                mdl.add_constraint(Cmax_var >= A_ret_tech_vars[k, r], ctname=f"makespan_tech_{k}_{r}")
    for d in D_drones_idx:
        for r in R_trips_idx:
            if (d, r) in A_ret_drone_vars:
                mdl.add_constraint(Cmax_var >= A_ret_drone_vars[d, r], ctname=f"makespan_drone_{d}_{r}")

    # Thời gian chờ tại khách hàng i: định nghĩa cho cả technician (mọi trip) và drone + trần W_i ≤ Wi_limit[i]
    for i in C_customers_idx:
        # Trường hợp kỹ thuật viên phục vụ i (mọi k,r)
        for k in K_techs_idx:
            for r in RT_trips_idx:
                tech_enters_i_sum = mdl.sum(X_vars[j, i, k, r] for j in N_nodes_idx if i != j and (j, i, k, r) in X_vars)
                mdl.add_constraint(
                    W_vars[i] >= A_ret_tech_vars[k, r] - (S_tech_vars[i, k, r] + sigma_tech[i]) - BIG_M * (1 - tech_enters_i_sum),
                    ctname=f"wait_time_tech_{i}_{k}_{r}"
                )

        # Trường hợp drone phục vụ i
        if i in C_A_idx and i in sigma_drone:
            for d in D_drones_idx:
                for r in R_trips_idx:
                    sum_Y_jidr_dr = mdl.sum(Y_vars[j, i, d, r] for j in N_minus_CT_idx if i != j and (j, i, d, r) in Y_vars)
                    mdl.add_constraint(
                        W_vars[i] >= A_ret_drone_vars[d, r] - (S_drone_vars[i, d, r] + sigma_drone[i]) - BIG_M * (1 - sum_Y_jidr_dr),
                        ctname=f"wait_time_drone_{i}_{d}_{r}"
                    )

        # Trần thời gian chờ riêng từng khách
        mdl.add_constraint(W_vars[i] <= Wi_limit[i], ctname=f"wait_cap_{i}")

    # Thống kê quy mô mô hình
    print(f"Constraints defined. Total constraints: {mdl.number_of_constraints}")
    print(f"Total variables: {mdl.number_of_variables}")
    print(f"  Binary: {mdl.number_of_binary_variables}")
    print(f"  Continuous: {mdl.number_of_continuous_variables}")
    if hasattr(mdl, "number_of_sos2") and mdl.number_of_sos2 > 0:
        print(f"  SOS2 sets: {mdl.number_of_sos2}")

    # --- Solve ---
    print("Solving the model...")
    mdl.set_time_limit(CPLEX_TIME_LIMIT)
    try:
        solution = mdl.solve(log_output=True)

        # --- Results ---
        if solution:
            print("\n-------------------- SOLUTION --------------------")
            print(f"Solve status: {mdl.get_solve_status()}")
            print(f"Objective value (Cmax): {Cmax_var.solution_value:.4f}")
            total_wait_time = sum(W_vars[i].solution_value for i in C_customers_idx if i in W_vars)
            print(f"Total Waiting Time (Sum Wi): {total_wait_time:.4f}")

            # In tuyến của kỹ thuật viên theo từng trip
            print("\nTechnician Routes:")
            for k in K_techs_idx:
                for r in RT_trips_idx:
                    if sum(X_vars[0, j, k, r].solution_value for j in C_customers_idx if (0, j, k, r) in X_vars) > 0.5:
                        route_kr = []
                        visited_in_route = set()
                        current_node = 0
                        for _ in range(num_customers + 2):
                            found_next = False
                            for j in N_nodes_idx:
                                if current_node != j and (current_node, j, k, r) in X_vars and X_vars[
                                    current_node, j, k, r].solution_value > 0.5:
                                    if j not in visited_in_route or j == 0:
                                        route_kr.append((current_node, j))
                                        visited_in_route.add(current_node)
                                        current_node = j
                                        found_next = True
                                        if current_node == 0:
                                            break
                                        else:
                                            visited_in_route.add(current_node)
                                        break
                            if not found_next or current_node == 0:
                                break
                        if route_kr:
                            print(f"  Tech {k}, Trip {r}: {route_kr}")
                            if (k, r) in A_ret_tech_vars:
                                print(f"    Return time: {A_ret_tech_vars[k, r].solution_value:.2f}")

            # In tuyến của drone theo từng chuyến
            print("\nDrone Routes:")
            for d in D_drones_idx:
                for r in R_trips_idx:
                    if sum(Y_vars[0, j, d, r].solution_value for j in C_A_idx if (0, j, d, r) in Y_vars) > 0.5:
                        route_dr = []
                        visited_in_trip = set()
                        current_node = 0
                        for _ in range(len(C_A_idx) + 2):
                            found_next = False
                            for j in N_minus_CT_idx:
                                if current_node != j and (current_node, j, d, r) in Y_vars and Y_vars[
                                    current_node, j, d, r].solution_value > 0.5:
                                    if j not in visited_in_trip or j == 0:
                                        route_dr.append((current_node, j))
                                        visited_in_trip.add(current_node)
                                        current_node = j
                                        found_next = True
                                        if current_node == 0:
                                            break
                                        else:
                                            visited_in_trip.add(current_node)
                                        break
                            if not found_next or current_node == 0:
                                break
                        if route_dr:
                            print(f"  Drone {d}, Trip {r}: {route_dr}")
                            if (d, r) in A_ret_drone_vars:
                                print(f"    Return time: {A_ret_drone_vars[d, r].solution_value:.2f}")
                            if (d, r) in E_ret_drone_vars:
                                print(f"    Return energy: {E_ret_drone_vars[d, r].solution_value:.2f} / {E_drone}")

            # In thời gian chờ từng khách hàng
            print("\nWaiting Times:")
            for i in C_customers_idx:
                if i in W_vars:
                    print(f"  Customer {i}: {W_vars[i].solution_value:.2f} (cap {Wi_limit[i]:.2f})")

        else:
            print("No solution found.")
            print(f"Solve status: {mdl.get_solve_status()}")

    except DOcplexException as e:
        print(f"CPLEX Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()

    # Tổng thời gian chạy
    print(f"\nTotal runtime: {time.time() - start_run_time:.2f} seconds")

    # =========================
    # (TÙY CHỌN) KIỂM TRA SUBTOUR CHO TECHNICIAN k, TRIP r
    # =========================
    from collections import deque

    k = 0  # technician index you want to inspect
    r = 1  # trip index to inspect

    selected_arcs_kr = [(i, j) for i in N_nodes_idx for j in N_nodes_idx
                        if i != j and (i, j, k, r) in X_vars and X_vars[i, j, k, r].solution_value > 0.5]

    print("\nAll tech arcs with X[i,j,{k},{r}]=1:".format(k=k, r=r))
    for (i, j) in selected_arcs_kr:
        print("  ({:d} -> {:d})".format(i, j))

    reachable = set([0])
    q = deque([0])
    while q:
        u = q.popleft()
        for (i, j) in selected_arcs_kr:
            if i == u and j not in reachable:
                reachable.add(j)
                q.append(j)

    subtour_candidate_arcs = [(i, j) for (i, j) in selected_arcs_kr if i not in reachable]
    print("\nPotential technician subtour arcs (not reachable from depot):")
    if not subtour_candidate_arcs:
        print("  (none)")
    else:
        for (i, j) in subtour_candidate_arcs:
            print("  ({:d} -> {:d})".format(i, j))

    unreachable_nodes = [v for v in C_customers_idx if v not in reachable]
    print("\nCustomers not reachable from depot for tech {k}, trip {r}: {nodes}".format(k=k, r=r, nodes=unreachable_nodes))
