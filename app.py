from flask import Flask, render_template, request, jsonify
import requests
import time
import math
import random

app = Flask(__name__)

# --- Lõi thuật toán ---

def get_coords_from_address(address):
    """Sử dụng Nominatim API để chuyển đổi địa chỉ thành tọa độ."""
    url = "https://nominatim.openstreetmap.org/search"
    params = {'q': address, 'format': 'json', 'limit': 1}
    headers = {'User-Agent': 'TSP-Solver-App/1.0'}
    try:
        response = requests.get(url, params=params, headers=headers, timeout=10)
        response.raise_for_status()
        data = response.json()
        if data:
            return {"display_name": data[0]['display_name'], "lat": float(data[0]['lat']), "lon": float(data[0]['lon'])}
    except requests.exceptions.RequestException as e:
        print(f"Lỗi Nominatim API cho '{address}': {e}")
    return None

def get_route_info(coords_list):
    """Sử dụng OSRM API để lấy ma trận khoảng cách và thời gian."""
    locations_str = ";".join([f"{coord['lon']},{coord['lat']}" for coord in coords_list])
    url = f"http://router.project-osrm.org/table/v1/driving/{locations_str}"
    params = {'annotations': 'distance,duration'}
    for attempt in range(3):
        try:
            response = requests.get(url, params=params, timeout=15)
            response.raise_for_status()
            data = response.json()
            if data['code'] == 'Ok':
                return data['distances'], data['durations']
        except requests.exceptions.RequestException as e:
            print(f"Lỗi OSRM API (lần thử {attempt + 1}): {e}")
            if attempt < 2: time.sleep(2)
    return None, None

def calculate_total_distance(path_indices, dist_matrix):
    """Tính tổng quãng đường cho một lộ trình."""
    total_dist = 0
    for i in range(len(path_indices) - 1):
        total_dist += dist_matrix[path_indices[i]][path_indices[i+1]]
    return total_dist

def run_nearest_neighbor(dist_matrix):
    """Chạy thuật toán Nearest Neighbor."""
    num_locations = len(dist_matrix)
    start_node, current_node = 0, 0
    unvisited = list(range(1, num_locations))
    path_indices = [start_node]
    while unvisited:
        reachable_nodes = {node: dist_matrix[current_node][node] for node in unvisited if dist_matrix[current_node][node] != float('inf')}
        if not reachable_nodes: raise ValueError("Đồ thị không liên thông, không thể tìm thấy đường đi.")
        nearest_node = min(reachable_nodes, key=reachable_nodes.get)
        current_node = nearest_node
        path_indices.append(current_node)
        unvisited.remove(nearest_node)
    path_indices.append(start_node)
    return path_indices

def apply_2_opt(path_indices, dist_matrix):
    """Áp dụng thuật toán 2-Opt để cải thiện lộ trình."""
    best_path = path_indices[:]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best_path) - 2):
            for j in range(i + 1, len(best_path)):
                if j == i + 1: continue
                current_dist = dist_matrix[best_path[i-1]][best_path[i]] + dist_matrix[best_path[j-1]][best_path[j]]
                new_dist = dist_matrix[best_path[i-1]][best_path[j-1]] + dist_matrix[best_path[i]][best_path[j]]
                if new_dist < current_dist:
                    best_path[i:j] = best_path[j-1:i-1:-1]
                    improved = True
    return best_path

def run_sa_solver(dist_matrix):
    """Chạy thuật toán Simulated Annealing."""
    num_locations = len(dist_matrix)
    # SỬA LỖI: Xử lý trường hợp có ít hơn 2 điểm giao hàng
    if num_locations < 3:
        return run_nearest_neighbor(dist_matrix)

    current_solution = list(range(1, num_locations))
    random.shuffle(current_solution)
    current_solution = [0] + current_solution + [0]
    current_cost = calculate_total_distance(current_solution, dist_matrix)
    best_solution, best_cost = current_solution[:], current_cost
    temp, stopping_temp, alpha = 10000, 1, 0.995
    while temp > stopping_temp:
        i, j = random.sample(range(1, num_locations), 2)
        neighbor_solution = current_solution[:]
        neighbor_solution[i], neighbor_solution[j] = neighbor_solution[j], neighbor_solution[i]
        neighbor_cost = calculate_total_distance(neighbor_solution, dist_matrix)
        cost_diff = neighbor_cost - current_cost
        if cost_diff < 0 or random.uniform(0, 1) < math.exp(-cost_diff / temp):
            current_solution, current_cost = neighbor_solution[:], neighbor_cost
            if current_cost < best_cost:
                best_solution, best_cost = current_solution[:], current_cost
        temp *= alpha
    return best_solution

def run_3_opt_solver(dist_matrix):
    """Chạy thuật toán 3-Opt."""
    initial_path = run_nearest_neighbor(dist_matrix)
    if len(initial_path) < 6: # 3-Opt cần ít nhất 6 điểm để có ý nghĩa
        return initial_path

    best_path = initial_path
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best_path) - 4):
            for j in range(i + 2, len(best_path) - 2):
                for k in range(j + 2, len(best_path)):
                    if k == len(best_path) -1: continue
                    A, B, C, D, E, F = best_path[i-1], best_path[i], best_path[j-1], best_path[j], best_path[k-1], best_path[k]
                    d0 = dist_matrix[A][B] + dist_matrix[C][D] + dist_matrix[E][F]
                    d1 = dist_matrix[A][D] + dist_matrix[E][B] + dist_matrix[C][F]
                    if d1 < d0:
                        new_path = best_path[:i] + best_path[j:k] + list(reversed(best_path[i:j])) + best_path[k:]
                        best_path, improved = new_path, True
                        break
                if improved: break
            if improved: break
    return best_path

def time_str_to_seconds(time_str):
    """Chuyển đổi 'HH:MM' thành giây."""
    parts = list(map(int, time_str.split(':')))
    return parts[0] * 3600 + parts[1] * 60

def calculate_tsptw_cost(path_indices, duration_matrix, time_windows, start_time_sec):
    """Tính chi phí (tổng thời gian) cho lộ trình có ràng buộc thời gian."""
    current_time, schedule = start_time_sec, []
    for i in range(len(path_indices) - 1):
        from_node, to_node = path_indices[i], path_indices[i+1]
        arrival_time = current_time + duration_matrix[from_node][to_node]
        earliest, latest = (0, float('inf')) if to_node == 0 else (time_windows[to_node-1]['earliest'], time_windows[to_node-1]['latest'])
        if arrival_time > latest: return float('inf'), []
        wait_time = max(0, earliest - arrival_time)
        departure_time = arrival_time + wait_time
        schedule.append({"arrival": time.strftime('%H:%M:%S', time.gmtime(arrival_time)), "wait": time.strftime('%H:%M:%S', time.gmtime(wait_time)), "departure": time.strftime('%H:%M:%S', time.gmtime(departure_time))})
        current_time = departure_time
    return current_time - start_time_sec, schedule

def run_sa_solver_for_tsptw(dist_matrix, duration_matrix, time_windows, start_time_sec):
    """Giải TSPTW bằng Simulated Annealing."""
    num_locations = len(dist_matrix)
    if num_locations < 3:
        path = [0, 1, 0]
        cost, schedule = calculate_tsptw_cost(path, duration_matrix, time_windows, start_time_sec)
        return path, calculate_total_distance(path, dist_matrix), cost, schedule

    current_solution = [0] + random.sample(range(1, num_locations), num_locations - 1) + [0]
    current_cost, _ = calculate_tsptw_cost(current_solution, duration_matrix, time_windows, start_time_sec)
    best_solution, best_cost, best_schedule = current_solution[:], current_cost, []
    temp, stopping_temp, alpha = 10000, 1, 0.995
    while temp > stopping_temp:
        i, j = random.sample(range(1, num_locations), 2)
        neighbor_solution = current_solution[:]
        neighbor_solution[i], neighbor_solution[j] = neighbor_solution[j], neighbor_solution[i]
        neighbor_cost, schedule = calculate_tsptw_cost(neighbor_solution, duration_matrix, time_windows, start_time_sec)
        if neighbor_cost < float('inf'):
            cost_diff = neighbor_cost - current_cost
            if cost_diff < 0 or random.uniform(0, 1) < math.exp(-cost_diff / temp):
                current_solution, current_cost = neighbor_solution[:], neighbor_cost
                if current_cost < best_cost:
                    best_solution, best_cost, best_schedule = current_solution[:], current_cost, schedule
        temp *= alpha
    if best_cost == float('inf'):
        raise ValueError("Không tìm thấy lộ trình nào hợp lệ với các ràng buộc thời gian đã cho.")
    final_distance = calculate_total_distance(best_solution, dist_matrix)
    return best_solution, final_distance, best_cost, best_schedule

# --- Routes ---

@app.route('/', methods=['GET', 'POST'])
def home():
    default_data = {
        'kho_hang': 'Bưu điện Trung tâm Sài Gòn', 'start_time': '08:00',
        'cac_diem_giao': [
            {'address': 'Sân bay Tân Sơn Nhất', 'earliest': '09:00', 'latest': '11:00'},
            {'address': 'Chợ Bến Thành', 'earliest': '10:00', 'latest': '12:00'},
            {'address': 'Dinh Độc Lập', 'earliest': '13:00', 'latest': '15:00'}
        ], 'mode': 'distance'
    }
    if request.method == 'POST':
        try:
            mode = request.form.get('mode', 'distance')
            warehouse_address = request.form['warehouse_address']
            all_addresses_text, delivery_points_input, time_windows = [warehouse_address], [], []
            point_indices = sorted(list(set([key.split('_')[-1] for key in request.form if key.startswith('point_address_')])))
            for index in point_indices:
                address = request.form.get(f'point_address_{index}')
                if address:
                    all_addresses_text.append(address)
                    point_data = {'address': address}
                    if mode == 'schedule':
                        earliest, latest = request.form.get(f'point_earliest_{index}', '00:00'), request.form.get(f'point_latest_{index}', '23:59')
                        point_data.update({'earliest': earliest, 'latest': latest})
                        time_windows.append({'earliest': time_str_to_seconds(earliest), 'latest': time_str_to_seconds(latest)})
                    delivery_points_input.append(point_data)
            if not delivery_points_input: return render_template('index.html', error="Vui lòng nhập ít nhất một điểm giao hàng.", form_data=default_data)

            all_addresses_data = [get_coords_from_address(addr) for addr in all_addresses_text]
            if any(c is None for c in all_addresses_data): raise ValueError(f"Không thể tìm tọa độ cho địa chỉ: {all_addresses_text[all_addresses_data.index(None)]}")
            dist_matrix, duration_matrix = get_route_info(all_addresses_data)
            if not dist_matrix: raise ConnectionError("Không thể lấy dữ liệu từ OSRM API.")
            
            form_data = {'kho_hang': warehouse_address, 'cac_diem_giao': delivery_points_input, 'mode': mode}
            
            if mode == 'schedule':
                start_time_str = request.form['start_time']
                form_data['start_time'] = start_time_str
                path_indices, distance, duration, schedule = run_sa_solver_for_tsptw(dist_matrix, duration_matrix, time_windows, time_str_to_seconds(start_time_str))
                final_path = [all_addresses_data[i] for i in path_indices]
                for i, step in enumerate(schedule):
                    if i < len(final_path) - 1: final_path[i+1]['schedule'] = step
                return render_template('index.html', result_tsptw=final_path, distance_km=distance/1000.0, duration_sec=duration, form_data=form_data, all_addresses_data=all_addresses_data)
            else:
                results = []
                initial_path_nn = run_nearest_neighbor(dist_matrix)
                start_time_2opt = time.time()
                path_indices_2opt = apply_2_opt(initial_path_nn, dist_matrix)
                results.append({"name": "NN + 2-Opt", "path": [all_addresses_data[i] for i in path_indices_2opt], "distance_km": calculate_total_distance(path_indices_2opt, dist_matrix) / 1000.0, "exec_time_ms": (time.time() - start_time_2opt) * 1000})
                
                start_time_3opt = time.time()
                path_indices_3opt = run_3_opt_solver(dist_matrix)
                results.append({"name": "NN + 3-Opt", "path": [all_addresses_data[i] for i in path_indices_3opt], "distance_km": calculate_total_distance(path_indices_3opt, dist_matrix) / 1000.0, "exec_time_ms": (time.time() - start_time_3opt) * 1000})
                
                start_time_sa = time.time()
                path_indices_sa = run_sa_solver(dist_matrix)
                results.append({"name": "Simulated Annealing", "path": [all_addresses_data[i] for i in path_indices_sa], "distance_km": calculate_total_distance(path_indices_sa, dist_matrix) / 1000.0, "exec_time_ms": (time.time() - start_time_sa) * 1000})
                
                results.sort(key=lambda x: x['distance_km'])
                return render_template('index.html', results=results, form_data=form_data, all_addresses_data=all_addresses_data)
        except (ValueError, ConnectionError) as e:
            return render_template('index.html', error=str(e), form_data=default_data)
    return render_template('index.html', form_data=default_data)

@app.route('/reroute', methods=['POST'])
def reroute():
    try:
        data = request.get_json()
        all_addresses_data, avoid_segment = data.get('all_addresses_data'), data.get('avoid_segment')
        if not all_addresses_data or not avoid_segment: return jsonify({'error': 'Dữ liệu không hợp lệ'}), 400
        dist_matrix, duration_matrix = get_route_info(all_addresses_data)
        if not dist_matrix: raise ConnectionError("Không thể lấy dữ liệu từ OSRM API.")
        from_idx = next((i for i, item in enumerate(all_addresses_data) if item["display_name"] == avoid_segment['from']), -1)
        to_idx = next((i for i, item in enumerate(all_addresses_data) if item["display_name"] == avoid_segment['to']), -1)
        if from_idx == -1 or to_idx == -1: return jsonify({'error': 'Không tìm thấy địa chỉ cần tránh.'}), 400
        
        modified_dist_matrix = [row[:] for row in dist_matrix]
        modified_dist_matrix[from_idx][to_idx] = float('inf')
        
        initial_path_indices = run_nearest_neighbor(modified_dist_matrix)
        final_path_indices = apply_2_opt(initial_path_indices, modified_dist_matrix)
        final_distance = calculate_total_distance(final_path_indices, modified_dist_matrix)
        if final_distance == float('inf'): raise ValueError("Không thể tìm thấy lộ trình hợp lệ khi tránh đoạn đường đã chọn.")
        
        total_duration = sum(duration_matrix[final_path_indices[i]][final_path_indices[i+1]] for i in range(len(final_path_indices) - 1))
        response_data = {'name': 'Tuyến đường thay thế', 'path': [all_addresses_data[i] for i in final_path_indices], 'distance_km': final_distance / 1000.0, 'total_duration_text': time.strftime("%H giờ %M phút", time.gmtime(total_duration))}
        return jsonify(response_data)
    except (ValueError, ConnectionError) as e:
        return jsonify({'error': str(e)}), 500
    except Exception as e:
        print(f"Lỗi không xác định khi reroute: {e}")
        return jsonify({'error': 'Lỗi phía server khi tính toán lại tuyến đường'}), 500

if __name__ == '__main__':
    app.run(debug=True)
