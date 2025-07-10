from flask import Flask, render_template, request, jsonify
import requests
import time
import math
import random

app = Flask(__name__)

# --- Lõi thuật toán (Không thay đổi nhiều, chỉ cấu trúc lại) ---

def get_coords_from_address(address):
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
    total_dist = 0
    for i in range(len(path_indices) - 1):
        total_dist += dist_matrix[path_indices[i]][path_indices[i+1]]
    return total_dist

def run_2_opt_solver(coords_list, dist_matrix, avoid_segment_indices=None):
    """Chạy chuỗi thuật toán Nearest Neighbor + 2-Opt, có thể tránh đường."""
    if avoid_segment_indices:
        from_idx, to_idx = avoid_segment_indices
        dist_matrix[from_idx][to_idx] = float('inf')

    # Chạy Nearest Neighbor để có giải pháp ban đầu
    num_locations = len(coords_list)
    start_node = 0
    current_node = start_node
    unvisited = list(range(1, num_locations))
    initial_path_indices = [start_node]
    while unvisited:
        reachable_nodes = {node: dist_matrix[current_node][node] for node in unvisited if dist_matrix[current_node][node] != float('inf')}
        if not reachable_nodes: raise ValueError("Không thể tìm thấy lộ trình hợp lệ khi tránh đoạn đường đã chọn.")
        nearest_node = min(reachable_nodes, key=reachable_nodes.get)
        current_node = nearest_node
        initial_path_indices.append(current_node)
        unvisited.remove(nearest_node)
    initial_path_indices.append(start_node)
    
    # Áp dụng 2-Opt
    if len(coords_list) > 3:
        best_path, best_dist = apply_2_opt(initial_path_indices, dist_matrix)
    else:
        best_path, best_dist = initial_path_indices, calculate_total_distance(initial_path_indices, dist_matrix)
    
    if best_dist == float('inf'):
         raise ValueError("Không thể tìm thấy lộ trình hợp lệ khi tránh đoạn đường đã chọn.")
         
    return best_path, best_dist

def apply_2_opt(initial_path_indices, dist_matrix):
    best_path = initial_path_indices[:]
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
        initial_path_indices = best_path
    return best_path, calculate_total_distance(best_path, dist_matrix)

def run_sa_solver(coords_list, dist_matrix):
    """Chạy thuật toán Simulated Annealing."""
    current_solution = list(range(1, len(coords_list)))
    random.shuffle(current_solution)
    current_solution = [0] + current_solution + [0]
    current_cost = calculate_total_distance(current_solution, dist_matrix)
    best_solution, best_cost = current_solution[:], current_cost
    temp, stopping_temp, alpha = 10000, 1, 0.995
    while temp > stopping_temp:
        i, j = random.sample(range(1, len(coords_list)), 2)
        neighbor_solution = current_solution[:]
        neighbor_solution[i], neighbor_solution[j] = neighbor_solution[j], neighbor_solution[i]
        neighbor_cost = calculate_total_distance(neighbor_solution, dist_matrix)
        cost_diff = neighbor_cost - current_cost
        if cost_diff < 0 or random.uniform(0, 1) < math.exp(-cost_diff / temp):
            current_solution, current_cost = neighbor_solution[:], neighbor_cost
            if current_cost < best_cost:
                best_solution, best_cost = current_solution[:], current_cost
        temp *= alpha
    return best_solution, best_cost

# --- Routes ---

@app.route('/', methods=['GET', 'POST'])
def home():
    default_data = {
        'kho_hang': 'Bưu điện Trung tâm Sài Gòn',
        'cac_diem_giao': ['Sân bay Tân Sơn Nhất', 'Chợ Bến Thành', 'Dinh Độc Lập', 'Bảo tàng Chứng tích Chiến tranh', 'Chùa Bửu Long']
    }

    if request.method == 'POST':
        try:
            warehouse_address = request.form['warehouse_address']
            delivery_addresses = [addr for addr in request.form.getlist('delivery_address[]') if addr.strip()]

            if not warehouse_address or not delivery_addresses:
                return render_template('index.html', error="Vui lòng nhập đủ địa chỉ.", form_data=default_data)

            all_addresses_text = [warehouse_address] + delivery_addresses
            all_addresses_data = [get_coords_from_address(addr) for addr in all_addresses_text]
            if any(c is None for c in all_addresses_data):
                failed_addr = all_addresses_text[all_addresses_data.index(None)]
                raise ValueError(f"Không thể tìm thấy tọa độ cho địa chỉ: {failed_addr}")

            dist_matrix, _ = get_route_info(all_addresses_data)
            if not dist_matrix:
                raise ConnectionError("Không thể lấy dữ liệu từ OSRM API.")

            results = []
            # Chạy 2-Opt
            start_time_2opt = time.time()
            path_indices_2opt, dist_2opt = run_2_opt_solver(all_addresses_data, dist_matrix)
            results.append({
                "name": "NN + 2-Opt",
                "path": [all_addresses_data[i] for i in path_indices_2opt],
                "distance_km": dist_2opt / 1000.0,
                "exec_time_ms": (time.time() - start_time_2opt) * 1000
            })
            # Chạy SA
            start_time_sa = time.time()
            path_indices_sa, dist_sa = run_sa_solver(all_addresses_data, dist_matrix)
            results.append({
                "name": "Simulated Annealing",
                "path": [all_addresses_data[i] for i in path_indices_sa],
                "distance_km": dist_sa / 1000.0,
                "exec_time_ms": (time.time() - start_time_sa) * 1000
            })
            
            form_data = {'kho_hang': warehouse_address, 'cac_diem_giao': delivery_addresses}
            return render_template('index.html', results=results, form_data=form_data, all_addresses_data=all_addresses_data)
        except (ValueError, ConnectionError) as e:
            return render_template('index.html', error=str(e), form_data=default_data)
    
    return render_template('index.html', form_data=default_data)

@app.route('/reroute', methods=['POST'])
def reroute():
    """API endpoint để tính toán lại tuyến đường khi có yêu cầu tránh một đoạn."""
    try:
        data = request.get_json()
        all_addresses_data = data.get('all_addresses_data')
        avoid_segment = data.get('avoid_segment')

        if not all_addresses_data or not avoid_segment:
            return jsonify({'error': 'Dữ liệu không hợp lệ'}), 400

        dist_matrix, duration_matrix = get_route_info(all_addresses_data)
        if not dist_matrix:
            raise ConnectionError("Không thể lấy dữ liệu từ OSRM API.")
            
        from_idx = next((i for i, item in enumerate(all_addresses_data) if item["display_name"] == avoid_segment['from']), -1)
        to_idx = next((i for i, item in enumerate(all_addresses_data) if item["display_name"] == avoid_segment['to']), -1)
        
        if from_idx == -1 or to_idx == -1: return jsonify({'error': 'Không tìm thấy địa chỉ cần tránh'}), 400

        # Khi reroute, luôn dùng thuật toán 2-Opt nhanh và hiệu quả để có phản hồi tức thì
        path_indices, distance = run_2_opt_solver(all_addresses_data, dist_matrix, avoid_segment_indices=(from_idx, to_idx))
        
        total_duration = 0
        for i in range(len(path_indices) - 1):
            total_duration += duration_matrix[path_indices[i]][path_indices[i+1]]

        response_data = {
            'name': 'Tuyến đường thay thế (2-Opt)',
            'path': [all_addresses_data[i] for i in path_indices],
            'distance_km': distance / 1000.0,
            'exec_time_ms': 0, # Không cần đo thời gian cho reroute
            'total_duration_text': time.strftime("%H giờ %M phút", time.gmtime(total_duration))
        }
        return jsonify(response_data)
        
    except (ValueError, ConnectionError) as e:
        return jsonify({'error': str(e)}), 500
    except Exception as e:
        print(f"Lỗi không xác định khi reroute: {e}")
        return jsonify({'error': 'Lỗi phía server khi tính toán lại tuyến đường'}), 500

if __name__ == '__main__':
    app.run(debug=True)
