from flask import Flask, render_template, request, jsonify
import requests
import time
import math
import random

app = Flask(__name__)

# --- Lõi thuật toán ---

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

def run_nearest_neighbor(coords_list, dist_matrix):
    num_locations = len(coords_list)
    start_node = 0
    current_node = start_node
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
    return path_indices, calculate_total_distance(path_indices, dist_matrix)

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

def run_3_opt_solver(coords_list, dist_matrix):
    initial_path, _ = run_nearest_neighbor(coords_list, dist_matrix)
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
                        best_path = new_path
                        improved = True
                        break
                if improved: break
            if improved: break
    return best_path, calculate_total_distance(best_path, dist_matrix)

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
            initial_path_2opt, _ = run_nearest_neighbor(all_addresses_data, dist_matrix)
            start_time_2opt = time.time()
            path_indices_2opt, dist_2opt = apply_2_opt(initial_path_2opt, dist_matrix)
            results.append({
                "name": "NN + 2-Opt",
                "path": [all_addresses_data[i] for i in path_indices_2opt],
                "distance_km": dist_2opt / 1000.0,
                "exec_time_ms": (time.time() - start_time_2opt) * 1000
            })
            start_time_3opt = time.time()
            path_indices_3opt, dist_3opt = run_3_opt_solver(all_addresses_data, dist_matrix)
            results.append({
                "name": "NN + 3-Opt",
                "path": [all_addresses_data[i] for i in path_indices_3opt],
                "distance_km": dist_3opt / 1000.0,
                "exec_time_ms": (time.time() - start_time_3opt) * 1000
            })
            start_time_sa = time.time()
            path_indices_sa, dist_sa = run_sa_solver(all_addresses_data, dist_matrix)
            results.append({
                "name": "Simulated Annealing",
                "path": [all_addresses_data[i] for i in path_indices_sa],
                "distance_km": dist_sa / 1000.0,
                "exec_time_ms": (time.time() - start_time_sa) * 1000
            })
            results.sort(key=lambda x: x['distance_km'])
            form_data = {'kho_hang': warehouse_address, 'cac_diem_giao': delivery_addresses}
            return render_template('index.html', results=results, form_data=form_data, all_addresses_data=all_addresses_data)
        except (ValueError, ConnectionError) as e:
            return render_template('index.html', error=str(e), form_data=default_data)
    return render_template('index.html', form_data=default_data)

@app.route('/reroute', methods=['POST'])
def reroute():
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
        
        if from_idx == -1 or to_idx == -1:
            return jsonify({'error': 'Không tìm thấy địa chỉ cần tránh trong dữ liệu.'}), 400

        # **SỬA LỖI LOGIC:**
        # 1. Sửa đổi ma trận để mô phỏng tắc đường TRƯỚC TIÊN.
        modified_dist_matrix = [row[:] for row in dist_matrix]
        modified_dist_matrix[from_idx][to_idx] = float('inf')

        # 2. Chạy lại toàn bộ thuật toán (NN + 2-Opt) trên ma trận đã bị sửa đổi.
        #    Cách này mạnh mẽ hơn là cố gắng "vá" lại lộ trình cũ.
        initial_path_indices, _ = run_nearest_neighbor(all_addresses_data, modified_dist_matrix)
        final_path_indices, final_distance = apply_2_opt(initial_path_indices, modified_dist_matrix)
        
        if final_distance == float('inf'):
            raise ValueError("Không thể tìm thấy lộ trình hợp lệ khi tránh đoạn đường đã chọn. Hãy thử chặn một đoạn đường khác.")

        total_duration = 0
        for i in range(len(final_path_indices) - 1):
            total_duration += duration_matrix[final_path_indices[i]][final_path_indices[i+1]]

        response_data = {
            'name': 'Tuyến đường thay thế',
            'path': [all_addresses_data[i] for i in final_path_indices],
            'distance_km': final_distance / 1000.0,
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
