from flask import Flask, render_template, request, jsonify
import requests
import time

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
    """
    Sử dụng OSRM API để lấy ma trận khoảng cách và thời gian.
    **Đã thêm cơ chế thử lại (retry) để tăng độ ổn định.**
    """
    locations_str = ";".join([f"{coord['lon']},{coord['lat']}" for coord in coords_list])
    url = f"http://router.project-osrm.org/table/v1/driving/{locations_str}"
    params = {'annotations': 'distance,duration'}
    
    # Thử lại tối đa 3 lần nếu có lỗi
    for attempt in range(3):
        try:
            response = requests.get(url, params=params, timeout=15) # Tăng timeout lên 15s
            response.raise_for_status() # Ném lỗi cho các mã HTTP 4xx/5xx
            data = response.json()
            if data['code'] == 'Ok':
                return data['distances'], data['durations']
        except requests.exceptions.RequestException as e:
            print(f"Lỗi OSRM API (lần thử {attempt + 1}): {e}")
            if attempt < 2: # Nếu chưa phải lần thử cuối, chờ 2 giây rồi thử lại
                print("Đang thử lại sau 2 giây...")
                time.sleep(2)
    
    # Nếu cả 3 lần đều thất bại
    return None, None


def calculate_total_distance(path_indices, dist_matrix):
    total_dist = 0
    for i in range(len(path_indices) - 1):
        total_dist += dist_matrix[path_indices[i]][path_indices[i+1]]
    return total_dist

def run_nearest_neighbor(coords_list, dist_matrix):
    """Hàm riêng để chạy thuật toán Nearest Neighbor."""
    num_locations = len(coords_list)
    start_node = 0
    current_node = start_node
    unvisited = list(range(1, num_locations))
    path_indices = [start_node]
    
    while unvisited:
        reachable_nodes = {node: dist_matrix[current_node][node] for node in unvisited if dist_matrix[current_node][node] != float('inf')}
        if not reachable_nodes:
            raise ValueError("Đồ thị không liên thông, không thể tìm thấy đường đi.")
        
        nearest_node = min(reachable_nodes, key=reachable_nodes.get)
        current_node = nearest_node
        path_indices.append(current_node)
        unvisited.remove(nearest_node)
        
    path_indices.append(start_node)
    total_distance = calculate_total_distance(path_indices, dist_matrix)
    return path_indices, total_distance

def apply_2_opt(initial_path_indices, dist_matrix):
    """Áp dụng thuật toán 2-Opt để cải thiện lộ trình."""
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
    final_distance = calculate_total_distance(best_path, dist_matrix)
    return best_path, final_distance

def solve_tsp_advanced(all_addresses_data, avoid_segment_indices=None):
    """
    Giải bài toán TSP, có tùy chọn tránh một đoạn đường.
    """
    coords_list = all_addresses_data
    if len(coords_list) < 2:
        return [], 0, 0, 0

    dist_matrix, duration_matrix = get_route_info(coords_list)
    if not dist_matrix:
        raise ConnectionError("Không thể lấy dữ liệu tuyến đường từ OSRM API sau nhiều lần thử.")

    # Chạy Nearest Neighbor trên ma trận GỐC để có giải pháp ban đầu
    initial_path_indices, initial_distance = run_nearest_neighbor(coords_list, dist_matrix)
    
    if avoid_segment_indices:
        from_idx, to_idx = avoid_segment_indices
        dist_matrix[from_idx][to_idx] = float('inf')

    # Áp dụng 2-Opt trên lộ trình ban đầu, nhưng với ma trận đã được sửa đổi.
    if len(coords_list) > 3:
        improved_path_indices, improved_distance = apply_2_opt(initial_path_indices, dist_matrix)
    else:
        improved_path_indices, improved_distance = initial_path_indices, initial_distance

    if improved_distance == float('inf'):
         raise ValueError("Không thể tìm thấy lộ trình hợp lệ khi tránh đoạn đường đã chọn.")

    final_path_coords = [coords_list[i] for i in improved_path_indices]
    
    total_duration = 0
    for i in range(len(improved_path_indices) - 1):
        from_node, to_node = improved_path_indices[i], improved_path_indices[i+1]
        total_duration += duration_matrix[from_node][to_node]

    return final_path_coords, initial_distance, improved_distance, total_duration

# --- Routes ---

@app.route('/', methods=['GET', 'POST'])
def home():
    default_data = {
        'kho_hang': 'Bưu điện Trung tâm Sài Gòn',
        'cac_diem_giao': [
            'Sân bay Tân Sơn Nhất', 'Chợ Bến Thành', 'Dinh Độc Lập', 
            'Bảo tàng Chứng tích Chiến tranh', 'Chùa Bửu Long'
        ]
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

            final_path, initial_dist, improved_dist, total_duration = solve_tsp_advanced(all_addresses_data)
            
            form_data = {'kho_hang': warehouse_address, 'cac_diem_giao': delivery_addresses}

            return render_template(
                'index.html',
                lo_trinh=final_path,
                initial_dist_km=initial_dist / 1000.0,
                improved_dist_km=improved_dist / 1000.0,
                tong_thoi_gian_text=time.strftime("%H giờ %M phút", time.gmtime(total_duration)),
                form_data=form_data,
                all_addresses_data=all_addresses_data
            )
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

        from_idx = next((i for i, item in enumerate(all_addresses_data) if item["display_name"] == avoid_segment['from']), -1)
        to_idx = next((i for i, item in enumerate(all_addresses_data) if item["display_name"] == avoid_segment['to']), -1)
        
        if from_idx == -1 or to_idx == -1:
            return jsonify({'error': 'Không tìm thấy địa chỉ cần tránh'}), 400

        final_path, _, improved_dist, total_duration = solve_tsp_advanced(all_addresses_data, avoid_segment_indices=(from_idx, to_idx))

        response_data = {
            'lo_trinh': final_path,
            'improved_dist_km': improved_dist / 1000.0,
            'tong_thoi_gian_text': time.strftime("%H giờ %M phút", time.gmtime(total_duration)),
        }
        return jsonify(response_data)
        
    except (ValueError, ConnectionError) as e:
        return jsonify({'error': str(e)}), 500
    except Exception as e:
        print(f"Lỗi không xác định khi reroute: {e}")
        return jsonify({'error': 'Lỗi phía server khi tính toán lại tuyến đường'}), 500


if __name__ == '__main__':
    app.run(debug=True)
