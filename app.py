from flask import Flask, render_template, request
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
    locations_str = ";".join([f"{coord['lon']},{coord['lat']}" for coord in coords_list])
    url = f"http://router.project-osrm.org/table/v1/driving/{locations_str}"
    params = {'annotations': 'distance,duration'}
    try:
        response = requests.get(url, params=params, timeout=10)
        response.raise_for_status()
        data = response.json()
        if data['code'] == 'Ok':
            return data['distances'], data['durations']
    except requests.exceptions.RequestException as e:
        print(f"Lỗi OSRM API: {e}")
    return None, None

def calculate_total_distance(path_indices, dist_matrix):
    """Tính tổng quãng đường cho một lộ trình dựa trên ma trận khoảng cách."""
    total_dist = 0
    for i in range(len(path_indices) - 1):
        total_dist += dist_matrix[path_indices[i]][path_indices[i+1]]
    return total_dist

def apply_2_opt(initial_path_indices, dist_matrix):
    """
    Áp dụng thuật toán 2-Opt để cải thiện lộ trình ban đầu.
    Nó hoạt động bằng cách loại bỏ các cạnh giao nhau.
    """
    best_path = initial_path_indices[:]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best_path) - 2):
            for j in range(i + 1, len(best_path)):
                if j == i + 1: continue
                # Lộ trình hiện tại: ...A-B...C-D...
                # Lộ trình mới:    ...A-C...B-D...
                # So sánh khoảng cách của (A-B) + (C-D) với (A-C) + (B-D)
                current_dist = dist_matrix[best_path[i-1]][best_path[i]] + dist_matrix[best_path[j-1]][best_path[j]]
                new_dist = dist_matrix[best_path[i-1]][best_path[j-1]] + dist_matrix[best_path[i]][best_path[j]]
                
                if new_dist < current_dist:
                    # Nếu lộ trình mới tốt hơn, đảo ngược đoạn ở giữa
                    best_path[i:j] = best_path[j-1:i-1:-1]
                    improved = True
        initial_path_indices = best_path
    
    final_distance = calculate_total_distance(best_path, dist_matrix)
    return best_path, final_distance


def solve_tsp_advanced(addresses):
    """
    Giải bài toán TSP bằng cách kết hợp Nearest Neighbor và 2-Opt.
    """
    # Bước 1: Chuyển đổi địa chỉ thành tọa độ
    coords_list = [get_coords_from_address(addr) for addr in addresses]
    if any(c is None for c in coords_list):
        failed_addr = addresses[coords_list.index(None)]
        raise ValueError(f"Không thể tìm thấy tọa độ cho địa chỉ: {failed_addr}")

    if len(coords_list) < 2:
        return coords_list, 0, 0, None, None

    # Bước 2: Lấy ma trận khoảng cách
    dist_matrix, duration_matrix = get_route_info(coords_list)
    if not dist_matrix:
        raise ConnectionError("Không thể lấy dữ liệu tuyến đường từ OSRM API.")

    # Bước 3: Chạy Nearest Neighbor để có giải pháp ban đầu
    num_locations = len(coords_list)
    start_node = 0
    current_node = start_node
    unvisited = list(range(1, num_locations))
    initial_path_indices = [start_node]
    
    while unvisited:
        nearest_node = min(unvisited, key=lambda node: dist_matrix[current_node][node])
        current_node = nearest_node
        initial_path_indices.append(current_node)
        unvisited.remove(nearest_node)
    initial_path_indices.append(start_node) # Quay về kho
    
    initial_distance = calculate_total_distance(initial_path_indices, dist_matrix)

    # Bước 4: Áp dụng 2-Opt để cải thiện lộ trình
    if len(coords_list) > 3: # 2-Opt chỉ có ý nghĩa với 4 điểm trở lên (bao gồm cả kho)
        improved_path_indices, improved_distance = apply_2_opt(initial_path_indices, dist_matrix)
    else:
        improved_path_indices, improved_distance = initial_path_indices, initial_distance

    # Sắp xếp lại kết quả cuối cùng
    final_path_coords = [coords_list[i] for i in improved_path_indices]
    
    # Tính tổng thời gian cho lộ trình cuối cùng
    total_duration = 0
    for i in range(len(improved_path_indices) - 1):
        from_node = improved_path_indices[i]
        to_node = improved_path_indices[i+1]
        total_duration += duration_matrix[from_node][to_node]

    return final_path_coords, initial_distance, improved_distance, total_duration

# --- Route chính ---

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

            all_addresses = [warehouse_address] + delivery_addresses
            
            final_path, initial_dist, improved_dist, total_duration = solve_tsp_advanced(all_addresses)
            
            form_data = {'kho_hang': warehouse_address, 'cac_diem_giao': delivery_addresses}

            return render_template(
                'index.html',
                lo_trinh=final_path,
                initial_dist_km=initial_dist / 1000.0,
                improved_dist_km=improved_dist / 1000.0,
                tong_thoi_gian_text=time.strftime("%H giờ %M phút", time.gmtime(total_duration)),
                form_data=form_data
            )
        except (ValueError, ConnectionError) as e:
            return render_template('index.html', error=str(e), form_data=default_data)
    
    return render_template('index.html', form_data=default_data)


if __name__ == '__main__':
    app.run(debug=True)
