from flask import Flask, render_template, request
import requests
import time

# Khởi tạo ứng dụng Flask
app = Flask(__name__)

# --- Lõi thuật toán (Đã viết lại để dùng OpenStreetMap APIs) ---

def get_coords_from_address(address):
    """
    Sử dụng Nominatim API để chuyển đổi địa chỉ thành tọa độ (lat, lon).
    """
    url = "https://nominatim.openstreetmap.org/search"
    params = {'q': address, 'format': 'json', 'limit': 1}
    # Nominatim yêu cầu một User-Agent tùy chỉnh
    headers = {'User-Agent': 'TSP-Solver-App/1.0'}
    
    try:
        response = requests.get(url, params=params, headers=headers, timeout=10)
        response.raise_for_status()  # Ném lỗi nếu request không thành công
        data = response.json()
        if data:
            # Trả về cả địa chỉ đầy đủ mà API tìm thấy và tọa độ
            return {
                "display_name": data[0]['display_name'],
                "lat": float(data[0]['lat']),
                "lon": float(data[0]['lon'])
            }
    except requests.exceptions.RequestException as e:
        print(f"Lỗi khi gọi Nominatim API cho địa chỉ '{address}': {e}")
    return None

def get_route_info(coords_list):
    """
    Sử dụng OSRM API để lấy ma trận khoảng cách và thời gian.
    """
    # Định dạng chuỗi tọa độ cho OSRM: lon1,lat1;lon2,lat2;...
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
        print(f"Lỗi khi gọi OSRM API: {e}")
    return None, None

def solve_tsp_with_osm(addresses):
    """
    Giải bài toán TSP bằng thuật toán Nearest Neighbor sử dụng OSRM.
    """
    # Bước 1: Chuyển đổi tất cả địa chỉ thành tọa độ
    coords_list = []
    for addr in addresses:
        coord_data = get_coords_from_address(addr)
        if coord_data:
            coords_list.append(coord_data)
        else:
            # Nếu một địa chỉ không tìm thấy, ném lỗi để báo cho người dùng
            raise ValueError(f"Không thể tìm thấy tọa độ cho địa chỉ: {addr}")

    if len(coords_list) < 2:
        return coords_list, 0, 0

    # Bước 2: Lấy ma trận khoảng cách và thời gian từ OSRM
    dist_matrix, duration_matrix = get_route_info(coords_list)
    if not dist_matrix or not duration_matrix:
        raise ConnectionError("Không thể lấy dữ liệu tuyến đường từ OSRM API.")

    # Bước 3: Chạy thuật toán Nearest Neighbor
    num_locations = len(coords_list)
    start_node = 0
    current_node = start_node
    unvisited = list(range(1, num_locations))
    
    path_indices = [start_node]
    total_distance = 0
    total_duration = 0

    while unvisited:
        nearest_node = min(unvisited, key=lambda node: dist_matrix[current_node][node])
        
        total_distance += dist_matrix[current_node][nearest_node]
        total_duration += duration_matrix[current_node][nearest_node]
        
        current_node = nearest_node
        path_indices.append(current_node)
        unvisited.remove(nearest_node)

    last_node = path_indices[-1]
    total_distance += dist_matrix[last_node][start_node]
    total_duration += duration_matrix[last_node][start_node]
    
    optimal_path_coords = [coords_list[i] for i in path_indices]
    optimal_path_coords.append(coords_list[start_node])

    return optimal_path_coords, total_distance, total_duration

# --- Route chính của ứng dụng web ---

@app.route('/', methods=['GET', 'POST'])
def home():
    default_data = {
        'kho_hang': '138 Hai Bà Trưng, Quận 1, TP.HCM',
        'cac_diem_giao': [
            'Sân bay Tân Sơn Nhất',
            'Chợ Bến Thành',
            'Dinh Độc Lập'
        ]
    }

    if request.method == 'POST':
        try:
            warehouse_address = request.form['warehouse_address']
            delivery_addresses = [addr for addr in request.form.getlist('delivery_address[]') if addr.strip()]

            if not warehouse_address or not delivery_addresses:
                return render_template('index.html', error="Vui lòng nhập đủ địa chỉ.", form_data=default_data)

            all_addresses = [warehouse_address] + delivery_addresses
            
            optimal_path, total_distance_m, total_duration_s = solve_tsp_with_osm(all_addresses)
            
            form_data = {'kho_hang': warehouse_address, 'cac_diem_giao': delivery_addresses}

            return render_template(
                'index.html',
                lo_trinh=optimal_path,
                tong_quang_duong_km=total_distance_m / 1000.0,
                tong_thoi_gian_text=time.strftime("%H giờ %M phút", time.gmtime(total_duration_s)),
                form_data=form_data
            )
        except (ValueError, ConnectionError) as e:
            return render_template('index.html', error=str(e), form_data=default_data)
    
    return render_template('index.html', form_data=default_data)


if __name__ == '__main__':
    app.run(debug=True)
