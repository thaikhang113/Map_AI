# Map_AI
🚚 Hệ thống Tối ưu Tuyến đường Giao hàng Thông minh

1. Tổng quan Đề tài
Dự án này là một ứng dụng web được xây dựng để giải quyết Bài toán Người bán hàng (Travelling Salesman Problem - TSP) và phiên bản nâng cao của nó là Bài toán Người bán hàng có Ràng buộc Thời gian (TSP with Time Windows - TSPTW).

Mục tiêu chính là xây dựng một công cụ thông minh, cho phép người dùng nhập vào một danh sách các địa điểm và tìm ra lộ trình tối ưu nhất để đi qua tất cả các điểm, dựa trên các tiêu chí khác nhau như quãng đường ngắn nhất hoặc tổng thời gian (bao gồm cả di chuyển và chờ đợi) ít nhất.

Ứng dụng được xây dựng hoàn toàn bằng các công nghệ mã nguồn mở, không phụ thuộc vào các API trả phí, và tích hợp nhiều thuật toán từ cơ bản đến nâng cao để so sánh và tìm ra giải pháp hiệu quả.

2. Các Tính năng Chính
Hệ thống được trang bị một loạt các tính năng mạnh mẽ, bao gồm:

✅ Giao diện Người dùng Trực quan & Tương tác
Nhập liệu linh hoạt: Cho phép người dùng nhập địa chỉ kho hàng và thêm/xóa các điểm giao hàng một cách dễ dàng.

Bản đồ tương tác: Sử dụng Leaflet.js và nền bản đồ OpenStreetMap để hiển thị các tuyến đường một cách trực quan và chuyên nghiệp.

Phản hồi tức thì: Sử dụng thanh tiến trình (progress bar) để thông báo cho người dùng trong lúc hệ thống đang tính toán, cải thiện trải nghiệm người dùng.

✅ Hỗ trợ Đa chế độ Tối ưu
Tối ưu Quãng đường (TSP): Chế độ mặc định, tập trung tìm ra lộ trình có tổng quãng đường di chuyển là ngắn nhất.

Tối ưu Lịch trình (TSPTW): Chế độ nâng cao, cho phép người dùng đặt ra các "khung giờ vàng" (time windows) cho mỗi điểm giao hàng. Thuật toán sẽ tìm ra một lịch trình hợp lệ (không vi phạm khung giờ) với tổng thời gian (bao gồm cả di chuyển và chờ đợi) là ngắn nhất.

✅ So sánh Hiệu quả Thuật toán
Tính toán đồng thời: Khi ở chế độ "Tối ưu Quãng đường", hệ thống sẽ tự động chạy 3 thuật toán khác nhau trên cùng một bộ dữ liệu.

Bảng so sánh chi tiết: Hiển thị một bảng kết quả rõ ràng, so sánh các chỉ số quan trọng của từng thuật toán:

Quãng đường tối ưu tìm được.

Thời gian xử lý (đo bằng mili giây) để thấy được tốc độ của mỗi thuật toán.

Trực quan hóa đa lớp: Vẽ lộ trình của cả 3 thuật toán lên bản đồ với các màu sắc khác nhau. Cung cấp một bảng điều khiển cho phép người dùng bật/tắt hiển thị của từng lộ trình để dễ dàng so sánh sự khác biệt.

✅ Mô phỏng & Xử lý Sự cố Thực tế
Tương tác trực tiếp trên bản đồ: Người dùng có thể nhấn trực tiếp vào một đoạn đường trên bản đồ để mô phỏng sự cố tắc đường.

Tìm đường thay thế thông minh: Khi một đoạn đường bị chặn, hệ thống sẽ tự động tính toán lại một lộ trình mới tối ưu nhất để "né" đoạn đường đó và hiển thị kết quả cho người dùng.

3. Công nghệ & Thuật toán Sử dụng
Công nghệ
Backend:

Ngôn ngữ: Python 3

Web Framework: Flask

Thư viện: requests (để gọi API)

Frontend:

Ngôn ngữ: HTML, CSS, JavaScript

CSS Framework: Tailwind CSS (để xây dựng giao diện nhanh chóng)

Thư viện Bản đồ: Leaflet.js & Leaflet Routing Machine

API Dữ liệu Địa lý (Mã nguồn mở):

Geocoding (Tìm kiếm địa chỉ): Nominatim API (dựa trên OpenStreetMap)

Routing (Tính quãng đường/thời gian): OSRM (Open Source Routing Machine) API

Thuật toán
Thuật toán Heuristic (Tìm giải pháp ban đầu):

Nearest Neighbor (Người hàng xóm gần nhất): Dùng để nhanh chóng tạo ra một lộ trình ban đầu có chất lượng tương đối.

Thuật toán Cải thiện (Local Search):

2-Opt: Một thuật toán kinh điển và hiệu quả để "gỡ rối" các đoạn đường bắt chéo nhau, cải thiện đáng kể lộ trình ban đầu.

3-Opt: Một phiên bản mạnh mẽ hơn của 2-Opt, có khả năng tìm kiếm sâu hơn trong không gian giải pháp.

Thuật toán Metaheuristic:

Simulated Annealing (Mô phỏng tôi luyện - SA): Một thuật toán rất mạnh mẽ, có khả năng "nhảy" ra khỏi các điểm tối ưu cục bộ để tìm kiếm giải pháp tối ưu toàn cục. Đây là thuật toán chính được sử dụng cho bài toán TSPTW phức tạp.

4. Hướng dẫn Cài đặt & Chạy ứng dụng
Để chạy ứng dụng trên máy của bạn, hãy làm theo các bước sau:

Bước 1: Chuẩn bị Môi trường

Mở terminal (hoặc Command Prompt trên Windows) và di chuyển đến thư mục gốc của dự án. Tạo một môi trường ảo để cài đặt các thư viện:

# Lệnh cho Linux/macOS
python3 -m venv venv
source venv/bin/activate

# Lệnh cho Windows
python -m venv venv
.\venv\Scripts\activate

Bước 2: Cài đặt các thư viện cần thiết

Tạo một file tên là requirements.txt trong thư mục gốc của dự án với nội dung sau:

Flask
requests

Sau đó, chạy lệnh sau để cài đặt:

pip install -r requirements.txt

Bước 3: Chạy ứng dụng

Trong terminal, đảm bảo bạn vẫn đang ở trong môi trường ảo, chạy lệnh:

python3 app.py

Bước 4: Truy cập ứng dụng

Mở trình duyệt web của bạn (Chrome, Firefox,...) và truy cập vào địa chỉ sau:

http://127.0.0.1:5000

Giao diện ứng dụng sẽ hiện ra và bạn có thể bắt đầu sử dụng.