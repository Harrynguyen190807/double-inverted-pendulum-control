---
name: dip-git-devops
description: >-
  Quản trị Kho mã nguồn Git, Tiêu chuẩn CI/CD và Kiểm thử Tự động cho Dự án DIP.
  Bao gồm kiểm tra cú pháp Python, unit tests kiểm tra tính điều khiển được và tính ổn định
  vòng kín của nghiệm Riccati LQR, cùng cấu hình GitHub Actions workflow (.github/workflows/ci.yml).
---

# Skill: `dip-git-devops`

## Tổng quan
Skill này phụ trách toàn diện quy trình kỹ nghệ phần mềm chất lượng cao, tích hợp liên tục (CI/CD), kiểm thử tự động (Unit Testing) và chuẩn hóa cấu trúc repository cho dự án Con lắc ngược đôi.

## Khi nào sử dụng
- Khi khởi tạo hoặc quản lý vòng đời Git repository cho dự án.
- Khi cần kiểm tra tính toàn vẹn của repo trước khi commit hoặc merge pull request.
- Khi thêm mới hoặc thay đổi các tham số vật lý của hệ thống và cần đảm bảo các điều kiện ổn định toán học không bị vi phạm:
  1. Hạng ma trận điều khiển được Kalman: $\text{rank}(\mathcal{C}) == 6$.
  2. Nghiệm phương trình Riccati liên tục $P$ là ma trận đối xứng xác định dương ($P = P^T > 0$).
  3. Tất cả các cực của hệ vòng kín $A - BK$ đều nằm ở nửa trái mặt phẳng phức: $\text{Re}(\lambda) < 0$.
- Khi xây dựng hoặc sửa đổi pipeline tự động `.github/workflows/ci.yml` trên GitHub Actions.

## Các lệnh & Công cụ thực thi

### 1. Khởi tạo Git Repo và Lưu Snapshot
```bash
git init
git add .
git commit -m "feat: initial commit with complete DIP modeling, simulation, and embedded firmware"
```

### 2. Chạy Kiểm thử Đơn vị LQR Stability (Unit Test)
```bash
pytest tests/test_lqr_stability.py -v
# Hoặc chạy trực tiếp bằng python:
python tests/test_lqr_stability.py
```

### 3. Cấu hình GitHub Actions CI Workflow
File cấu hình tại: `.github/workflows/ci.yml`
Pipeline tự động thực hiện:
- Kiểm tra cú pháp các file Python và C.
- Cài đặt các thư viện trong `requirements.txt`.
- Chạy toàn bộ test suites trong `tests/`.
- Kiểm tra tính ổn định tiệm cận của ma trận LQR mỗi khi có commit hoặc pull request.
