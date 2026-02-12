# 🐧 Hướng dẫn sử dụng tmux (Terminal Multiplexer)

**tmux** là một công cụ cực kỳ mạnh mẽ giúp bạn quản lý nhiều cửa sổ terminal trong một kết nối duy nhất. Rất hữu ích khi làm việc từ xa qua SSH hoặc khi bạn muốn chia màn hình làm việc linh hoạt.

---

## 1. Cấu trúc của tmux
Một phiên làm việc của tmux được chia làm 3 cấp độ:
* **Session**: Một phiên làm việc lớn (ví dụ: dự án A, dự án B).
* **Window**: Giống như các tab trên trình duyệt web.
* **Pane**: Các ô nhỏ được chia ra trong cùng một Window.



---

## 2. Phím tắt "Prefix" thần thánh
Tất cả các lệnh điều khiển bên trong tmux đều bắt đầu bằng tổ hợp phím mặc định:
> **`Ctrl + b`** (gọi tắt là **Prefix**)

*Cách dùng: Nhấn `Ctrl` và `b` cùng lúc, thả ra, sau đó mới nhấn phím chức năng tiếp theo.*

---

## 3. Các lệnh điều khiển Pane (Chia màn hình)
Sau khi nhấn `Ctrl + b`:

| Phím tắt | Chức năng |
| :--- | :--- |
| `%` | Chia đôi màn hình theo **chiều dọc** |
| `"` | Chia đôi màn hình theo **chiều ngang** |
| `Phím mũi tên` | Di chuyển giữa các Pane |
| `z` | Phóng to (Zoom) Pane hiện tại (nhấn lại để thu nhỏ) |
| `x` | Đóng Pane hiện tại (hoặc gõ `exit`) |
| `space` | Thay đổi layout sắp xếp các Pane |

---

## 4. Các lệnh điều khiển Window (Tab)
Sau khi nhấn `Ctrl + b`:

| Phím tắt | Chức năng |
| :--- | :--- |
| `c` | Tạo một Window mới (Create) |
| `n` | Chuyển sang Window tiếp theo (Next) |
| `p` | Chuyển về Window phía trước (Previous) |
| `0..9` | Chuyển nhanh đến Window số tương ứng |
| `,` | Đổi tên Window hiện tại |
| `w` | Hiển thị danh sách Window để chọn |

---

## 5. Quản lý Session (Từ Terminal)
Các lệnh này gõ trực tiếp ở dấu nhắc dòng lệnh bên ngoài:

* **Tạo session mới có tên:**
    ```bash
    tmux new -s ten_du_an
    ```
* **Rời khỏi session (Detach):** Nhấn `Ctrl + b` rồi nhấn `d`. (Session vẫn chạy ngầm).
* **Xem danh sách session đang chạy:**
    ```bash
    tmux ls
    ```
* **Kết nối lại (Attach) vào session gần nhất:**
    ```bash
    tmux attach
    ```
* **Kết nối lại vào session cụ thể:**
    ```bash
    tmux attach -t ten_du_an
    ```
* **Xóa session:**
    ```bash
    tmux kill-session -t ten_du_an
    ```

---

## 6. Mẹo nhỏ: Cho phép dùng chuột
Mặc định tmux không cho dùng chuột để chuyển Pane hay cuộn. Để bật, hãy tạo file cấu hình:

1. Gõ `nano ~/.tmux.conf`
2. Dán dòng này vào: `set -g mouse on`
3. Lưu lại và khởi động lại tmux hoặc gõ lệnh: `tmux source-file ~/.tmux.conf`

---
