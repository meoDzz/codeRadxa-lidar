 
//Lưu ý: Code cần phát triển thêm phần lùi về

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Khởi tạo đối tượng đèn
Adafruit_NeoPixel pixels(1, 48, NEO_GRB + NEO_KHZ800);

//Định nghĩa các trạng thái cho Máy Trạng Thái (FSM)
#define STATE_DI_THANG 0
#define STATE_DUNG_CHO 1
#define STATE_DANG_LUI 2
#define STATE_DANG_QUAY 3

//Định nghĩa tham số thời gian (đơn vị: mili giây)
#define TG_DUNG_CHO 1100  // Dừng chờ 1 giây
#define TG_LUI_TONG 2500  // Lùi trong 1.5 giây
#define TG_QUAY 1000       // Quay trong 0.8 giây

//Khai báo các biến quản lý trạng thái và thời gian
int trang_thai_hien_tai = STATE_DI_THANG;  // Bắt đầu là đi thẳng

unsigned long last_loop_time = 0;                // Để tính delta time
unsigned long thoi_gian_bat_dau_trang_thai = 0;  // Lưu mốc thời gian bắt đầu hành động
long thoi_gian_lui_con_lai = 0;                  // Dùng long (có dấu) để trừ thời gian lùi


// --- Khai báo chân cho IR ----
#define IR_TRUOC_PIN_1 9  // Chân cảm biến IR phía trước -trái
#define IR_TRUOC_PIN_2 15 // Chân cảm biến IR phía trước - phải
#define IR_SAU_PIN_1 13   // chan cam bien IR phia sau - trái
#define IR_SAU_PIN_2 12   // chan cam bien IR phia sau - phải

//--IR phụ
#define IR_TRUOC_PIN_3 8  // Chân cảm biến IR phía trước -trái
#define IR_TRUOC_PIN_4 7 // Chân cảm biến IR phía trước - phải
#define IR_SAU_PIN_3 14   // chan cam bien IR phia sau - trái
#define IR_SAU_PIN_4 16   // chan cam bien IR phia sau - phải

// --- Khai báo chân cho dc1, dc2, dc3, dc4
#define MA_PWM 4// trái - trên
#define DC1_IN1_PIN 5
#define DC1_IN2_PIN 6

#define MB_PWM 4// 7- trái
#define DC2_IN1_PIN 5//15
#define DC2_IN2_PIN 6//16

#define MC_PWM 10//Bên phải - trước
#define DC3_IN1_PIN 17
#define DC3_IN2_PIN 18

#define MD_PWM 10 //13 - Bên phải - sau
#define DC4_IN1_PIN 17 //14
#define DC4_IN2_PIN 18 //21

// --- Định nghĩa trạng thái di chuyển ---
#define CHAY_THUAN 1
#define CHAY_NGHICH -1
#define DUNG_LAI 0

// --- Khai báo ham truoc
void Ngat_Uart();
void IRAM_ATTR IR_TRUOC();
void IRAM_ATTR IR_SAU();
void lidar_quet();
void dieuKhienDongCo(int mode, int Pin_A, int Pin_B);
void xe_dung();

// --- Biến toàn cục ---
// IR
volatile int status_ir_truoc_1 = LOW;
volatile int status_ir_sau_1 = LOW;
volatile int status_ir_truoc_2 = LOW;
volatile int status_ir_sau_2 = LOW;
volatile int status_ir_truoc_3 = LOW;
volatile int status_ir_sau_3 = LOW;
volatile int status_ir_truoc_4 = LOW;
volatile int status_ir_sau_4 = LOW;
// lidar
volatile float distance = 100;
volatile float angle = 0.0;

//----Hàm ngắt cho UART---------
void Ngat_Uart() {
  String data = Serial.readStringUntil('\n');
  // 2. Tách số: "%f,%f" nghĩa là tìm số thực, bỏ qua dấu phẩy, tìm số thực tiếp
  float t1, t2;
  if (sscanf(data.c_str(), "%f,%f", &t1, &t2) == 2) 
  {
    distance = t1;
    angle = t2;
  }
  lidar_quet();
}

// --- Hàm ngắt cho IR (ISR) ---
void IRAM_ATTR IR_TRUOC_1() {
  status_ir_truoc_1 = digitalRead(IR_TRUOC_PIN_1);
}
void IRAM_ATTR IR_SAU_1() {
  status_ir_sau_1 = digitalRead(IR_SAU_PIN_1);
}
void IRAM_ATTR IR_TRUOC_2() {
  status_ir_truoc_2 = digitalRead(IR_TRUOC_PIN_2);
}
void IRAM_ATTR IR_SAU_2() {
  status_ir_sau_2 = digitalRead(IR_SAU_PIN_2);
}
void IRAM_ATTR IR_TRUOC_3() {
  status_ir_truoc_3 = digitalRead(IR_TRUOC_PIN_3);
}
void IRAM_ATTR IR_SAU_3() {
  status_ir_sau_3 = digitalRead(IR_SAU_PIN_3);
}
void IRAM_ATTR IR_TRUOC_4() {
  status_ir_truoc_4 = digitalRead(IR_TRUOC_PIN_4);
}
void IRAM_ATTR IR_SAU_4() {
  status_ir_sau_4 = digitalRead(IR_SAU_PIN_4);
}

volatile int status_lidar_truoc = 0;
volatile int status_lidar_sau = 0;
void lidar_quet() {
  status_lidar_truoc = 0;
  status_lidar_sau = 0;
  if ((distance > 0.15 && distance < 0.4) && (angle > -45 && angle < 45)) {
    status_lidar_truoc = 1;
  } else if ((distance > 0.15 && distance < 0.4) && (angle > 135 || angle < -135)) {
    status_lidar_sau = 1;
  }
}

void setup() {
  Serial.begin(115200);
  // Cấu hình đèn LED RGB
  pixels.begin(); // Bắt đầu giao tiếp với LED
  pixels.setBrightness(50);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();

  // Cấu hình chân cảm biến
  pinMode(IR_TRUOC_PIN_1, INPUT_PULLUP);
  pinMode(IR_SAU_PIN_1, INPUT_PULLUP);
  pinMode(IR_TRUOC_PIN_2, INPUT_PULLUP);
  pinMode(IR_SAU_PIN_2, INPUT_PULLUP);
  pinMode(IR_TRUOC_PIN_3, INPUT_PULLUP);
  pinMode(IR_SAU_PIN_3, INPUT_PULLUP);
  pinMode(IR_TRUOC_PIN_4, INPUT_PULLUP);
  pinMode(IR_SAU_PIN_4, INPUT_PULLUP);


  // cau hinh chan dieu khien dong co
  pinMode(DC1_IN1_PIN, OUTPUT);
  pinMode(DC1_IN2_PIN, OUTPUT);
  pinMode(DC2_IN1_PIN, OUTPUT);
  pinMode(DC2_IN2_PIN, OUTPUT);
  pinMode(DC3_IN1_PIN, OUTPUT);
  pinMode(DC3_IN2_PIN, OUTPUT);
  pinMode(DC4_IN1_PIN, OUTPUT);
  pinMode(DC4_IN2_PIN, OUTPUT);
  pinMode(MA_PWM, OUTPUT);
  pinMode(MB_PWM, OUTPUT);
  pinMode(MC_PWM, OUTPUT);
  pinMode(MD_PWM, OUTPUT);
  analogWrite(MA_PWM, 0);
  analogWrite(MB_PWM, 0);
  analogWrite(MC_PWM, 0);
  analogWrite(MD_PWM, 0);

  //khoi tao ngat cho IRR
  attachInterrupt(digitalPinToInterrupt(IR_TRUOC_PIN_1), IR_TRUOC_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_SAU_PIN_1), IR_SAU_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_TRUOC_PIN_2), IR_TRUOC_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_SAU_PIN_2), IR_SAU_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_TRUOC_PIN_3), IR_TRUOC_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_SAU_PIN_3), IR_SAU_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_TRUOC_PIN_4), IR_TRUOC_4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_SAU_PIN_4), IR_SAU_4, CHANGE);

  //khoi tao ngat cho UART
  Serial.onReceive(Ngat_Uart);
  Serial.setTimeout(1);  // Tăng độ nhạy để kích hoạt ngắt nhanh hơn

  // Dừng xe khi khởi động
  xe_dung();
  int time = millis();
  while (millis() - time <= 3000) {};
}

//-------Hàm điều khiển động cơ------------------
void dieuKhienDongCo(int mode, int Pin_A, int Pin_B) {
  if (mode == CHAY_THUAN) {
    digitalWrite(Pin_A, HIGH);
    digitalWrite(Pin_B, LOW);
  } else if (mode == CHAY_NGHICH) {
    digitalWrite(Pin_A, LOW);
    digitalWrite(Pin_B, HIGH);
  } else {
    digitalWrite(Pin_A, LOW);
    digitalWrite(Pin_B, LOW);
  }
}

// --- Các hành vi của xe ---
void xe_tien() {
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); //Màu xanh lá cây
  pixels.show();
  analogWrite(MA_PWM, 100);
  analogWrite(MB_PWM, 100);
  analogWrite(MC_PWM, 100);
  analogWrite(MD_PWM, 100);
  dieuKhienDongCo(CHAY_THUAN, DC1_IN1_PIN, DC1_IN2_PIN);
  dieuKhienDongCo(CHAY_THUAN, DC2_IN1_PIN, DC2_IN2_PIN);
  dieuKhienDongCo(CHAY_THUAN, DC3_IN1_PIN, DC3_IN2_PIN);
  dieuKhienDongCo(CHAY_THUAN, DC4_IN1_PIN, DC4_IN2_PIN);
}

void xe_lui() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // màu đỏ
  pixels.show();
  analogWrite(MA_PWM, 100);
  analogWrite(MB_PWM, 100);
  analogWrite(MC_PWM, 100);
  analogWrite(MD_PWM, 100);
  dieuKhienDongCo(CHAY_NGHICH, DC1_IN1_PIN, DC1_IN2_PIN);
  dieuKhienDongCo(CHAY_NGHICH, DC2_IN1_PIN, DC2_IN2_PIN);
  dieuKhienDongCo(CHAY_NGHICH, DC3_IN1_PIN, DC3_IN2_PIN);
  dieuKhienDongCo(CHAY_NGHICH, DC4_IN1_PIN, DC4_IN2_PIN);
}

void xe_dung() {
  pixels.setPixelColor(0, pixels.Color(0,0,0)); 
  pixels.show();
  dieuKhienDongCo(DUNG_LAI, DC1_IN1_PIN, DC1_IN2_PIN);
  dieuKhienDongCo(DUNG_LAI, DC2_IN1_PIN, DC2_IN2_PIN);
  dieuKhienDongCo(DUNG_LAI, DC3_IN1_PIN, DC3_IN2_PIN);
  dieuKhienDongCo(DUNG_LAI, DC4_IN1_PIN, DC4_IN2_PIN);
}

void xe_quay() {  // Xoay tại chỗ
  // Bên trái chạy thuận, bên phải chạy nghịch (hoặc ngược lại)
  pixels.setPixelColor(0, pixels.Color(0,0,255)); // màu xanh dương
  pixels.show();
  analogWrite(MA_PWM, 145);
  analogWrite(MB_PWM, 145);
  analogWrite(MC_PWM, 110);
  analogWrite(MD_PWM, 110);
  dieuKhienDongCo(CHAY_THUAN, DC1_IN1_PIN, DC1_IN2_PIN);
  dieuKhienDongCo(CHAY_THUAN, DC2_IN1_PIN, DC2_IN2_PIN);
  dieuKhienDongCo(CHAY_NGHICH, DC3_IN1_PIN, DC3_IN2_PIN);
  dieuKhienDongCo(CHAY_NGHICH, DC4_IN1_PIN, DC4_IN2_PIN);
}

void loop() {
  // Tính khoảng thời gian trôi qua giữa 2 lần loop (Delta Time)
  unsigned long dt = millis() - last_loop_time;
  last_loop_time = millis();

  // --- MÁY TRẠNG THÁI ---
  switch (trang_thai_hien_tai) {

    // --------------------------------------
    // TRẠNG THÁI 1: ĐANG ĐI THẲNG
    // --------------------------------------
    case STATE_DI_THANG:
      // Logic: Nếu gặp vật cản -> Chuyển sang Dừng chờ
      if ( status_ir_truoc_1 == HIGH || status_ir_truoc_2 == HIGH || status_ir_truoc_3 == HIGH || status_ir_truoc_4 == HIGH || status_lidar_truoc == 1) 
      {
        xe_dung();
        // Cài đặt cho trạng thái tiếp theo
        thoi_gian_bat_dau_trang_thai = millis();
        trang_thai_hien_tai = STATE_DUNG_CHO;
      } else {
        xe_tien();  // Không có vật thì cứ đi
      }
      break;

    // --------------------------------------
    // TRẠNG THÁI 2: DỪNG CHỜ (Có phanh nghịch 50ms đầu)
    // --------------------------------------
    case STATE_DUNG_CHO:
    { // <--- BẠN ĐÃ MỞ NGOẶC Ở ĐÂY
      
      // Tính thời gian đã trôi qua kể từ khi bắt đầu vào trạng thái này
      unsigned long thoi_gian_da_qua = millis() - thoi_gian_bat_dau_trang_thai;

      // GIAI ĐOẠN 1: Hãm quán tính (trong 50ms đầu tiên)
      if (thoi_gian_da_qua < 150) {
        xe_lui(); // Kích hoạt động cơ quay ngược để phanh gấp
      } 
      // GIAI ĐOẠN 2: Dừng hẳn và chờ (từ 50ms đến 1000ms)
      else if (thoi_gian_da_qua < TG_DUNG_CHO) {
        xe_dung(); // Ngắt động cơ, xe đứng yên
      }
      // GIAI ĐOẠN 3: Chuyển trạng thái (sau 1000ms)
      else {
        // Cài đặt cho trạng thái lùi
        thoi_gian_lui_con_lai = TG_LUI_TONG; 
        trang_thai_hien_tai = STATE_DANG_LUI;
      }
    } // <--- BẠN BỊ THIẾU DẤU ĐÓNG NGOẶC NÀY (Đã bổ sung)
    break;

    // --------------------------------------
    // TRẠNG THÁI 3: ĐANG LÙI (Logic thông minh)
    // --------------------------------------
    case STATE_DANG_LUI:
    {
      // Khai báo biến static để lưu mốc thời gian gặp vật (chỉ khởi tạo 1 lần)
      static unsigned long thoi_gian_bat_dau_phanh = 0;

      // Gom điều kiện cảm biến vào biến bool cho gọn
      bool co_vat_can_sau = (status_ir_sau_1 == HIGH || status_ir_sau_2 == HIGH || status_ir_sau_3 == HIGH || status_ir_sau_4 == HIGH || status_lidar_sau == 1);

      if (co_vat_can_sau) 
      {
        // --- A. XỬ LÝ KHI CÓ VẬT CẢN ---
        
        // Nếu đây là lần đầu tiên phát hiện vật (biến thời gian đang = 0)
        if (thoi_gian_bat_dau_phanh == 0) {
          thoi_gian_bat_dau_phanh = millis(); // Lưu lại thời điểm hiện tại
        }

        // Tính xem đã phanh được bao lâu rồi
        if (millis() - thoi_gian_bat_dau_phanh < 150) {
          xe_tien(); // Trong 150ms đầu: Chạy tới để hãm quán tính
        } else {
          xe_dung(); // Sau 150ms: Dừng hẳn
          thoi_gian_bat_dau_trang_thai = millis();
          trang_thai_hien_tai = STATE_DANG_QUAY;
        }
        
        // Lưu ý: Khi đang xử lý vật cản thì KHÔNG trừ thời gian lùi (thoi_gian_lui_con_lai giữ nguyên)
      } 
      else 
      {
        // --- B. KHÔNG CÓ VẬT CẢN ---
        
        // Reset biến đếm phanh về 0 để sẵn sàng cho lần gặp vật tiếp theo
        thoi_gian_bat_dau_phanh = 0; 

        xe_lui(); // Lùi bình thường
        thoi_gian_lui_con_lai -= dt; // Trừ dần thời gian lùi
      }

      // Kiểm tra xem đã lùi hết giờ chưa (Tổng thời gian lùi thực tế)
      if (thoi_gian_lui_con_lai <= 0) {
        thoi_gian_bat_dau_phanh = 0; // Reset kỹ lưỡng trước khi chuyển trạng thái
        
        // Cài đặt cho trạng thái quay
        thoi_gian_bat_dau_trang_thai = millis();
        trang_thai_hien_tai = STATE_DANG_QUAY;
      }
    }
    break;

    // --------------------------------------
    // TRẠNG THÁI 4: ĐANG QUAY
    // --------------------------------------
    case STATE_DANG_QUAY:
      xe_quay();

      // Kiểm tra xem đã quay đủ giờ chưa
      if (millis() - thoi_gian_bat_dau_trang_thai >= TG_QUAY) {
        // Reset biến Lidar ảo để tránh kẹt
        status_lidar_truoc = 0;
        while(Serial.available()) Serial.read();
        // Quay về trạng thái ban đầu
        trang_thai_hien_tai = STATE_DI_THANG;
      }
      break;
  }
}