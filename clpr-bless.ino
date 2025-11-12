/**
 * @file clpr-bless.ino
 * @author Takuro Fukamizu (takuro.f.1987@gmail.com)
 * @brief GGT2025 Demo.
 * @version 0.1
 * @date 2025-11-12
 *
 *
 * @Hardwares: M5Stack Core2, M5Unit ToF4M, M5Unit UnitRCA, Takaha Multi Controller A
 * @Platform Version: Arduino M5Stack Board Manager v2.0.7
 * @Dependent Library:
 * VL53L1X: https://github.com/pololu/vl53l1x-arduino
 * M5Unified: https://github.com/m5stack/M5Unified
 * EspEasyUtils: https://github.com/tanakamasayuki/EspEasyUtils
 * QuickStats: https://github.com/dndubins/QuickStats
 */

#include <M5Unified.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <EspEasyLED.h>
#include <EspEasyTask.h>
#include <QuickStats.h>

#define MEASRE_INTERVAL_MS 80
#define PIN_SOL01 GPIO_NUM_13 // PortC pin1
#define PIN_SOL02 GPIO_NUM_14 // PortC pin2
#define PIN_RCA01 GPIO_NUM_36 // PortB pin1
#define PIN_RCA02 GPIO_NUM_26 // PortB pin2
#define PIN_LED GPIO_NUM_27

// PortA: ToF(i2c)
// PortB: UnitRCA
// PortC: Solenoid
// GPIO2: RGB LED


//-------------------------------

EspEasyLED perfomanceLed(PIN_LED, 8, 128); // RGB LED

EspEasyTask solenoidTask;
EspEasyTask performanceTask;

//-------------------------------

VL53L1X sensor;

int calib = 40; // mm
int min_distance = 40;
int max_distance = 140;

int num_distances = 5;
float distances[5];

bool solenoid_enabled = false;
int prevMode = 0;
int value_ml = 0;

//-------------------------------

void startPeformance();
void stopPerformance();

void solenoidTaskLoop();
void performanceTaskLoop();

//-------------------------------

void setup() {
    auto cfg = M5.config();        // 既定設定を取得
    cfg.clear_display   = true;
    // cfg.external_speaker= false;
    cfg.internal_imu    = false;   // 使っていなければOFF（任意）
    cfg.output_power    = true;    // Core2のPMU電源出力をON
    M5.begin(cfg);

    M5.Log.setLogLevel(m5::log_target_serial, ESP_LOG_VERBOSE);
    M5.Log.setLogLevel(m5::log_target_serial, ESP_LOG_INFO);

    // The primary display can be used with M5.Display.
    M5.Display.print("Initialize...\n");

    // ----

    M5.Display.print(" - Peripherals: ");
    pinMode(PIN_SOL01, OUTPUT);
    pinMode(PIN_SOL02, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    perfomanceLed.showColor(EspEasyLEDColor::RED);
    M5.Display.println("    OK");

    // ----

    // M5.Ex_I2C.begin(32, 33, 400000);
    Wire.begin(M5.Ex_I2C.getSDA(), M5.Ex_I2C.getSCL(), 100000);
    sensor.setBus(&Wire);
    sensor.setTimeout(500);
    M5.Display.print(" - ToF: ");
    if (!sensor.init()) {
        M5.Display.println("    Failed to detect and initialize sensor!");
        while (1)
            ;
    }
    M5.Display.println("    OK");

    // Use long distance mode and allow up to 50000 us (50 ms) for a
    // measurement. You can change these settings to adjust the performance of
    // the sensor, but the minimum timing budget is 20 ms for short distance
    // mode and 33 ms for medium and long distance modes. See the VL53L1X
    // datasheet for more information on range and timing limits.
    // https://www.st.com/resource/en/datasheet/vl53l1x.pdf
    // Short: 136/135cm
    // Medium: 290/76cm
    // Long: 360/73cm
    sensor.setDistanceMode(VL53L1X::Short);

    //測定タイミングバジェット(1回の距離測定)に許容される時間[ms]
    sensor.setMeasurementTimingBudget(MEASRE_INTERVAL_MS * 1000); // 50ms


    // 例1) FoVを約1/2（≈13°）に：8x8、中心はデフォルト(199)
    sensor.setROISize(8, 8);
    sensor.setROICenter(199); // 中央

    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    //連続測定モードの測定間隔[msec]　0を指定すると可能な限り最速になる
    sensor.startContinuous(MEASRE_INTERVAL_MS);

    // ----
    // randomSeed(analogRead(0)); 
    randomSeed(esp_random()); // ESP32のHW RNGを利用
    // ----

    perfomanceLed.showColor(EspEasyLEDColor::GREEN);    
    M5.delay(1000); // 1s
    perfomanceLed.clear();
    M5.Display.fillScreen(BLACK);
    M5.Log(ESP_LOG_INFO, "init complete");
}


void loop() {
    // ToFセンサーから数回値を読み取り、フローの距離を取得する
    for (int i = 0; i < num_distances; i++) {
        sensor.read();
        if (sensor.timeoutOccurred()) { 
            Serial.print(" TIMEOUT"); 
            continue;
        }
        if (sensor.ranging_data.range_status != VL53L1X::RangeStatus::RangeValid) {
            // 測定値が信用できない場合はスキップ
            Serial.printf("status: %20s -> skip\n", VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
            continue;
        }
        distances[i] = sensor.ranging_data.range_mm;
        M5.delay(MEASRE_INTERVAL_MS); // 50ms
    }
    QuickStats stats;//QuickStatsを初期化
    int value_mm = (int)stats.median(distances, num_distances); //中央値

    // ToFセンサーから取得したフローの距離より、吸気量(ml)に変換する
    value_ml = 0;
    if (value_mm < min_distance) {
        value_ml = 0;
        // TODO: 表示用の例外処理
    } else if (max_distance < value_mm) {
        value_ml = 5000;
        // TODO: 表示用の例外処理
    } else { // 有効範囲内
        value_ml = (value_mm - min_distance) * 50;
    }

    // 吸気量(ml)より動作条件を決定
    if (value_mm < min_distance * 1.1) {
        if (prevMode == 1) {
            stop();
        }
        prevMode = 0; // not run
    } else {
        if (prevMode == 0) {
            start();
        }
        prevMode = 1; // run
    }

    // 画面に情報を表示
    M5.Display.setFont(&Font4);
    M5.Display.setTextSize(1); 
    M5.Display.setCursor(0, 10);
    M5.Display.printf("Distance: %04d mm\n", value_mm);
    M5.Display.printf("Volume  : %04d ml\n", value_ml);

    M5.Display.setFont(&Font0);
    M5.Display.setTextSize(1); 
    M5.Display.setCursor(0, 60);
    M5.Display.printf("status     : %20s\n", VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
    M5.Display.printf("peak signal: %8.4f\n", sensor.ranging_data.peak_signal_count_rate_MCPS);
    M5.Display.printf("ambient    : %8.4f\n", sensor.ranging_data.ambient_count_rate_MCPS);

    M5.delay(1);
}


//-------------------------------

void start() {
    M5.Log(ESP_LOG_INFO, "startPeformance");
    solenoid_enabled = true;
    solenoidTask.begin(solenoidTaskLoop, 2, ESP_EASY_TASK_CPU_NUM);
    // performanceTask.begin(performanceTaskLoop, 2, ESP_EASY_TASK_CPU_NUM);
}

void stop() {
    M5.Log(ESP_LOG_INFO, "stopPerformance");
    solenoid_enabled = false;
    solenoidTask.suspend(); // stop Task
    // performanceTask.suspend(); // stop Task
    perfomanceLed.clear(); // trun off LED
    digitalWrite(PIN_SOL01, LOW); // turn off Solenoid
    digitalWrite(PIN_SOL02, LOW); // turn off Solenoid

}

void solenoidTaskLoop() {
    while(1){
        if (!solenoid_enabled) {
            // ソレノイド実行中でなければ全てを停止してスキップ
            digitalWrite(PIN_SOL01, LOW);
            perfomanceLed.showColor(EspEasyLEDColor::BLACK);
            perfomanceLed.show();
            M5.delay(100); // 100ms
            continue;
        }
        // ----
        // 計測結果より、パフォーマンスの動作条件を決定する
        uint8_t point = 0;
        uint8_t delay_ms = 0;
        if (value_ml < 1000) {
            point = 1;
            delay_ms = 1400;
        } else if (value_ml < 2000) {
            point = 2;
            delay_ms = 1200;
        } else if (value_ml < 3000) {
            point = 4;
            delay_ms = 1000;
        } else if (value_ml < 4000) {
            point = 5;
            delay_ms = 800;
        } else { // over 4000ml
            point = 6;
            delay_ms = 600;
        }
        // LEDとソレノイドを動作させる
        uint8_t r, g, b;
        M5.Log.printf("solenoid_enabled %d, %d\n", point, delay_ms);
        M5.Log.printf("led num: %d", perfomanceLed.getLedNum());

        // on phase
        M5.Log(ESP_LOG_INFO, "on");
        digitalWrite(PIN_SOL01, HIGH);
        digitalWrite(PIN_SOL02, HIGH);
        // // fade in 0(MIN) to 100(MAX)
        // for (int i = 1; i <= 10; i++) {
        //     r = random(256);
        //     g = random(256);
        //     b = random(256);
        //     perfomanceLed.showColor(r, g, b);
        //     perfomanceLed.setBrightness(i * 10);
        //     perfomanceLed.show();
        //     M5.delay(delay_ms / 10);
        // }
        perfomanceLed.showColor(EspEasyLEDColor::WHITE);
        M5.delay(delay_ms);

        // off phase
        M5.Log(ESP_LOG_INFO, "phase");
        digitalWrite(PIN_SOL01, LOW);
        digitalWrite(PIN_SOL02, LOW);
        perfomanceLed.showColor(EspEasyLEDColor::BLACK);
        M5.delay(delay_ms);
    }
}

void performanceTaskLoop() {
  while(1){
    uint8_t r, g, b, brightness;
    for(int i = 0; i < perfomanceLed.getLedNum(); i++) {
        r = random(256);
        g = random(256);
        b = random(256);
        brightness = random(90)+10;
        perfomanceLed.setColor(i, r, g, b);
        perfomanceLed.setBrightness(brightness);
        perfomanceLed.show();
    }
    M5.delay(200);
  }
}