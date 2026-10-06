/*
 * Real-time telemetry device — main firmware
 * Board:   LILYGO T-Display-S3 (ESP32-S3) + custom 4-layer carrier PCB
 * Sensors: u-blox MAX-M10S (GNSS, I2C) · Bosch BNO055 (9-axis IMU, I2C)
 * UI:      LVGL, designed in SquareLine Studio (ui_*.c / ui_*.h, see firmware/README.md)
 *
 * Author:  Julián Samblás Caballero — Bachelor's thesis, University of Málaga (2024)
 */

#define LCD_MODULE_CMD_1
#define USE_UI

#include "OneButton.h" /* https://github.com/mathertel/OneButton.git */
#include "lvgl.h"      /* https://github.com/lvgl/lvgl.git */
#include <TFT_eSPI.h>
#include "Arduino.h"
#include "Wire.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "ui.h"
#include "pin_config.h"
#include "esp_sntp.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // u-blox MAX-M10S

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
int Screen = 1;

double linearAccelDataX = 0;
double linearAccelDataY = 0;
double linearAccelDataZ = 0;

double latitude;
int altitude;
// int speedAccEst;
double longitude;
double SIV;          // satellites in view
int nedNorthVel;     // mm/s
int nedEastVel;      // mm/s
int nedDownVel;      // mm/s

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

SFE_UBLOX_GNSS myGNSS;
long lastTime = 0;   // GNSS polling timer

// Button handler: toggles between the speed screen and the data screen
void IRAM_ATTR toggleLCD()
{
    if (Screen == 1) {
        lv_disp_load_scr(ui_Screen2);
        Screen = 2;
    } else {
        lv_disp_load_scr(ui_Screen1);
        Screen = 1;
    }
}

bool enabled = 0;
OneButton button1(PIN_BUTTON_1, true);
OneButton button2(PIN_BUTTON_2, true);

// ---------------------------------------------------------------------------
// Display driver (ST7789 over 8-bit i80 bus) + LVGL glue
// ---------------------------------------------------------------------------
esp_lcd_panel_io_handle_t io_handle = NULL;
static lv_disp_draw_buf_t disp_buf; // internal LVGL draw buffers
static lv_disp_drv_t disp_drv;      // display driver callbacks
static lv_color_t *lv_disp_buf;
static bool is_initialized_lvgl = false;

#if defined(LCD_MODULE_CMD_1)
typedef struct {
    uint8_t cmd;
    uint8_t data[14];
    uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};
#endif

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                            esp_lcd_panel_io_event_data_t *edata,
                                            void *user_ctx)
{
    if (is_initialized_lvgl) {
        lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
        lv_disp_flush_ready(disp_driver);
    }
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

// Forward declarations
void ui_reset();
void GPSinit();
void BNOinit();
void printEvent(sensors_event_t *event);
void updateBNO();
void updateGPS();

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup()
{
    // (POWER ON) IO15 must be HIGH before starting, otherwise the display
    // will not turn on when running from battery.
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    Serial.begin(115200);

    pinMode(PIN_LCD_RD, OUTPUT);
    digitalWrite(PIN_LCD_RD, HIGH);

    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .wr_gpio_num = PIN_LCD_WR,
        .clk_src = LCD_CLK_SRC_PLL160M,
        .data_gpio_nums = {
            PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3,
            PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7,
        },
        .bus_width = 8,
        .max_transfer_bytes = LVGL_LCD_BUF_SIZE * sizeof(uint16_t),
        .psram_trans_align = 0,
        .sram_trans_align = 0
    };
    esp_lcd_new_i80_bus(&bus_config, &i80_bus);

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 20,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RES,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
        .vendor_config = NULL
    };
    esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, false, true);
    esp_lcd_panel_set_gap(panel_handle, 0, 35);

#if defined(LCD_MODULE_CMD_1)
    for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
        esp_lcd_panel_io_tx_param(io_handle, lcd_st7789v[i].cmd, lcd_st7789v[i].data,
                                  lcd_st7789v[i].len & 0x7f);
        if (lcd_st7789v[i].len & 0x80)
            delay(120);
    }
#endif

    /* Lighten the screen with gradient */
    ledcSetup(0, 10000, 8);
    ledcAttachPin(PIN_LCD_BL, 0);
    for (uint8_t i = 0; i < 0xFF; i++) {
        ledcWrite(0, i);
        delay(2);
    }

    lv_init();
    lv_disp_buf = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t),
                                                 MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf, NULL, LVGL_LCD_BUF_SIZE);

    /* Display driver init */
    lv_disp_drv_init(&disp_drv);
    /* Screen resolution */
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    is_initialized_lvgl = true;

    // Load the UI exported from SquareLine Studio
    ui_init();
    // Reset all sensor values shown on screen to 0
    ui_reset();

    attachInterrupt(PIN_BUTTON_2, toggleLCD, RISING);

    GPSinit();
    BNOinit();
}

// Reset all on-screen values to 0
void ui_reset()
{
    lv_slider_set_value(ui_Slider1, 0, LV_ANIM_OFF);
    lv_label_set_text(ui_Labelfront, "0");
    lv_label_set_text(ui_Satelitesnumero, "0");
    lv_label_set_text(ui_Aceleracionxd, "0");
    lv_label_set_text(ui_aceleracionyd, "0");
    lv_label_set_text(ui_Aceleracionzd, "0");
    lv_label_set_text(ui_Gradoslatitudn, "0");
    lv_label_set_text(ui_Gradoslongitudn, "0");
    lv_label_set_text(ui_Altitudt, "0");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop()
{
    lv_timer_handler();
    delay(10);

    if (millis() > 2000 && enabled == 0) {
        enabled = 1;
    }

    // --- GNSS ---
    // Query the module only once per second; polling faster only adds I2C traffic.
    // The module only answers when a new position is available.
    if (millis() - lastTime > 1000) {
        lastTime = millis(); // update the timer

        latitude = myGNSS.getLatitude();
        Serial.print(F("Lat: "));
        Serial.print(latitude);

        longitude = myGNSS.getLongitude();
        Serial.print(F(" Long: "));
        Serial.print(longitude);
        Serial.print(F(" (degrees * 10^-7)"));

        altitude = myGNSS.getAltitude();
        Serial.print(F(" Alt: "));
        Serial.print(altitude);
        Serial.print(F(" (mm)"));

        SIV = myGNSS.getSIV();
        Serial.print(F(" SIV: "));
        Serial.print(SIV);

        nedNorthVel = myGNSS.getNedNorthVel();
        Serial.print(F(" VelN: "));
        Serial.print(nedNorthVel);
        Serial.print(F(" (mm/s)"));

        nedEastVel = myGNSS.getNedEastVel();
        Serial.print(F(" VelE: "));
        Serial.print(nedEastVel);
        Serial.print(F(" (mm/s)"));

        nedDownVel = myGNSS.getNedDownVel();
        Serial.print(F(" VelD: "));
        Serial.print(nedDownVel);
        Serial.print(F(" (mm/s)"));

        // Alternative (less accurate) speed estimate, kept for reference:
        // speedAccEst = myGNSS.getSpeedAccEst();
        // Serial.print(F(" SpeedAccEst: "));
        // Serial.print(speedAccEst);
        // Serial.print(F(" (mm/s)"));

        Serial.println();

        // --- IMU ---
        // Any other vector (e.g. gravity) can be added to the measurements.
        sensors_event_t orientationData, angVelocityData, linearAccelData,
                        magnetometerData, accelerometerData, gravityData;
        bno.getEvent(&orientationData,   Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelocityData,   Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&linearAccelData,   Adafruit_BNO055::VECTOR_LINEARACCEL);
        bno.getEvent(&magnetometerData,  Adafruit_BNO055::VECTOR_MAGNETOMETER);
        bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        bno.getEvent(&gravityData,       Adafruit_BNO055::VECTOR_GRAVITY);

        printEvent(&orientationData);
        printEvent(&angVelocityData);
        printEvent(&linearAccelData);
        printEvent(&magnetometerData);
        printEvent(&accelerometerData);
        printEvent(&gravityData);

        int8_t boardTemp = bno.getTemp();
        Serial.println();
        Serial.print(F("temperature: "));
        Serial.println(boardTemp);

        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.println();
        Serial.print("Calibration: Sys=");
        Serial.print(system);
        Serial.print(" Gyro=");
        Serial.print(gyro);
        Serial.print(" Accel=");
        Serial.print(accel);
        Serial.print(" Mag=");
        Serial.println(mag);
        Serial.println("--");

        delay(BNO055_SAMPLERATE_DELAY_MS);
    }

    updateBNO();
    updateGPS();
    delay(10);
}

// ---------------------------------------------------------------------------
// Sensor initialisation (both sensors share the I2C bus on GPIO1/GPIO2)
// ---------------------------------------------------------------------------
void BNOinit()
{
    Serial.println("Orientation Sensor Test");
    Serial.println("");
    Wire.begin(1, 2);

    /* Initialise the sensor */
    if (!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    delay(1000);
}

void GPSinit()
{
    Serial.println("SparkFun u-blox Example");
    Wire.begin(1, 2);

    if (myGNSS.begin() == false) { // connect to the u-blox module over Wire
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // UBX only on I2C (turn off NMEA noise)
    myGNSS.setNavigationFrequency(2);  // two navigation solutions per second
    myGNSS.setAutoPVT(true);           // module sends every solution automatically
    myGNSS.saveConfiguration();        // persist the configuration in the module
}

// ---------------------------------------------------------------------------
// Print an IMU event to the serial monitor and store linear acceleration
// ---------------------------------------------------------------------------
void printEvent(sensors_event_t *event)
{
    double x = -1000000, y = -1000000, z = -1000000; // dumb values, easy to spot problem

    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        Serial.print("Accl:");
        x = event->acceleration.x;
        linearAccelDataX = event->acceleration.x;
        y = event->acceleration.y;
        linearAccelDataY = event->acceleration.y;
        z = event->acceleration.z;
        linearAccelDataZ = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION) {
        Serial.print("Orient:");
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        Serial.print("Mag:");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    }
    else if (event->type == SENSOR_TYPE_GYROSCOPE) {
        Serial.print("Gyro:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
        Serial.print("Rot:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
        Serial.print("Linear:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_GRAVITY) {
        Serial.print("Gravity:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else {
        Serial.print("Unk:");
    }

    Serial.print("\tx= ");
    Serial.print(x);
    Serial.print(" |\ty= ");
    Serial.print(y);
    Serial.print(" |\tz= ");
    Serial.println(z);
}

// ---------------------------------------------------------------------------
// UI updates
// ---------------------------------------------------------------------------
// IMU accelerations on screen 2
void updateBNO()
{
    lv_label_set_text(ui_Aceleracionxd, String(linearAccelDataX).c_str());
    lv_label_set_text(ui_aceleracionyd, String(linearAccelDataY).c_str());
    lv_label_set_text(ui_Aceleracionzd, String(linearAccelDataZ).c_str());
}

// GNSS values on screens 1 and 2.
// The receiver reports NED velocity in mm/s: compute the magnitude, then convert to km/h.
void updateGPS()
{
    int velocidad = sqrt(sq(nedNorthVel) + sq(nedEastVel) + sq(nedDownVel));
    int velocidadkmh = velocidad * 0.0036;

    lv_label_set_text(ui_Labelfront, String(velocidadkmh).c_str());
    lv_slider_set_value(ui_Slider1, int(velocidadkmh), LV_ANIM_ON);
    lv_label_set_text(ui_Satelitesnumero, String(SIV).c_str());
    lv_label_set_text(ui_Gradoslatitudn, String(latitude).c_str());
    lv_label_set_text(ui_Gradoslongitudn, String(longitude).c_str());

    int Altitude1 = altitude / 100;
    lv_label_set_text(ui_Altitudt, String(Altitude1).c_str());
}
