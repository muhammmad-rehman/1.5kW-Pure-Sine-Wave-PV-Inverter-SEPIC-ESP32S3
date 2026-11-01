#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "driver/mcpwm_prelude.h"

// ==========================================
// 1. PIN DEFINITIONS
// ==========================================
#define HIN_A 4
#define LIN_A 5
#define HIN_B 10
#define LIN_B 11       

#define SEPIC_PIN 7
#define I2C_SDA 8
#define I2C_SCL 9

#define BUZZER_PIN 1   
#define V_AC_SENSOR 16 
#define I_AC_SENSOR 18  

const int sensorPins[] = {12, 13, 14}; // Temp Sensors

// ==========================================
// 2. SYSTEM PARAMETERS
// ==========================================
#define CARRIER_FREQ 30000       
#define SPWM_UPDATE_FREQ 10000   
#define SINE_FREQ 50             

#define MOD_INDEX 1.0            
#define DEADTIME_US 1.5          

#define SEPIC_FREQ 60000         
#define BUZZER_FREQ 4000         

const float PI_P = 0.001;
const float PI_I = 0.05;
const float DT = 0.4; 

// Calibration Constants
float V_SCALE = 0.712;   
float I_SCALE = 0.016;  

// NTC Thermistor Constants
const float R_FIXED = 10000.0;
const float NOMINAL_R = 10000.0;
const float B_COEFFICIENT = 3950.0;
const float TEMP_THRESHOLD = 125.0;

// ==========================================
// 3. GLOBAL VARIABLES & BUFFERS
// ==========================================
#define RMS_SAMPLES 1024
float v_buffer[RMS_SAMPLES];
float i_buffer[RMS_SAMPLES];
volatile int rms_index = 0;

volatile float v_ac_rms = 0.0;
volatile float i_ac_rms = 0.0;
volatile float apparent_power = 0.0;
volatile float temps[3] = {0.0, 0.0, 0.0};

// SPWM Math Arrays 
const int num_points = SPWM_UPDATE_FREQ / SINE_FREQ; 
float duty_A_array[200];
float duty_B_array[200];

// --- State Machine Timers & Variables ---
int step_index = 0;
unsigned long last_spwm_update = 0; 
unsigned long last_lcd_update = 0; 
unsigned long last_pi_update = 0;

// SEPIC & Fault Tracking
float sepic_duty = 0.0;          
float integral_error = 0.0;
bool startup_phase = true;       

// Unified Fault Management
enum FaultState { NONE, OVERLOAD, UNDERVOLTAGE, OVERTEMP };
volatile FaultState active_fault = NONE;       

// v3.x Hardware Handles
mcpwm_timer_handle_t spwm_timer = NULL; 
mcpwm_cmpr_handle_t cmpr_a = NULL;
mcpwm_cmpr_handle_t cmpr_b = NULL;
uint32_t timer_period; 

// FreeRTOS Task Handle
TaskHandle_t SensorTask;

// LCD & Custom Characters
LiquidCrystal_I2C lcd(0x27, 16, 2);
byte subscriptO[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x12, 0x0C};
byte sub1[8]       = {0x00, 0x00, 0x00, 0x04, 0x0C, 0x04, 0x04, 0x0E};
byte sub2[8]       = {0x00, 0x00, 0x00, 0x1E, 0x02, 0x0C, 0x10, 0x1E};
byte sub3[8]       = {0x00, 0x00, 0x00, 0x1E, 0x02, 0x0E, 0x02, 0x1E};

// --- Custom RMS Math Function ---
float computeRMS(float *buffer) {
  float sum = 0;
  for (int i = 0; i < RMS_SAMPLES; i++) sum += buffer[i] * buffer[i];
  return sqrt(sum / RMS_SAMPLES);
}

// ==========================================
// 4. CORE 0 TASK: RMS BUFFER & TEMP SENSOR
// ==========================================
void sensorTaskFunction(void * pvParameters) {
  float v_offset = 0; 
  float i_offset = 0;
  int loop_counter = 0;
  
  // Take 200 initial samples to find the zero-point DC offset
  for (int i = 0; i < 200; i++) {
    v_offset += analogRead(V_AC_SENSOR);
    i_offset += analogRead(I_AC_SENSOR);
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
  v_offset /= 200.0; 
  i_offset /= 200.0;

  for(;;) {
    if (active_fault == NONE) {
      // --- 1. Fast AC RMS Sampling ---
      int rv = analogRead(V_AC_SENSOR);
      int ri = analogRead(I_AC_SENSOR);

      v_buffer[rms_index] = (float)rv - v_offset;
      i_buffer[rms_index] = (float)ri - i_offset;
      if (++rms_index >= RMS_SAMPLES) rms_index = 0;

      float temp_v = computeRMS(v_buffer) * V_SCALE;
      float temp_i = computeRMS(i_buffer) * I_SCALE;

      v_ac_rms = (temp_v > 300.0 || temp_v < 3.0) ? 0.0 : temp_v;
      i_ac_rms = (temp_i > 50.0 || temp_i < 0.05) ? 0.0 : temp_i;
      apparent_power = v_ac_rms * i_ac_rms;

      // --- 2. Slower Temperature Sampling (Every ~500ms) ---
      // We run this every 250 loops of the 2ms delay = ~500ms
      if (++loop_counter >= 250) {
        loop_counter = 0;
        for (int i = 0; i < 3; i++) {
          int raw = analogRead(sensorPins[i]);
          if (raw >= 4095) {
            temps[i] = 0; 
          } else {
            float resistance = R_FIXED * ((4095.0 / (float)raw) - 1.0);
            float steinhart = resistance / NOMINAL_R;
            steinhart = log(steinhart);
            steinhart /= B_COEFFICIENT;
            steinhart += 1.0 / (25.0 + 273.15);
            steinhart = 1.0 / steinhart;
            temps[i] = steinhart - 273.15;
          }

          if (temps[i] >= TEMP_THRESHOLD) {
            active_fault = OVERTEMP; // Trigger fault globally
          }
        }
      }
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// 5. SETUP SEQUENCE
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  lcd.init();
  lcd.backlight();
  
  // Load custom characters
  lcd.createChar(0, subscriptO);
  lcd.createChar(1, sub1);
  lcd.createChar(2, sub2);
  lcd.createChar(3, sub3);

  // Launch the Buffer & Temp Engine on Core 0
  xTaskCreatePinnedToCore(
    sensorTaskFunction, "SensorTask", 4096, NULL, 1, &SensorTask, 0);

  // --- Startup Buzzer ---
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, 8); 
  ledcWrite(BUZZER_PIN, 127); 
  delay(1000);                    
  ledcWrite(BUZZER_PIN, 0);   

  // --- Initializing Screen ---
  lcd.setCursor(0, 0);
  lcd.print("    WELCOME"); 
  delay(1000); 
  lcd.clear();  
  
  lcd.setCursor(0, 0);
  lcd.print("Model.No:"); 
  lcd.setCursor(0, 1);
  lcd.print("SARH FYPIV-1500"); 
  delay(2000); 
  lcd.clear();          

  // --- Startup Countdown ---
  for (int i = 5; i >= 0; i--) {
    lcd.setCursor(0, 0);
    lcd.print("System Initiates");
    lcd.setCursor(0, 1);
    lcd.print("in : ");
    lcd.print(i);
    lcd.print(" sec ");
    delay(1000);          
  }
  lcd.clear();

  // --- Pre-calculate SPWM Sine Table ---
  for (int i = 0; i < num_points; i++) {
    float angle = (2.0 * PI * i) / num_points; 
    duty_A_array[i] = 50.0 + (MOD_INDEX * 50.0 * sin(angle));
    duty_B_array[i] = 50.0 - (MOD_INDEX * 50.0 * sin(angle)); 
  }

  // --- Configure SEPIC ---
  ledcAttach(SEPIC_PIN, SEPIC_FREQ, 10); 
  ledcWrite(SEPIC_PIN, 0); 

  // --- ESP-IDF v5 MCPWM HARDWARE INIT ---
  timer_period = 80000000 / (CARRIER_FREQ * 2); 

  mcpwm_timer_config_t timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = 80000000, 
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
      .period_ticks = timer_period,
  };
  mcpwm_new_timer(&timer_config, &spwm_timer);

  mcpwm_operator_config_t op_config = { .group_id = 0 };
  mcpwm_oper_handle_t oper_a, oper_b;
  mcpwm_new_operator(&op_config, &oper_a);
  mcpwm_new_operator(&op_config, &oper_b);

  mcpwm_operator_connect_timer(oper_a, spwm_timer);
  mcpwm_operator_connect_timer(oper_b, spwm_timer);

  mcpwm_comparator_config_t cmp_config = { .flags = { .update_cmp_on_tez = true } };
  mcpwm_new_comparator(oper_a, &cmp_config, &cmpr_a);
  mcpwm_new_comparator(oper_b, &cmp_config, &cmpr_b);

  mcpwm_generator_config_t gen_a_h_cfg = { .gen_gpio_num = HIN_A };
  mcpwm_generator_config_t gen_a_l_cfg = { .gen_gpio_num = LIN_A };
  mcpwm_gen_handle_t gen_a_h, gen_a_l;
  mcpwm_new_generator(oper_a, &gen_a_h_cfg, &gen_a_h);
  mcpwm_new_generator(oper_a, &gen_a_l_cfg, &gen_a_l);

  mcpwm_generator_config_t gen_b_h_cfg = { .gen_gpio_num = HIN_B };
  mcpwm_generator_config_t gen_b_l_cfg = { .gen_gpio_num = LIN_B };
  mcpwm_gen_handle_t gen_b_h, gen_b_l;
  mcpwm_new_generator(oper_b, &gen_b_h_cfg, &gen_b_h);
  mcpwm_new_generator(oper_b, &gen_b_l_cfg, &gen_b_l);

  mcpwm_generator_set_action_on_compare_event(gen_a_h, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_a, MCPWM_GEN_ACTION_LOW));
  mcpwm_generator_set_action_on_compare_event(gen_a_h, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, cmpr_a, MCPWM_GEN_ACTION_HIGH));

  mcpwm_generator_set_action_on_compare_event(gen_b_h, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_b, MCPWM_GEN_ACTION_LOW));
  mcpwm_generator_set_action_on_compare_event(gen_b_h, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, cmpr_b, MCPWM_GEN_ACTION_HIGH));

  uint32_t dt_ticks = DEADTIME_US * 80; 
  mcpwm_dead_time_config_t dt_pos = { .posedge_delay_ticks = dt_ticks };
  mcpwm_dead_time_config_t dt_neg = { .negedge_delay_ticks = dt_ticks, .flags = { .invert_output = true } };

  mcpwm_generator_set_dead_time(gen_a_h, gen_a_h, &dt_pos); 
  mcpwm_generator_set_dead_time(gen_a_h, gen_a_l, &dt_neg); 
  mcpwm_generator_set_dead_time(gen_b_h, gen_b_h, &dt_pos);
  mcpwm_generator_set_dead_time(gen_b_h, gen_b_l, &dt_neg);

  mcpwm_timer_enable(spwm_timer);
  mcpwm_timer_start_stop(spwm_timer, MCPWM_TIMER_START_NO_STOP);

  last_spwm_update = micros();
  last_pi_update = millis();
}

// ==========================================
// 6. MAIN LOOP (CORE 1)
// ==========================================
void loop() {
  unsigned long current_millis = millis();
  
  // TASK 1: SPWM High-Speed Update (100us)
  unsigned long current_micros = micros();
  if (active_fault == NONE && (current_micros - last_spwm_update >= 100)) {
    last_spwm_update += 100; 

    uint32_t cmp_val_a = (duty_A_array[step_index] * timer_period) / 100;
    uint32_t cmp_val_b = (duty_B_array[step_index] * timer_period) / 100;

    mcpwm_comparator_set_compare_value(cmpr_a, cmp_val_a);
    mcpwm_comparator_set_compare_value(cmpr_b, cmp_val_b);

    step_index++;
    if (step_index >= num_points) step_index = 0;
  }

  // TASK 2: Safety & PI Control (Every 400ms)
  if (active_fault == NONE && (current_millis - last_pi_update >= 400)) {
    last_pi_update = current_millis;

    // Detect Output Overload
    if (apparent_power > 1500) active_fault = OVERLOAD;

    // --- SEPIC PI VOLTAGE CONTROL ---
    if (startup_phase) {
      sepic_duty += 0.05; 
      if (sepic_duty >= 0.48) {
        sepic_duty = 0.48;
        startup_phase = false; 
      }
    } else {
      float error = 230.0 - v_ac_rms;
      integral_error += (error * DT);
      
      if (integral_error > 1000.0) integral_error = 1000.0;
      if (integral_error < -1000.0) integral_error = -1000.0;

      float pi_output = (PI_P * error) + (PI_I * integral_error);
      sepic_duty = 0.48 + pi_output;

      if (sepic_duty > 0.92) sepic_duty = 0.92;
      if (sepic_duty < 0.0) sepic_duty = 0.0;
    }

    ledcWrite(SEPIC_PIN, (uint32_t)(sepic_duty * 1023.0));

    // Detect Input Undervoltage
    if (sepic_duty >= 0.915 && v_ac_rms < 120 && !startup_phase) {
      active_fault = UNDERVOLTAGE;
    }
  }

  // TASK 3: Display Logic (Updates every 500ms)
  // Rotates: 2 Seconds Inverter Data -> 1 Second Temp Data
  if (active_fault == NONE && (current_millis - last_lcd_update >= 500)) {
    last_lcd_update = current_millis;

    static int display_cycle = 0; // 0, 1, 2, 3 (Inverter) -> 4, 5 (Temp)

    // Setup the static text only when transitioning to avoid screen flicker
    if (display_cycle == 0) {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("V"); lcd.write(0); lcd.print(":");
      lcd.setCursor(8, 0); lcd.print("I"); lcd.write(0); lcd.print(":");
      lcd.setCursor(0, 1); lcd.print("Power:");
    } else if (display_cycle == 4) {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("T"); lcd.write(1); lcd.print(":");
      lcd.setCursor(8, 0); lcd.print("T"); lcd.write(2); lcd.print(":");
      lcd.setCursor(0, 1); lcd.print("T"); lcd.write(3); lcd.print(":");
    }

    // Print active variables based on cycle state
    if (display_cycle < 4) {
      // --- INVERTER VIEW ---
      lcd.setCursor(3, 0);
      if(v_ac_rms < 100) lcd.print("0");
      if(v_ac_rms < 10)  lcd.print("0");
      lcd.print((int)v_ac_rms); lcd.print("V");

      lcd.setCursor(11, 0); lcd.print(i_ac_rms, 1); lcd.print("A ");
      lcd.setCursor(6, 1);  lcd.print((int)apparent_power); lcd.print("W   ");
    } else {
      // --- TEMPERATURE VIEW ---
      // Left-aligned numbers with controlled trailing spaces to clear old digits
      
      // T1 (Safe within columns 3 to 7)
      lcd.setCursor(3, 0);  
      lcd.print((int)temps[0]); lcd.write(0xDF); lcd.print("C");
      if (temps[0] < 100) lcd.print(" ");
      if (temps[0] < 10)  lcd.print(" ");
      
      // T2 (Safe within columns 11 to 15)
      lcd.setCursor(11, 0); 
      lcd.print((int)temps[1]); lcd.write(0xDF); lcd.print("C");
      if (temps[1] < 100) lcd.print(" ");
      if (temps[1] < 10)  lcd.print(" ");
      
      // T3 (Safe within columns 3 to 7 on row 2)
      lcd.setCursor(3, 1);  
      lcd.print((int)temps[2]); lcd.write(0xDF); lcd.print("C");
      if (temps[2] < 100) lcd.print(" ");
      if (temps[2] < 10)  lcd.print(" ");
    }

    display_cycle++;
    if (display_cycle > 5) display_cycle = 0; 
  }
  // TASK 4: Fault Handling Lockout
  if (active_fault != NONE) {
    ledcWrite(SEPIC_PIN, 0);                                  
    mcpwm_timer_start_stop(spwm_timer, MCPWM_TIMER_STOP_EMPTY); 
    lcd.clear();

    if (active_fault == OVERLOAD) {
      lcd.setCursor(0, 0); lcd.print(" SYSTEM OVERLOAD");
      lcd.setCursor(0, 1); lcd.print("   Restarting   ");
      
      for (int b = 0; b < 3; b++) {
        ledcWrite(BUZZER_PIN, 127); delay(500);
        ledcWrite(BUZZER_PIN, 0);   delay(500);
      }
    } 
    else if (active_fault == UNDERVOLTAGE) {
      ledcWrite(BUZZER_PIN, 127); 
      lcd.setCursor(0, 0); lcd.print("   Low Input   "); 
      lcd.setCursor(0, 1); lcd.print("   Voltages    ");
      delay(2000); 
    }
    else if (active_fault == OVERTEMP) {
      lcd.setCursor(0, 0); lcd.print("Critical TEMP!");
      lcd.setCursor(0, 1); lcd.print("Rebooting...");
      
      for (int b = 0; b < 3; b++) {
        ledcWrite(BUZZER_PIN, 127); delay(500);
        ledcWrite(BUZZER_PIN, 0);   delay(500);
      }
    }
    
    delay(1000); 
    ESP.restart(); 
  }
}

