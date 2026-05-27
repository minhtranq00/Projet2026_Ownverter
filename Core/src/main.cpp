// FOC_UART
/*
 * Copyright (c) 2024-present LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file is an example of a Field Oriented Control for
 *         OwnTech OwnVerter board.
 *         Please check example documentation to get more details
 *         how to use this example: https://docs.owntech.org/examples/
 *
 * @author Régis Ruelland <regis.ruelland@laas.fr>
 * @author Jean Alinei <jean.alinei@laas.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */

#include "ScopeMimicry.h"
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "transform.h"
#include "trigo.h"
#include "zephyr/console/console.h"
#include <string.h>
#include "Rs485Communication.h"
 

/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task */
void loop_background_task();
void rs485_rx_callback();
/* Code to be executed in real time in the critical */
void loop_critical_task();
void application_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */
#define HALL1 PC6
#define HALL2 PC7
#define HALL3 PD2

#define MY_INVERTER_ID 4
#define MY_INVERTER_ID_L 5

// 3. Create the RS485 object
Rs485Communication rs485;

#pragma pack(push, 1)
typedef struct {
    uint8_t target_id;
    float   target_line_length;
} InverterPayload_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t  source_id;
    float msg_code;
} InverterReply_t;
#pragma pack(pop)


uint8_t rs485_tx_buffer[sizeof(InverterPayload_t)];
uint8_t rs485_rx_buffer[sizeof(InverterPayload_t)];


const int EXPECTED_BYTES = 1 + sizeof(InverterPayload_t); // 1 header + 4 byte float
uint8_t rx_buffer[EXPECTED_BYTES];
int rx_index = 0;
static bool handshake_complete = false;
static bool dma_needs_reset = false;

static const float32_t AC_CURRENT_LIMIT = 13.0;
static const float32_t DC_CURRENT_LIMIT = 13.0;

/* Used in the control algorithm */
static const float32_t MIN_DC_VOLTAGE = 30.0F;
/* Used as a threshold to start POWER mode */
static const float32_t V_HIGH_MIN = 5.0;
static const float32_t Ts = 100.0e-6F;
static const uint32_t control_task_period = (uint32_t)(Ts * 1.e6F);

/* Hall effect sensors */
static uint8_t HALL1_value;
static uint8_t HALL2_value;
static uint8_t HALL3_value;

static uint8_t angle_index;
static float32_t hall_angle;
static PllAngle pllangle = controlLibFactory.pllAngle (Ts, 10.0F, 0.04F); //(Ts, 10.0F, 0.04F);
static PllDatas pllDatas;
static float32_t angle_filtered;
static float32_t w_meas;

static bool current_step = false;
static uint32_t trigger_counter = 0;

/*
 * One sector for one index value
 * Index = H1*2^0 + H2*2^1 + H3*2^2
 */
//static int16_t sector[] = {-1, 5, 1, 0, 3, 4, 2};
static int16_t sector[] = {-1, 2, 4, 3, 0, 1, 5};
static float32_t k_angle_offset = PI/6.0F; //0.0F;

/* Power LEG measures */
static float32_t meas_data;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I1_offset;
static float32_t I2_offset;
static float32_t tmpI1_offset;
static float32_t tmpI2_offset;
static const float32_t NB_OFFSET = 2000.0;
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t V12_value;

/* DC measures */
static float32_t I_high;
static float32_t V_high;

/* Three phase system and Park DQ Frame (dqo) */
static three_phase_t Vabc;
static three_phase_t duty_abc;
static three_phase_t Iabc;
static dqo_t Vdq;
static dqo_t Idq;
static dqo_t Idq_ref;
static float32_t angle_4_control;

static float32_t pi_d_integral;
static float32_t pi_q_integral;

/* Variables used to get static value for ScopeMimicry */
static three_phase_t Iabc_ref;
static float32_t duty_a, duty_b;
static float32_t Ia_ref;
static float32_t Ib_ref;
static float32_t Va;
static float32_t Iq_meas;
static float32_t Id_meas;
static float32_t Iq_ref;
static float32_t Iq_max;
static float32_t Vd, Vq;
static float32_t HALL1_value_f;
static float32_t HALL2_value_f;
static float32_t HALL3_value_f;
static float32_t angle_index_f;
static float32_t pi_d_integral_f;
static float32_t pi_q_integral_f;

/* We only make torque control. */
static volatile float32_t manual_Iq_ref;
static float32_t manual_Iq_ref_l;
static char recieved_Iq_char[4];
static float32_t recieved_Iq_f;

/* Angle control variables */
static float32_t theta_m_ref = 0.0F;
static float32_t theta_m = 0.0F;
static float32_t omega_m = 0.0F;
static float32_t Ki_pos = -2.0F;
static float32_t Kp_pos = -0.3F;
static float32_t Kd_pos = -0.002F;
static float32_t Iq_ref_pos = 0.0F;
 
static float32_t int_pos = 0.0F;
static int anti_windup = 1;

static float32_t const capstan_radius = 1.5F; // in centimeters
static float32_t const capstan_gear = 45.0F;
static float32_t const motor_gear = 15.0F;
static float32_t const line_cm_to_motor_rad = capstan_gear / (capstan_radius * motor_gear);

static const float32_t pole_pairs = 4.0F; // Motor 8 poles -> 4 pole pairs

static float32_t angle_prev = 0.0F;
static float32_t angle_elec_unwrapped = 0.0F;

/**
 * Low Pass Filters Init
 */
static LowPassFirstOrderFilter vHigh_filter =
						controlLibFactory.lowpassfilter(Ts, 5.0e-3F);

static LowPassFirstOrderFilter w_mes_filter =
						controlLibFactory.lowpassfilter(Ts, 5.0e-3F);

static float32_t V_high_filtered;
static float32_t inverse_Vhigh;

/**
 *  PID
 */

static float32_t T_delay = 5.5e-3F; // 500 micro-seconds of delay between control and actuation
static float32_t angle_error;

static float32_t Kp = 30 * 0.035; //30 * 0.035;
static float32_t Ti = 0.002029*1; //0.002029;
static float32_t Td = 0.0F;
static float32_t N = 1.0;
static float32_t R_L = 0.065;
static float32_t L_d = 0.3e-3F;
static float32_t L_q = 0.3e-3F;
static float32_t phi = 0.12F;
/* Coefficient 0.4 comes from Va_max =  (α_max - 0.5) * Udc     */
static float32_t lower_bound = -MIN_DC_VOLTAGE * 0.4;
static float32_t upper_bound = MIN_DC_VOLTAGE * 0.4;
static Pid pi_d = controlLibFactory.pid(Ts, Kp, Ti, Td, N,
										lower_bound, upper_bound);

static Pid pi_q = controlLibFactory.pid(Ts, Kp, Ti, Td, N,
										lower_bound, upper_bound);

/* --- Disturbance Observer (ESO) Variables --- */
static float32_t theta_m_hat = 0.0F;
static float32_t omega_m_hat = 0.0F;
static float32_t f_hat = 0.0F;
static uint8_t prev_angle_index = 255; // Initialized to an impossible state


/* Observer Tuning Parameters */
static float32_t J_inertia = 0.00019F;  // [kg.m^2] MUST BE TUNED FOR YOUR MOTOR/LOAD
static float32_t obs_L1 = 0.8F;        // Angle correction gain
static float32_t obs_L2 = 10.0F;       // Velocity correction gain
static float32_t obs_L3 = 50.0F;     // Disturbance correction gain (Integration)
static float32_t obs_L3_torque = J_inertia / (pole_pairs * Ts * 10.0F); // Precompute this term for efficiency

const float32_t ALPHA_F_HAT = Ts / (0.0032F + Ts); 

const float32_t ALPHA_IQ = Ts / (0.0008F + Ts);	 // Filter coefficient for Iq measurement (cutoff ~200 Hz);     


/* Decimation is used to limit the rate of measurement plotted in ScopeMimicry*/
const static uint32_t decimation = 10;
static uint32_t counter_time;
float32_t counter_time_f;
static float32_t w_estimate;
uint8_t received_serial_char;

/* List of possible modes for the OwnTech power shield */
enum serial_interface_menu_mode
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

/* List of possible control states */
enum control_state_mode {
	OFFSET_ST = 0,
	IDLE_ST = 1,
	POWER_ST = 2,
	ERROR_ST = 3
};

enum control_state_mode control_state;
static float32_t control_state_f;

static uint16_t error_counter;
static bool pwm_enable;
uint8_t asked_mode = IDLEMODE;

/**
 *  Definition of variables and functions for plotting real time values
 *  using ScopeMimicry.
 */

const uint16_t SCOPE_SIZE = 3000; //512;
uint16_t k_app_idx;
ScopeMimicry scope(SCOPE_SIZE, 6);
static bool is_downloading;
static bool memory_print;

bool mytrigger()
{
	return (control_state == POWER_ST);
	//return (current_step == true);
	//return (I1_low_value > 1.0F);
	
/* 	if (trigger_counter > 0) {
		trigger_counter--;
		return true;
	}
	return false; */
	
}

/**
 * Disturbance Observer for Angle, Speed, and Load estimation.
 * Must run at the PWM frequency (10 kHz).
 */

static float32_t sign_iq_prev = 0.0F;
static float32_t theta_e_hat = 0.0F; // Estimated electrical angle for plotting
static float32_t omega_e_hat = 0.0F;
inline void run_disturbance_observer()
{
	float32_t sign_iq = (manual_Iq_ref > 0.05F) ? 1.0F : ((manual_Iq_ref < -0.05F) ? -1.0F : 0.0F); // Hysteresis to avoid noise around zero
	if (sign_iq != sign_iq_prev && sign_iq != 0.0F) {
		// If we detect a change in the sign of Iq reference, we can assume the motor is starting to move in the opposite direction. 
		// This can be used as a pseudo-trigger to reset the observer states for better convergence.
		f_hat = 0.0F; // Reset load estimate
		omega_e_hat *= 0.1F; // Dampen speed estimate to help it converge faster in the new direction
	}
	sign_iq_prev = sign_iq;

/* 	if (!isfinite(omega_e_hat) || !isfinite(theta_e_hat) || !isfinite(f_hat) || fabsf(omega_e_hat) > 5000.0F) {
		// Handle non-finite values (e.g., reset observer states)
		theta_e_hat = hall_angle;
		omega_e_hat = 0.0F;
		f_hat = 0.0F;
		prev_angle_index = angle_index;
		return;
	}  */

    /* ----------------------------------------------------------------
     * 1. PREDICTION STEP (Runs every 100us)
     * ----------------------------------------------------------------*/
    // Calculate Electromagnetic Torque: Te = 1.5 * P * lambda_m * Iq
    // Using phi (0.12F) as your flux linkage and the previous cycle's Idq.q
	//float32_t Iq_for_observer = (control_state == POWER_ST) ? Idq.q : 0.0F; // Use actual Iq during POWER mode, else assume 0
	
    //float32_t Iq_for_obs = (control_state == POWER_ST) ? Idq.q : 0.0F;
    //float32_t Te_elec = 1.5F * pole_pairs * phi * Iq_for_obs;
    /* Effective electrical inertia: J_e = J_inertia / pole_pairs */
    //float32_t omega_e_dot = (Te_elec - f_hat) / (J_inertia / pole_pairs);
	//float32_t omega_e_dot = pole_pairs * (Te_elec - f_hat) / J_inertia;
    
    //omega_e_hat += omega_e_dot * Ts;
    theta_e_hat += omega_e_hat * Ts;
    theta_e_hat = ot_modulo_2pi(theta_e_hat);

	omega_e_hat *= 0.999F; // Simple low-pass filtering to improve robustness of the observer (tuning parameter, can be adjusted or removed)

	//float32_t w_n = 20.0F + 0.35F * fabsf(omega_m_hat); // 2. Clamp w_n between 0.0 and 200.0 // (Note: fabsf guarantees it is >= 2.0, so we only really need to clamp the top) 

	//if (w_n > 200.0F) { w_n = 200.0F; } // 3. Update the global observer gains 

	//obs_L1 = 100.25F * w_n; 

	//obs_L2 = 2.5F * (w_n * w_n); 

    /* ----------------------------------------------------------------
     * 2. CORRECTION STEP (Runs only when Hall state changes)
     * ----------------------------------------------------------------*/
	
   if (angle_index != prev_angle_index) {
        if (prev_angle_index != 255 && angle_index >= 1 && angle_index <= 6) {
		//if (angle_index != prev_angle_index && angle_index >= 1 && angle_index <= 6) {
            float32_t e = hall_angle - theta_e_hat;
            if (e > PI)       e -= 2.0F * PI;
            else if (e < -PI) e += 2.0F * PI;

            theta_e_hat = ot_modulo_2pi(theta_e_hat + obs_L1 * e);
            omega_e_hat += obs_L2 * e;
            f_hat       -= obs_L3 * e;
        }
        prev_angle_index = angle_index;
    } 
}

void dump_scope_datas(ScopeMimicry &scope) {
	printk("begin record\n");
	scope.reset_dump();
	while (scope.get_dump_state() != finished) {
		printk("%s", scope.dump_datas());
		task.suspendBackgroundUs(200);
	}
	printk("end record\n");
}

/**
 * VHigh Filter and PID init function
 */
void init_filt_and_reg(void)
{
	pllangle.reset(0.F);
	vHigh_filter.reset(V_HIGH_MIN);
	pi_d.reset();
	pi_q.reset();
	theta_e_hat = 0.0F;
    omega_e_hat = 0.0F;
    f_hat = 0.0F;
    prev_angle_index = 255;
	error_counter = 0;
}

/**
 * @brief A period-meter function which estimate pulsation
 * for the sector variable (one integer value correspond to π/3).
 *
 * @param sector assume sector is integer in [0, 5]
 * @param time in [s]
 * @return pulsation (float)
 */
float32_t pulsation_estimator(int16_t sector, float32_t time)
{
	static float32_t w_estimate_intern = 0.0F;
	static int16_t prev_sector;
	static float32_t prev_time = 0.0F;
	int16_t delta_sector;
	float32_t sixty_degre_step_time;
	delta_sector = sector - prev_sector;
	prev_sector = sector;
	if (delta_sector == 1 || delta_sector == -5) {
		/* positive speed */
		sixty_degre_step_time = (time - prev_time);
		w_estimate_intern = (PI / 3.0) / sixty_degre_step_time;
		prev_time = time;
	}
	if (delta_sector == -1 || delta_sector == 5) {
		sixty_degre_step_time = (time - prev_time);
		w_estimate_intern = (-PI / 3.0) / sixty_degre_step_time;
		prev_time = time;
	}
	return w_estimate_intern;
}

/**
 *  Function that retrieves necessary measurements from power shield sensors.
 */
inline void retrieve_analog_datas()
{
	meas_data = shield.sensors.getLatestValue(I1_LOW);
	if (meas_data != NO_VALUE) {
		I1_low_value = meas_data + I1_offset;
	}

	meas_data = shield.sensors.getLatestValue(I2_LOW);
	if (meas_data != NO_VALUE) {
		I2_low_value = meas_data + I2_offset;
	}

	if (control_state == OFFSET_ST && counter_time < NB_OFFSET) {
		tmpI1_offset += I1_low_value;
		tmpI2_offset += I2_low_value;
	}

	meas_data = shield.sensors.getLatestValue(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_high = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(I_HIGH);
	if (meas_data != NO_VALUE) {
		/* Sign is negative because of the way hardware sensor is routed */
		I_high = -meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V1_LOW);
	if (meas_data != NO_VALUE) {
		V1_low_value = meas_data;
	}

	meas_data = shield.sensors.getLatestValue(V2_LOW);
	if (meas_data != NO_VALUE) {
		V2_low_value = meas_data;
	}

	/* Vhigh measurement gets additional filtering */
	V_high_filtered = vHigh_filter.calculateWithReturn(V_high);

	V12_value = V1_low_value - V2_low_value;
}

/**
 * Reads Hall sensors and estimate position and speed.
 */
inline void get_position_and_speed()
{
	/* We get individual HALL sensor readings */
	HALL1_value = spin.gpio.readPin(HALL1);
	HALL2_value = spin.gpio.readPin(HALL2);
	HALL3_value = spin.gpio.readPin(HALL3);
	/* We compute angle index using HALL values. */
	angle_index = HALL1_value * 1 + HALL2_value * 2 + HALL3_value * 4;

	hall_angle =
			ot_modulo_2pi(PI / 3.0 * sector[angle_index] +
			PI * k_angle_offset / 24.0);

	w_estimate = pulsation_estimator(sector[angle_index], counter_time * Ts);
	pllDatas = pllangle.calculateWithReturn(hall_angle);

	angle_filtered = pllDatas.angle;

	/* Unwrap the electrical angle */
	float32_t delta = theta_e_hat - angle_prev;

	if (delta > PI) {
		angle_elec_unwrapped -= 2.0F * PI;
	} else if (delta < -PI) {
		angle_elec_unwrapped += 2.0F * PI;
	}
	angle_elec_unwrapped += delta;
	angle_prev = theta_e_hat; //angle_filtered;

	theta_m = angle_elec_unwrapped / pole_pairs;
	omega_m = omega_e_hat / pole_pairs; // omega_e_hat / pole_pairs; // omega_m * 0.95F + (omega_e_hat / pole_pairs) * 0.05F;

	w_meas = w_mes_filter.calculateWithReturn(pllDatas.w);
}

/**
 * Handles current limits and switch to Error state if limits exceeded.
 */
inline void overcurrent_mngt()
{
	if (I1_low_value > AC_CURRENT_LIMIT || I1_low_value < -AC_CURRENT_LIMIT ||
	    I2_low_value > AC_CURRENT_LIMIT || I2_low_value < -AC_CURRENT_LIMIT ||
	    I_high > DC_CURRENT_LIMIT) {
		error_counter++;
	}
	if (error_counter > 2) {
		control_state = ERROR_ST;
	}
}

/**
 * Stops PWM and reset filter and PID states
 */
inline void stop_pwm_and_reset_states_ifnot()
{
	if (pwm_enable == true) {
		shield.power.stop(ALL);
		/* Reset filters and pid */
		init_filt_and_reg();
		pwm_enable = false;
	}
}

/**
 * Performs Torque control using Field Oriented Control algorithm
 */
inline void control_torque()
{
	angle_4_control = theta_e_hat; //angle_filtered; // hall_angle
	//q_angle = atan2(q_beta, q_alpha);
	//angle_4_control = ot_modulo_2pi(angle_filtered + w_meas * T_delay); //angle_filtered; // hall_angle
	//Iq_ref_pos = -5.2 * (angle_4_control - theta_m_ref) - 2.3* w_meas; 
	//float32_t omega_m_dump = w_meas / pole_pairs;
	//int_pos += Ts * (theta_m - theta_m_ref) * anti_windup;
	//Iq_ref_pos = Ki_pos * int_pos + Kp_pos * theta_m + Kd_pos * omega_m;
/*	if (Iq_ref_pos > 3.0F) Iq_ref_pos = 3.0F;
	else if (Iq_ref_pos < -3.0F) Iq_ref_pos = -3.0F; */
	Idq_ref.q = Iq_ref_pos;
	//Idq_ref.q = manual_Iq_ref;
	
	
	angle_error = hall_angle - theta_e_hat;
	if (angle_error > PI) {
		angle_error -= 2.0F * PI;
	} else if (angle_error < -PI) {
		angle_error += 2.0F * PI;
	}

	/* Saturation */
	if (Idq_ref.q > Iq_max) {
		anti_windup = 0;
		Idq_ref.q = Iq_max;
	}
	else if (Idq_ref.q < -Iq_max) {
		anti_windup = 0;
		Idq_ref.q = -Iq_max;
	}
	else {
		anti_windup = 1;
	}

	Idq_ref.d = 0.0F;
	Iabc.a = I1_low_value;
	Iabc.b = I2_low_value;
	Iabc.c = -(Iabc.a + Iabc.b);

	Idq = Transform::to_dqo(Iabc, angle_4_control);
	Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d); //- w_meas * L_q * Idq.q;
	//Vdq.d = 0.0F;
	Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q); // + w_meas * (L_d * Idq.d + phi);
	//Vdq.q = theta_m_ref;
	Vdq.o = 0.0F;

	/*
	float32_t Vmax = 0.45F * V_high_filtered;
	float32_t Vdq_modulo = sqrt(Vdq.d * Vdq.d + Vdq.q * Vdq.q);

	if (Vdq_modulo > Vmax) {
		float32_t scaling = Vmax / Vdq_modulo;
		Vdq.d *= scaling;
		Vdq.q *= scaling;
	}
	*/

	pi_d_integral = pi_d.getIntegral();
	pi_q_integral = pi_q.getIntegral();

	Vabc = Transform::to_threephase(Vdq, angle_4_control);
}

/**
 * Helper function that computes duty cycles from ABC frame.
 */
inline void compute_duties()
{
	inverse_Vhigh = 1.0 / V_high_filtered; //V_high_filtered; // MIN_DC_VOLTAGE
	duty_abc.a = (Vabc.a * inverse_Vhigh + 0.5);
	duty_abc.b = (Vabc.b * inverse_Vhigh + 0.5);
	duty_abc.c = (Vabc.c * inverse_Vhigh + 0.5);
}

/**
 * Helper function that set the duty cycles to the power shield.
 */
inline void apply_duties()
{
	shield.power.setDutyCycle(LEG1, duty_abc.a);
	shield.power.setDutyCycle(LEG2, duty_abc.b);
	shield.power.setDutyCycle(LEG3, duty_abc.c);
}

/**
 * Helper function to start the power shield PWMs
 */
void start_pwms_ifnot()
{
	if (!pwm_enable) {
		pwm_enable = true;
		shield.power.start(ALL);
	}
}

/**
 * Setter function for required variables
 */
void init_variables()
{
	/* Time counter */
	counter_time = 0;
	/* Measurements variables */
	I1_low_value = 0.0F;
	I2_low_value = 0.0F;
	I_high = 0.0F;
	V_high = 0.0F;
	/* Offset variables */
	I1_offset = 0.0F;
	I2_offset = 0.0F;
	tmpI1_offset = 0.0F;
	tmpI2_offset = 0.0F;
	/* State view of the pwm */
	pwm_enable = false;
	/* Idle or power mode*/
	asked_mode = IDLEMODE;
	/* We begin to measure the current offset before all */
	control_state = OFFSET_ST;
	Iq_max = 5.0;
	manual_Iq_ref = 0.0F;
	manual_Iq_ref_l = 0.0F;

}

void rs485_rx_callback(void)
{
    spin.led.toggle();

	handshake_complete = true;
    InverterPayload_t* payload = (InverterPayload_t*)rs485_rx_buffer;
    
    if (payload->target_id == MY_INVERTER_ID) {
        // Update the reference based on the incoming command
        theta_m_ref = (payload->target_line_length) * line_cm_to_motor_rad;

        /* --- Send ACK 123 back to the control station --- */
        //InverterReply_t ack_reply = {4,123.0F};
   
        //memset(rs485_tx_buffer, 0, sizeof(rs485_tx_buffer));
        //memcpy(rs485_tx_buffer, &ack_reply, sizeof(InverterReply_t));
        
        // Transmit the buffer
        //rs485.startTransmission();
    }
	 if (payload->target_id == MY_INVERTER_ID_L) {
        // Update the reference based on the incoming command
        manual_Iq_ref_l = payload->target_line_length;

        /* --- Send ACK 123 back to the control station --- */
        //InverterReply_t ack_reply = {4,123.0F};
   
        //memset(rs485_tx_buffer, 0, sizeof(rs485_tx_buffer));
        //memcpy(rs485_tx_buffer, &ack_reply, sizeof(InverterReply_t));
        
        // Transmit the buffer
        //rs485.startTransmission();
    }
}
/* --------------SETUP FUNCTIONS------------------------------- */

/**
 * In this setup routine :
 *  - Power shield is initialized
 * 		- Shield is set in Buck Mode.
 * 		- Default sensors are activated
 * 		- GPIOs are set for Hall effect sensor
 *  - ScopeMimicry is initialized
 * 	- VHigh filter and PIDs are initialized
 * 	- LED is turned on.
 *  - Tasks are initialized and started
 */


void setup_routine()
{
	
	/* Setup the hardware first */
	shield.power.initBuck(ALL);
	shield.sensors.enableDefaultOwnverterSensors();

	//uart1.usart1SwapRxTx();
	//uart1.usart1Init();

	rs485.configureCustom(rs485_tx_buffer, 
                          rs485_rx_buffer, 
                          5, 
                          rs485_rx_callback, 
                          115200,  
                          false);  

	rs485.turnOnCommunication();

	/* --- Send Startup Code 111 --- */
    InverterReply_t startup_reply = {4,111.0F};
	memset(rs485_tx_buffer, 0, sizeof(rs485_tx_buffer));
    memcpy(rs485_tx_buffer, &startup_reply, sizeof(InverterReply_t));
    
    // Transmit the startup code
    rs485.startTransmission();

	spin.gpio.configurePin(HALL1, INPUT);
	spin.gpio.configurePin(HALL2, INPUT);
	spin.gpio.configurePin(HALL3, INPUT);

	/* Scope configuration */
	//scope.connectChannel(V12_value, "V12_value");           /* 0 */
	//scope.connectChannel(Vq, "Vq");                         /* 1 */
	//scope.connectChannel(Vd, "Vd");                         /* 2 */
	//scope.connectChannel(I1_low_value, "I1_low_value");     /* 3 */
	//scope.connectChannel(I2_low_value, "I2_low_value");     /* 4 */
	//scope.connectChannel(I_high, "I_high_value");     	    /* 5 */
	scope.connectChannel(Iq_meas, "Iq_meas");               /* 6 */
	scope.connectChannel(Iq_ref, "Iq_ref");                 /* 7 */
	//scope.connectChannel(Id_meas, "Id_meas");             /* 8 */
	//scope.connectChannel(angle_filtered, "angle_filtered"); /* 9 */
	scope.connectChannel(theta_e_hat, "theta_e_hat");               /* 9 */
	//scope.connectChannel(Ib_ref, "Ib_ref");                 /* 10 */
	scope.connectChannel(hall_angle, "hall_angle");         /* 11 */
	//scope.connectChannel(Ia_ref, "Ia_ref");                 /* 12 */
	scope.connectChannel(control_state_f, "control_state"); /* 13 */
	scope.connectChannel(angle_error, "angle_error");     /* 14 */
	//scope.connectChannel(pi_d_integral_f, "pi_d_integral");     /* 14 */
	//scope.connectChannel(pi_q_integral_f, "pi_q_integral");     /* 15 */
	//scope.connectChannel(duty_a, "duty_a");                 /* 16 */
	//scope.connectChannel(duty_b, "duty_b");                 /* 17 */
	scope.set_trigger(&mytrigger);
	scope.set_delay(0.0);
	scope.start();

	/* Initialize values */
	init_filt_and_reg();
	init_variables();
	spin.led.turnOn();

	/* Declare tasks */
	uint32_t background_task_number =
					task.createBackground(loop_background_task);

	uint32_t app_task_number = task.createBackground(application_task);
	task.createCritical(loop_critical_task, control_task_period);

	/* Finally, start tasks */
	task.startBackground(background_task_number);
	task.startBackground(app_task_number);
	task.startCritical();
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * This background task retrieve user inputs to control the OwnVerter:
 * - P and I keys respectively Power ON and Power OFF the inverter
 * - U and D keys respectively Increase and Decrease Iq reference.
 * - R Q and M keys are used to control ScopeMimicry data retrieval.
 */

void loop_background_task()
{
	
	/* Task content */
 
	received_serial_char = console_getchar();
	switch (received_serial_char) {
	case 'p':
	{
		//uart1.usart1WriteChar('c'); // Echo back the received character
		//current_step = true;
		printk("power asked");
		//trigger_counter = 100; 
		manual_Iq_ref += 0.0F;
		//received_serial_char = uart1.usart1ReadChar();
		
		//theta_m_ref = theta_m;
		//Iq_ref_pos = 0.0F;
		asked_mode = POWERMODE;
		scope.start();
		break;
	}
	case 'i':
		printk("idle asked");
		asked_mode = IDLEMODE;
		break;
	case 'r':
		is_downloading = true;
		break;
	case 'u':
	{
		theta_m_ref += 0.5F;
		//manual_Iq_ref += 1.0F;
		break;
		}
	case 'd':
		theta_m_ref -= 0.5F;
		//manual_Iq_ref = 0.0F;
		break;
	case 's':
		//theta_m_ref = 0.0F;
		//manual_Iq_ref = 2.0F;
		Iq_ref_pos = 2.0F;

		break;
	case 'm':
		/* To print scope datas in ownplot as soon as possible */
		memory_print = !memory_print;
		break;
	case 'q':
		/* Relaunch scope acquisition */
		//trigger_counter = SCOPE_SIZE;
		scope.start();
		break;
	}
}

/**
 * This application task sends data over USB Serial.
 */
void application_task()
{
	if (!memory_print) {
		printk("%7.2f:", V_high);
		//printk("%7.2f:", k_angle_offset);
		printk("%7.2f:", Iq_max);
		//printk("%7.2f:", manual_Iq_ref);
		//printk("%7.2f:", I1_offset);
		printk("%7.2f:", theta_m);
		printk("%7.2f:", theta_m_ref);
		printk("%7.2f:", Iq_ref_pos);
		//printk("%7.2f:", recieved_Iq_f);
		//printk("%7.2f:", I_high);
		//printk("%7.2f:", I1_low_value);
		//printk("%7.2f:", I2_low_value);
		printk("%7.2f:", Idq.q);
		//printk("%7.2f:", pi_d_integral);
		//printk("%7.2f:", pi_q_integral);
		printk("%7d:", control_state);
		printk("%7d\n", angle_index);

	} else {
		/* If memory_print is true then we plot scope datas in an infinite loop
		 * This can be used with ownplot if you have not python script installed
		 * to plot downloaded data using dump_scope_datas().
		 */
		k_app_idx = (k_app_idx + 1) % SCOPE_SIZE;
		printk("%.2f:", scope.get_channel_value(k_app_idx, 0));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 1));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 2));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 3));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 4));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 5));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 6));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 7));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 8));
		printk("%.2f:", scope.get_channel_value(k_app_idx, 9));
		printk("\n");
	}

	if (is_downloading) {
		dump_scope_datas(scope);
		is_downloading = false;
	}

	if (!handshake_complete) {
    InverterReply_t startup_reply = {4,111.0F};
	memset(rs485_tx_buffer, 0, sizeof(rs485_tx_buffer));
    memcpy(rs485_tx_buffer, &startup_reply, sizeof(InverterReply_t));
        
    rs485.startTransmission();
	}
	switch (control_state) {
	case OFFSET_ST:
		if (counter_time > (uint32_t)NB_OFFSET) {
			spin.led.turnOff();
			I1_offset = -tmpI1_offset / NB_OFFSET;
			I2_offset = -tmpI2_offset / NB_OFFSET;
			control_state = IDLE_ST;
		}
		break;

	case IDLE_ST:
		if ((asked_mode == POWERMODE) && (V_high_filtered > V_HIGH_MIN)) {
			theta_e_hat = hall_angle;       
        	omega_e_hat = 0.0F; //0.0F;
        	f_hat = 0.0F;
			prev_angle_index = 255;

			angle_prev = hall_angle;
			angle_elec_unwrapped = (float32_t)hall_angle;
			theta_m = angle_elec_unwrapped / pole_pairs;
			omega_m = 0.0F;

			theta_m_ref = theta_m; // Set position reference to current position to avoid jumps at startup

			control_state = POWER_ST;
		}
		break;

	case POWER_ST:
		if (asked_mode == IDLEMODE) {
			control_state = IDLE_ST;
		}
		break;

	case ERROR_ST:
		if (asked_mode == IDLEMODE) {
			error_counter = 0;
			control_state = IDLE_ST;
		}
		break;
	}

	task.suspendBackgroundMs(250);
}


/**
 * This is the critical task that runs at 10kHz.
 * It performs the Field Oriented Control.
 */
void loop_critical_task()
{
	counter_time++;

	retrieve_analog_datas();

	get_position_and_speed();

	//run_S1_observer();

	//run_disturbance_observer();

	overcurrent_mngt();

	switch (control_state) {
	case OFFSET_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case IDLE_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case ERROR_ST:
		stop_pwm_and_reset_states_ifnot();
		break;
	case POWER_ST:
		/* Control loop is executed here */

		control_torque();
		run_disturbance_observer();
		compute_duties();
		apply_duties();
		start_pwms_ifnot();
		break;
	}

	/* Decimation is used to reduce rate of plotting in ScopeMimicry */
	if (counter_time % decimation == 0) {
		angle_index_f = angle_index;
		Va = Vabc.a;
		duty_a = duty_abc.a;
		duty_b = duty_abc.b;
		Iq_ref = Idq_ref.q;
		Iq_meas = Idq.q;
		Id_meas = Idq.d;
		Vd = Vdq.d;
		Vq = Vdq.q;
		Iabc_ref = Transform::to_threephase(Idq_ref, angle_4_control);
		Ia_ref = Iabc_ref.a;
		Ib_ref = Iabc_ref.b;
		counter_time_f = (float32_t)counter_time;
		HALL1_value_f = HALL1_value;
		HALL2_value_f = HALL2_value;
		HALL3_value_f = HALL3_value;
		control_state_f = control_state;
		pi_d_integral_f = pi_d_integral;
		pi_q_integral_f = pi_q_integral;
		scope.acquire();
	}
}

int main(void)
{
	setup_routine();

	return 0;
}

