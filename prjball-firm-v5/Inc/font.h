/* * * * * * * * * * * * * * * * * * * * * * * * * *
 * ProjectionBall Firmware						   *
 * Copyright (c) 2017  							   *
 * K.Watanabe,Crescent 							   *
 * Released under the MIT license 				   *
 * http://opensource.org/licenses/mit-license.php  *
 * 17/06/16 v1.0 Initial Release                   *
 * 												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f3xx_hal.h"


#define USE_WETHER_FONT
#define USE_STRING_FONT
#define USE_LARGE_STRING_FONT
#define USE_MARK_FONT
#define USE_Limted_MARK_FONT
#define USE_NUMBER_FONT


	/*	NUMBER	*/
#ifdef USE_NUMBER_FONT
extern const int8_t num_zero_l[38];
extern const int8_t num_zero_x[38];
extern const int8_t num_zero_y[38];

extern const int8_t num_one_l[34];
extern const int8_t num_one_x[34];
extern const int8_t num_one_y[34];

extern const int8_t num_two_l[39];
extern const int8_t num_two_x[39];
extern const int8_t num_two_y[39];

extern const int8_t num_three_l[46];
extern const int8_t num_three_x[46];
extern const int8_t num_three_y[46];

extern const int8_t num_four_l[50];
extern const int8_t num_four_x[50];
extern const int8_t num_four_y[50];

extern const int8_t num_five_l[50];
extern const int8_t num_five_x[50];
extern const int8_t num_five_y[50];

extern const int8_t num_six_l[44];
extern const int8_t num_six_x[44];
extern const int8_t num_six_y[44];

extern const int8_t num_seven_l[45];
extern const int8_t num_seven_x[45];
extern const int8_t num_seven_y[45];

extern const int8_t num_eight_l[55];
extern const int8_t num_eight_x[55];
extern const int8_t num_eight_y[55];

extern const int8_t num_nine_l[47];
extern const int8_t num_nine_x[47];
extern const int8_t num_nine_y[47];
#endif //NUMBER

	/*		MARK	*/
#ifdef USE_Limted_MARK_FONT

extern const int8_t mk_colon_l[38];
extern const int8_t mk_colon_x[38];
extern const int8_t mk_colon_y[38];

extern const int8_t mk_over_l[46];
extern const int8_t mk_over_x[46];
extern const int8_t mk_over_y[46];

extern const int8_t mk_comma_l[35];
extern const int8_t mk_comma_x[35];
extern const int8_t mk_comma_y[35];

extern const int8_t mk_period_l[34];
extern const int8_t mk_period_x[34];
extern const int8_t mk_period_y[34];

#endif//USE_Limted_MARK_FONT

#ifdef USE_MARK_FONT

extern const int8_t mk_heart_l[70];
extern const int8_t mk_heart_x[70];
extern const int8_t mk_heart_y[70];

extern const int8_t mk_exclamation_l[31];
extern const int8_t mk_exclamation_x[31];
extern const int8_t mk_exclamation_y[31];

extern const int8_t mk_hash_l[116];
extern const int8_t mk_hash_x[116];
extern const int8_t mk_hash_y[116];

extern const int8_t mk_dollars_l[80];
extern const int8_t mk_dollars_x[80];
extern const int8_t mk_dollars_y[80];

extern const int8_t mk_percent_l[89];
extern const int8_t mk_percent_x[89];
extern const int8_t mk_percent_y[89];

extern const int8_t mk_ampersand_l[53];
extern const int8_t mk_ampersand_x[53];
extern const int8_t mk_ampersand_y[53];

extern const int8_t mk_parenl_l[50];
extern const int8_t mk_parenl_x[50];
extern const int8_t mk_parenl_y[50];

extern const int8_t mk_parenr_l[55];
extern const int8_t mk_parenr_x[55];
extern const int8_t mk_parenr_y[55];

extern const int8_t mk_asterisk_l[73];
extern const int8_t mk_asterisk_x[73];
extern const int8_t mk_asterisk_y[73];

extern const int8_t mk_question_l[59];
extern const int8_t mk_question_x[59];
extern const int8_t mk_question_y[59];

extern const int8_t mk_at_l[65];
extern const int8_t mk_at_x[65];
extern const int8_t mk_at_y[65];

extern const int8_t mk_bracketl_l[54];
extern const int8_t mk_bracketl_x[54];
extern const int8_t mk_bracketl_y[54];

extern const int8_t mk_bracketr_l[54];
extern const int8_t mk_bracketr_x[54];
extern const int8_t mk_bracketr_y[54];

extern const int8_t mk_hat_l[32];
extern const int8_t mk_hat_x[32];
extern const int8_t mk_hat_y[32];

extern const int8_t mk_underscore_l[36];
extern const int8_t mk_underscore_x[36];
extern const int8_t mk_underscore_y[36];

extern const int8_t mk_verticalbar_l[44];
extern const int8_t mk_verticalbar_x[44];
extern const int8_t mk_verticalbar_y[44];

extern const int8_t mk_tilde_l[33];
extern const int8_t mk_tilde_x[33];
extern const int8_t mk_tilde_y[33];

extern const int8_t mk_singlequotation_l[32];
extern const int8_t mk_singlequotation_x[32];
extern const int8_t mk_singlequotation_y[32];

extern const int8_t mk_doublequotation_l[65];
extern const int8_t mk_doublequotation_x[65];
extern const int8_t mk_doublequotation_y[65];

extern const int8_t mk_yen_l[123];
extern const int8_t mk_yen_x[123];
extern const int8_t mk_yen_y[123];

extern const int8_t mk_plus_l[63];
extern const int8_t mk_plus_x[63];
extern const int8_t mk_plus_y[63];

extern const int8_t mk_lessthan_l[43];
extern const int8_t mk_lessthan_x[43];
extern const int8_t mk_lessthan_y[43];

extern const int8_t mk_equal_l[61];
extern const int8_t mk_equal_x[61];
extern const int8_t mk_equal_y[61];

extern const int8_t mk_graterthan_l[40];
extern const int8_t mk_graterthan_x[40];
extern const int8_t mk_graterthan_y[40];

extern const int8_t mk_dash_l[23];
extern const int8_t mk_dash_x[23];
extern const int8_t mk_dash_y[23];
#endif // USE_MARK_FONT


	/*	WATCH	*/
extern const int awatch_frame_l[94];
extern const int awatch_frame_x[94];
extern const int awatch_frame_y[94];

extern const int awatch_min_l[24];
extern const int awatch_min_x[24];
extern const int awatch_min_y[24];

extern const int awatch_hour_l[23];
extern const int awatch_hour_x[23];
extern const int awatch_hour_y[23];

	/*	STRING	*/

#ifdef USE_STRING_FONT

extern const int8_t str_space_l[10];
extern const int8_t str_space_x[10];
extern const int8_t str_space_y[10];

extern const int8_t str_a_l[46];
extern const int8_t str_a_x[46];
extern const int8_t str_a_y[46];

extern const int8_t str_b_l[59];
extern const int8_t str_b_x[59];
extern const int8_t str_b_y[59];

extern const int8_t str_c_l[35];
extern const int8_t str_c_x[35];
extern const int8_t str_c_y[35];

extern const int8_t str_d_l[57];
extern const int8_t str_d_x[57];
extern const int8_t str_d_y[57];

extern 	const int8_t str_e_l[44];
extern 	const int8_t str_e_x[44];
extern 	const int8_t str_e_y[44];

extern 	const int8_t str_f_l[60];
extern 	const int8_t str_f_x[60];
extern 	const int8_t str_f_y[60];

extern const int8_t str_g_l[52];
extern const int8_t str_g_x[52];
extern const int8_t str_g_y[52];

extern const int8_t str_h_l[58];
extern const int8_t str_h_x[58];
extern const int8_t str_h_y[58];

extern const int8_t str_i_l[50];
extern const int8_t str_i_x[50];
extern const int8_t str_i_y[50];

extern const int8_t str_j_l[53];
extern const int8_t str_j_x[53];
extern const int8_t str_j_y[53];

extern const int8_t str_k_l[77];
extern const int8_t str_k_x[77];
extern const int8_t str_k_y[77];

extern const int8_t str_l_l[37];
extern const int8_t str_l_x[37];
extern const int8_t str_l_y[37];

extern const int8_t str_m_l[71];
extern const int8_t str_m_x[71];
extern const int8_t str_m_y[71];

extern const int8_t str_n_l[52];
extern const int8_t str_n_x[52];
extern const int8_t str_n_y[52];

extern const int8_t str_o_l[38];
extern const int8_t str_o_x[38];
extern const int8_t str_o_y[38];

extern const int8_t str_p_l[65];
extern const int8_t str_p_x[65];
extern const int8_t str_p_y[65];

extern const int8_t str_q_l[54];
extern const int8_t str_q_x[54];
extern const int8_t str_q_y[54];

extern const int8_t str_r_l[43];
extern const int8_t str_r_x[43];
extern const int8_t str_r_y[43];

extern const int8_t str_s_l[38];
extern const int8_t str_s_x[38];
extern const int8_t str_s_y[38];

extern const int8_t str_t_l[59];
extern const int8_t str_t_x[59];
extern const int8_t str_t_y[59];

extern const int8_t str_u_l[60];
extern const int8_t str_u_x[60];
extern const int8_t str_u_y[60];

extern const int8_t str_v_l[45];
extern const int8_t str_v_x[45];
extern const int8_t str_v_y[45];

extern const int8_t str_w_l[66];
extern const int8_t str_w_x[66];
extern const int8_t str_w_y[66];

extern const int8_t str_x_l[71];
extern const int8_t str_x_x[71];
extern const int8_t str_x_y[71];

extern const int8_t str_y_l[78];
extern const int8_t str_y_x[78];
extern const int8_t str_y_y[78];

extern const int8_t str_z_l[45];
extern const int8_t str_z_x[45];
extern const int8_t str_z_y[45];

extern const int8_t str_space_l[10];
extern const int8_t str_space_x[10];
extern const int8_t str_space_y[10];

#endif //#ifdef USE_STRING_FONT

/*	LARGE STRING	*/
#ifdef USE_LARGE_STRING_FONT

extern const int8_t str_A_l[68];
extern const int8_t str_A_x[68];
extern const int8_t str_A_y[68];

extern const int8_t str_B_l[66];
extern const int8_t str_B_x[66];
extern const int8_t str_B_y[66];

extern const int8_t str_C_l[38];
extern const int8_t str_C_x[38];
extern const int8_t str_C_y[38];

extern const int8_t str_D_l[55];
extern const int8_t str_D_x[55];
extern const int8_t str_D_y[55];

extern const int8_t str_E_l[66];
extern const int8_t str_E_x[66];
extern const int8_t str_E_y[66];

extern const int8_t str_F_l[64];
extern const int8_t str_F_x[64];
extern const int8_t str_F_y[64];

extern const int8_t str_G_l[49];
extern const int8_t str_G_x[49];
extern const int8_t str_G_y[49];

extern const int8_t str_H_l[84];
extern const int8_t str_H_x[84];
extern const int8_t str_H_y[84];

extern const int8_t str_I_l[52];
extern const int8_t str_I_x[52];
extern const int8_t str_I_y[52];

extern const int8_t str_J_l[40];
extern const int8_t str_J_x[40];
extern const int8_t str_J_y[40];

extern const int8_t str_K_l[80];
extern const int8_t str_K_x[80];
extern const int8_t str_K_y[80];

extern const int8_t str_L_l[46];
extern const int8_t str_L_x[46];
extern const int8_t str_L_y[46];

extern const int8_t str_M_l[67];
extern const int8_t str_M_x[67];
extern const int8_t str_M_y[67];

extern const int8_t str_N_l[64];
extern const int8_t str_N_x[64];
extern const int8_t str_N_y[64];

extern const int8_t str_O_l[47];
extern const int8_t str_O_x[47];
extern const int8_t str_O_y[47];

extern const int8_t str_P_l[52];
extern const int8_t str_P_x[52];
extern const int8_t str_P_y[52];

extern const int8_t str_Q_l[74];
extern const int8_t str_Q_x[74];
extern const int8_t str_Q_y[74];

extern const int8_t str_R_l[63];
extern const int8_t str_R_x[63];
extern const int8_t str_R_y[63];

extern const int8_t str_S_l[48];
extern const int8_t str_S_x[48];
extern const int8_t str_S_y[48];

extern const int8_t str_T_l[62];
extern const int8_t str_T_x[62];
extern const int8_t str_T_y[62];

extern const int8_t str_U_l[52];
extern const int8_t str_U_x[52];
extern const int8_t str_U_y[52];

extern const int8_t str_V_l[46];
extern const int8_t str_V_x[46];
extern const int8_t str_V_y[46];

extern const int8_t str_W_l[79];
extern const int8_t str_W_x[79];
extern const int8_t str_W_y[79];

extern const int8_t str_X_l[85];
extern const int8_t str_X_x[85];
extern const int8_t str_X_y[85];

extern const int8_t str_Y_l[67];
extern const int8_t str_Y_x[67];
extern const int8_t str_Y_y[67];

extern const int8_t str_Z_l[52];
extern const int8_t str_Z_x[52];
extern const int8_t str_Z_y[52];

#endif //#ifdef USE_LARGE_STRING_FONT


	/*	WETHER	*/
#ifdef USE_WETHER_FONT

extern const int8_t mk_sun_l[197];
extern const int mk_sun_x[197];
extern const int mk_sun_y[197];

extern const int8_t mk_cloud_l[53];
extern const int8_t mk_cloud_x[53];
extern const int8_t mk_cloud_y[53];

extern const int8_t mk_rain_l[101];
extern const int mk_rain_x[101];
extern const int mk_rain_y[101];


extern const int8_t mk_snow_l[112];
extern const int mk_snow_x[112];
extern const int mk_snow_y[112];

extern const int8_t mk_thunder_l[123];
extern const int8_t mk_thunder_x[123];
extern const int8_t mk_thunder_y[123];

#endif //#ifdef USE_WETHER_FONT
