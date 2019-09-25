#ifndef __SETUP_H__
#define __SETUP_H__

/*Сетевые регистры*/
#define	ext_sec_cnt						(buf_3x[0])
#define ext_flags_reg					(buf_3x[1])

#define ext_sample_tension1_reg_hi			(buf_3x[2])
#define ext_sample_tension1_reg_low			(buf_3x[3])
#define ext_sample_tension2_reg_hi			(buf_3x[4])
#define ext_sample_tension2_reg_low			(buf_3x[5])
#define ext_sample_tension3_reg_hi			(buf_3x[6])
#define ext_sample_tension3_reg_low			(buf_3x[7])
#define ext_remaining_cycles_reg_hi		  (buf_3x[8])
#define ext_remaining_cycles_reg_low		(buf_3x[9])
#define ext_current_cycles_reg_hi		    (buf_3x[10])
#define ext_current_cycles_reg_low		  (buf_3x[11])

#define ext_cmd_reg						            (buf_4x[0])
#define ext_adjusted_cycles_reg_hi			  (buf_4x[1])
#define ext_adjusted_cycles_reg_low			  (buf_4x[2])
#define ext_adjusted_up_tension_limit1	  (buf_4x[3])
#define ext_adjusted_low_tension_limit1 	(buf_4x[4])
#define ext_adjusted_up_tension_limit2	  (buf_4x[5])
#define ext_adjusted_low_tension_limit2	  (buf_4x[6])
#define ext_adjusted_up_tension_limit3	  (buf_4x[7])
#define ext_adjusted_low_tension_limit3 	(buf_4x[8])
#define ext_adjusted_speed_reg			      (buf_4x[9])

#endif
