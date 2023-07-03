Header header

int64 utime


uint8 pos_type 			# 0:NONE, 1:SP_2D, 2:SP_3D, 3:DGPS, 4:RTK_float, 5:RTK_fix
uint8 meas_enable 		# bit0:pos, bit1:vel, bit2:roll, bit3:pitch, bit4:heading, bit5~7:unused
uint8 GNSS_mask 		# 0~8 bit for GPS, GLONASS, BDS, Galileo, QZSS, RNSS, SBAS, PSAT
uint8 ant_num
uint8 sv_num_tracked
uint8 sv_num_used
uint8 diff_age
uint8 sol_age

float64 ms          	# Milliseconds from the beginning of the GPS reference week
float64 longitude
float64 latitude
float64 height
float64 ve
float64 vn
float64 vu
float64 roll
float64 pitch
float64 heading

float64 std_longitude
float64 std_latitude
float64 std_height
float64 std_ve
float64 std_vn
float64 std_vu
float64 std_roll
float64 std_pitch
float64 std_heading

float64 undulation
float64 baseline_length

uint8 msg_cnt
