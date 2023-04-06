#ifndef Gvar_h
#define Gvar_h

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <unistd.h>
#include <DynamixelWorkbench.h>

const char *port_name = "/dev/ttyUSB0";
int baud_rate = 1000000;

DynamixelWorkbench dxl_wb;

/////////////// keperluan dynamixel /////////////////////////
//RDepan
#define DXL_ID_1 1
#define DXL_ID_2 2
#define DXL_ID_3 3
//RBelakang
#define DXL_ID_4 4
#define DXL_ID_5 5
#define DXL_ID_6 6
//LDepan
#define DXL_ID_7 7
#define DXL_ID_8 8
#define DXL_ID_9 9
//LBelakang
#define DXL_ID_10 10
#define DXL_ID_11 11
#define DXL_ID_12 12
//Putar
#define DXL_ID_13 13
//Gripper
#define DXL_ID_14 14
#define DXL_ID_15 15

////////////////////////////////////////////////////////

#define L1 0
#define L2 1
#define R1 2
#define R2 3
#define xx -1  //none

#define range(a) sizeof(a)/sizeof(a[0])

//IK
float c = 21.70; //23.45 // 21.70
float f = 50.08;
float t = 76.95; //76.95
// panjang link kaki

float x, y, z;
float theta_c, theta_f, theta_t, kalkulasi; //sudut dari analisa IK
float theta_c_real, theta_f_real, theta_t_real; //sudut hasil normalisasi servo
float theta_f1, theta_f2, a, x0, x1; //digunakan pd perhitungann
float sudut; //digunakan untuk body statis
float step_cal = 1.5; //perhitungan step gait

//KONFIGURASI TRAJECTORY
float iterasi = 0.1;        //banyak step 1/0.05=20
#define memori_data 25      //jumlah memori per koordinat per kaki
#define default_dly 10      //delay default 
uint dly_trj ;  //delay trajektori yg dipakai
float titik_puncak = -50;   //titik puncak langkah -50
float x_awal, y_awal, z_awal, z_puncak;
float x_akhir, y_akhir, z_akhir;

//variabel data titik trajectory
int jlh_data;

//BODY
float max_putar = 10; //derajat

//KONFIGURASI GERAK
//gerak langkah
float max_step = 30; //mm
//float max_stepy = 30; //mm
//float max_stepx = 30; //mm

//gerak putar
double e_theta, e_alpha, e_beta; //sudut rotasi
float step_kanan, step_kiri, step_kanan_x, step_kanan_y, step_kiri_x, step_kiri_y ; //langkah kaki
float body_step, body_step_y, body_step_x, jumlah_step, jumlah_step_y, jumlah_step_x, step_gait, step_gait_y, step_gait_x ;
float arah_2;
float arah;
int state = 0;

//Simpan Data IK
//L1
float array_px_L1[memori_data];
float array_py_L1[memori_data];
float array_pz_L1[memori_data];
//L2
float array_px_L2[memori_data];
float array_py_L2[memori_data];
float array_pz_L2[memori_data];
//R1
float array_px_R1[memori_data];
float array_py_R1[memori_data];
float array_pz_R1[memori_data];
//R2
float array_px_R2[memori_data];
float array_py_R2[memori_data];
float array_pz_R2[memori_data];

//variabel koordinat inverse kinematics (current position)
float L1_x, L1_y, L1_z; //kaki L1
float L2_x, L2_y, L2_z; //kaki L2
float R1_x, R1_y, R1_z; //kaki R1
float R2_x, R2_y, R2_z; //kaki R2

//koordinat IK sebelum (old)
float o_L1_x, o_L1_y, o_L1_z; //kaki L1
float o_L2_x, o_L2_y, o_L2_z; //kaki L2
float o_R1_x, o_R1_y, o_R1_z; //kaki R1
float o_R2_x, o_R2_y, o_R2_z; //kaki R2

//koordinat IK baru (new)
float n_L1_x, n_L1_y, n_L1_z; //kaki L1
float n_L2_x, n_L2_y, n_L2_z; //kaki L2
float n_R1_x, n_R1_y, n_R1_z; //kaki R1
float n_R2_x, n_R2_y, n_R2_z; //kaki R2

//offset bodi (mm)
//kaki L1
float offset_L1_x = -55.00;
float offset_L1_y = 55.00;
float offset_L1_z = 0;
//kaki L2
float offset_L2_x = -55.00;
float offset_L2_y = -55.00;
float offset_L2_z = 0;
//kaki R1
float offset_R1_x = 55.00;
float offset_R1_y = 55.00;
float offset_R1_z = 0;
//kaki R2
float offset_R2_x = 55.00;
float offset_R2_y = -55.00;
float offset_R2_z = 0;

//posisi bodi (current position)
float body_x = 0, body_y = 0;

//sudut bodi (current position)
float body_theta = 0, body_alpha = 0, body_beta = 0;

//sudut arah body
float body_arah = 0;

//variabel koordinat kaki thdp bodi (current position)
float body_L1_x, body_L1_y, body_L1_z; //kaki L1
float body_L2_x, body_L2_y, body_L2_z; //kaki L2
float body_R1_x, body_R1_y, body_R1_z; //kaki R1
float body_R2_x, body_R2_y, body_R2_z; //kaki R2

//variabel koordinat baru, kaki thdp bodi (new)
float n_body_L1_x, n_body_L1_y, n_body_L1_z; //kaki L1
float n_body_L2_x, n_body_L2_y, n_body_L2_z; //kaki L2
float n_body_R1_x, n_body_R1_y, n_body_R1_z; //kaki R1
float n_body_R2_x, n_body_R2_y, n_body_R2_z; //kaki R2

int constrain(int valuein, int mini, int maxi){
    if(valuein > maxi){
        valuein = maxi;
    }
    if(valuein < mini){
        valuein = mini;
    }
    
    return valuein;
}

/// @todo keperluan dynamixel 

int32_t posisi_servo[13];

uint8_t dxl_id[13] = { DXL_ID_1, DXL_ID_2, DXL_ID_3,
                       DXL_ID_4, DXL_ID_5, DXL_ID_6,
                       DXL_ID_7, DXL_ID_8, DXL_ID_9,
                       DXL_ID_10, DXL_ID_11, DXL_ID_12, DXL_ID_13};

const uint8_t handler_index = 0;

#endif
