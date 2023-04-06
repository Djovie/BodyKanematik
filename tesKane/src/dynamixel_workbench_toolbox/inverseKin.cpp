#include "../../include/dynamixel_workbench_toolbox/Gvar.h"

void Inverse_s(uint8_t dxl_id, float x, float y, float z) {
  const char *log;  // syncwrite 
  //a. HITUNG THETA Coxa
  //hitung theta c

  theta_c = atan2(y, x);
  theta_c = theta_c * 180 / M_PI;  //rad to deg
  x0 = sqrt(pow(y,2) + pow(x,2));

  //b. HITUNG THETA Femur
  //hitung theta f1
  x1 = x0 - c;  // pengurangan panjang x0 dan coxa
  theta_f1 = atan2(z, x1);
  theta_f1 = theta_f1 * 180 / M_PI;
  //hitung panjang a
  a = sqrt(pow(z,2) + pow(x1,2));
  //hitung f2
  theta_f2 = acos((pow(f,2) + pow(a,2) - pow(t,2)) / (2 * a * f));
  theta_f2 = theta_f2 * 180 / M_PI;
  // hitung f
  theta_f = theta_f1 + theta_f2;
  //c. HITUNG THETA Tibia
  //hitung theta t
  theta_t = acos((pow(f,2) + pow(t,2) - pow(a,2)) / (2 * f * t));
  theta_t = (theta_t * (180 / M_PI)) - 90;

  //d. Normalisasi 0 derajat servo
  if (dxl_id == R1 || dxl_id == R2) {
    // coba hitung manual
    theta_c_real = 150 - theta_c;
    theta_f_real = 150 - theta_f;
    theta_t_real = 150 + theta_t;
  }
  if (dxl_id == L1 || dxl_id == L2) {
    if (theta_c < 0) theta_c = 360 + theta_c;
    theta_c_real = 330 - theta_c;
    theta_f_real = 150 - theta_f;
    theta_t_real = 150 + theta_t;
  }
  switch (dxl_id) {
    case R1:
      //putar
      posisi_servo[0] = theta_t_real * 3.41;
      posisi_servo[1] = theta_f_real * 3.41;
      posisi_servo[2] = theta_c_real * 3.41;


      dxl_wb.syncWrite(handler_index, &posisi_servo[0], &log);
      

      R1_x = x;
      R1_y = y;
      R1_z = z;
      break;
    case R2:
      //putar
      posisi_servo[3] = theta_t_real * 3.41;
      posisi_servo[4] = theta_f_real * 3.41;
      posisi_servo[5] = theta_c_real * 3.41;


      dxl_wb.syncWrite(handler_index, &posisi_servo[0], &log);

      R2_x = x;
      R2_y = y;
      R2_z = z;
      break;
    case L1:
      //putar
      posisi_servo[6] = theta_t_real * 3.41;
      posisi_servo[7] = theta_f_real * 3.41;
      posisi_servo[8] = theta_c_real * 3.41;


      dxl_wb.syncWrite(handler_index, &posisi_servo[0], &log);

      L1_x = x;
      L1_y = y;
      L1_z = z;
      break;
    case L2:
      //putar
      posisi_servo[9] = theta_t_real * 3.41;
      posisi_servo[10] = theta_f_real * 3.41;
      posisi_servo[11] = theta_c_real * 3.41;


      dxl_wb.syncWrite(handler_index, &posisi_servo[0], &log);

      L2_x = x;
      L2_y = y;
      L2_z = z;
      break;
  }
}

void polinomial_trj(uint8_t dxl_id, float xp1, float yp1, float zp1, float xp2, float yp2, float zp2, float xp3, float yp3, float zp3, float xp4, float yp4, float zp4) {
  float A, B, C, D;  //utk perhitungan polinomial
  float px, py, pz;  //hasil polinomial
  int nmr_data = 0;  //no data array

  //hitung end point dengan polinomial

  for (float t = 0.0; t <= 1.009; t = t + iterasi) {
    //hitung polinomial
    A = pow((1 - t), 3);
    B = 3 * t * pow((1 - t),2);
    C = 3 * pow(t, 2) * (1 - t);
    D = pow(t, 3);
    px = A * xp1 + B * xp2 + C * xp3 + D * xp4;
    py = A * yp1 + B * yp2 + C * yp3 + D * yp4;
    pz = A * zp1 + B * zp2 + C * zp3 + D * zp4;

    //simpan hasil perhitungan
    switch (dxl_id) {
      case L1:
        array_px_L1[nmr_data] = px;
        array_py_L1[nmr_data] = py;
        array_pz_L1[nmr_data] = pz;
        break;
      case L2:
        array_px_L2[nmr_data] = px;
        array_py_L2[nmr_data] = py;
        array_pz_L2[nmr_data] = pz;
        break;
      case R1:
        array_px_R1[nmr_data] = px;
        array_py_R1[nmr_data] = py;
        array_pz_R1[nmr_data] = pz;
        break;
      case R2:
        array_px_R2[nmr_data] = px;
        array_py_R2[nmr_data] = py;
        array_pz_R2[nmr_data] = pz;
        break;
    }

    nmr_data++;
  }

  jlh_data = nmr_data;
}

void trj_lurus(uint8_t dxl_id, float x0, float y0, float z0, float x1, float y1, float z1) {
  float xp1, yp1, zp1;  //titik vektor1
  float xp2, yp2, zp2;  //titik vektor2
  float xp3, yp3, zp3;  //titik vektor3
  float xp4, yp4, zp4;  //titik vektor4

  //tentukan titik vektor polinomial
  xp1 = x0; yp1 = y0; zp1 = z0;  //P1
  xp2 = x0; yp2 = y0; zp2 = z0;  //P2
  xp3 = x1; yp3 = y1; zp3 = z1;  //P3
  xp4 = x1; yp4 = y1; zp4 = z1;  //P4
  polinomial_trj(dxl_id, xp1, yp1, zp1, xp2, yp2, zp2, xp3, yp3, zp3, xp4, yp4, zp4);
}

//trayektori langkah
void trj_langkah(uint8_t dxl_id, float x0, float y0, float z0, float x1, float y1, float zp) {
  float xp1, yp1, zp1;  //titik vektor1
  float xp2, yp2, zp2;  //titik vektor2
  float xp3, yp3, zp3;  //titik vektor3
  float xp4, yp4, zp4;  //titik vektor4

  float z1;
  z1 = (zp - (0.25 * z0)) / 0.75;
  //tentukan titik vektor polinomial
  xp1 = x0; yp1 = y0; zp1 = z0;  //P1
  xp2 = x0; yp2 = y0; zp2 = z1;  //P2
  xp3 = x1; yp3 = y1; zp3 = z1;  //P3
  xp4 = x1; yp4 = y1; zp4 = z0; //P4
  polinomial_trj(dxl_id, xp1, yp1, zp1, xp2, yp2, zp2, xp3, yp3, zp3, xp4, yp4, zp4);
}

//eksekusi trayektori
void trj_start(uint8_t id_kakiL1, uint8_t id_kakiL2, uint8_t id_kakiR1, uint8_t id_kakiR2,int dly_trj) {
  //Hitung hasil perhitungan tsb menggunakan IK

  for (uint8_t i = 0; i < jlh_data; i++) {
    if (id_kakiL1 == L1) {
      Inverse_s(id_kakiL1, array_px_L1[i], array_py_L1[i], array_pz_L1[i]);
    }

    if (id_kakiL2 == L2) {
      Inverse_s(id_kakiL2, array_px_L2[i], array_py_L2[i], array_pz_L2[i]);
    }

    if (id_kakiR1 == R1) {
      Inverse_s(id_kakiR1, array_px_R1[i], array_py_R1[i], array_pz_R1[i]);
    }

    if (id_kakiR2 == R2) {
      Inverse_s(id_kakiR2, array_px_R2[i], array_py_R2[i], array_pz_R2[i]);
    }

    usleep(dly_trj);
  }
}

void siap(){
  posisi_servo[13] = 230;
  posisi_servo[14] = 400;
  posisi_servo[12] = 204;
  Inverse_s(L1, -65, 45, -65); //x1,y1,zp
  Inverse_s(L2, -65, -45, -65);
  Inverse_s(R1, 65, 45, -65);
  Inverse_s(R2, 65, -45, -65);
}

/// dari tripod gait
/// biar dikit aja library nya

void tripod_gait(int sekuen, int dly_trj ) {
  //ganjil
  if (sekuen % 2 != 0) {
    //step kaki kanan
    //langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, n_R1_x, n_R1_y, titik_puncak);
    trj_langkah(L2, L2_x, L2_y, L2_z, n_L2_x, n_L2_y, titik_puncak);

    //geser kaki kiri
    trj_lurus(L1, L1_x, L1_y, L1_z, n_L1_x, n_L1_y, n_L1_z);
    trj_lurus(R2, R2_x, R2_y, R2_z, n_R2_x, n_R2_y, n_R2_z);
    trj_start(L1, L2, R1, R2,dly_trj);
  }

  //genap
  else if (sekuen % 2 == 0) {
    //step kaki kiri
    //langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, n_L1_x, n_L1_y, titik_puncak);
    trj_langkah(R2, R2_x, R2_y, R2_z, n_R2_x, n_R2_y, titik_puncak);

    //geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, n_R1_x, n_R1_y, n_R1_z);
    trj_lurus(L2, L2_x, L2_y, L2_z, n_L2_x, n_L2_y, n_L2_z);
    trj_start(L1, L2, R1, R2, dly_trj);
  }
}

void tripod_gaitxy(int sekuenx, int sekueny, int dly_trj ) {
  //ganjil
  if (sekuenx % 2 != 0 && sekueny % 2 != 0) {
    //step kaki kanan
    //langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, n_R1_x, n_R1_y, titik_puncak);
    trj_langkah(L2, L2_x, L2_y, L2_z, n_L2_x, n_L2_y, titik_puncak);

    //geser kaki kiri
    trj_lurus(L1, L1_x, L1_y, L1_z, n_L1_x, n_L1_y, n_L1_z);
    trj_lurus(R2, R2_x, R2_y, R2_z, n_R2_x, n_R2_y, n_R2_z);
    trj_start(L1, L2, R1, R2,dly_trj);
  }

  //genap
  else if (sekuenx % 2 == 0 && sekueny % 2 == 0) {
    //step kaki kiri
    //langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, n_L1_x, n_L1_y, titik_puncak);
    trj_langkah(R2, R2_x, R2_y, R2_z, n_R2_x, n_R2_y, titik_puncak);

    //geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, n_R1_x, n_R1_y, n_R1_z);
    trj_lurus(L2, L2_x, L2_y, L2_z, n_L2_x, n_L2_y, n_L2_z);
    trj_start(L1, L2, R1, R2, dly_trj);
  }
}