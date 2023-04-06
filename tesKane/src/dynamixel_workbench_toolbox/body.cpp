#include "inverseKin.cpp"

// simpan koordinat IK lama
void simpan_IK_lama()
{
  // simpan koordinat (old)
  o_L1_x = L1_x;
  o_L1_y = L1_y;
  o_L1_z = L1_z;
  o_L2_x = L2_x;
  o_L2_y = L2_y;
  o_L2_z = L2_z;
  o_R1_x = R1_x;
  o_R1_y = R1_y;
  o_R1_z = R1_z;
  o_R2_x = R2_x;
  o_R2_y = R2_y;
  o_R2_z = R2_z;
}

// hitung koordinat kaki terhadap pusat body
void kaki_to_body()
{
  // kaki L1
  body_L1_x = L1_x + offset_L1_x;
  body_L1_y = L1_y + offset_L1_y;
  body_L1_z = L1_z + offset_L1_z;
  // kaki L2
  body_L2_x = L2_x + offset_L2_x;
  body_L2_y = L2_y + offset_L2_y;
  body_L2_z = L2_z + offset_L2_z;
  // kaki R1
  body_R1_x = R1_x + offset_R1_x;
  body_R1_y = R1_y + offset_R1_y;
  body_R1_z = R1_z + offset_R1_z;
  // kaki R2
  body_R2_x = R2_x + offset_R2_x;
  body_R2_y = R2_y + offset_R2_y;
  body_R2_z = R2_z + offset_R2_z;
}

// hitung koordinat kaki terhadap pusat coxa (Inverse kinematics)
void kaki_to_coxa()
{
  // koordinat kaki baru
  // kaki L1
  n_L1_x = n_body_L1_x - offset_L1_x;
  n_L1_y = n_body_L1_y - offset_L1_y;
  n_L1_z = n_body_L1_z - offset_L1_z;
  // kaki L2
  n_L2_x = n_body_L2_x - offset_L2_x;
  n_L2_y = n_body_L2_y - offset_L2_y;
  n_L2_z = n_body_L2_z - offset_L2_z;
  // kaki R1
  n_R1_x = n_body_R1_x - offset_R1_x;
  n_R1_y = n_body_R1_y - offset_R1_y;
  n_R1_z = n_body_R1_z - offset_R1_z;
  // kaki R2
  n_R2_x = n_body_R2_x - offset_R2_x;
  n_R2_y = n_body_R2_y - offset_R2_y;
  n_R2_z = n_body_R2_z - offset_R2_z;
}

// gerak putar
void gerak_putar(float arah, int dly_trj)
{
  float e_theta, e_alpha, e_beta; // sudut rotasi
  float step_kanan, step_kiri;    // langkah kaki
  float body_step, jumlah_step, step_gait;
  float arah_2;

  // hitung step, banyak step
  for (int n = 1; n <= 1000; n++)
  {
    body_step = arah / n;
    if (abs(body_step) <= max_putar)
    {
      jumlah_step = n;
      break;
    }
  }

  // hitung total step gait
  step_gait = jumlah_step + 1;

  // hitung body kinematic, gait
  for (int i = 1; i <= step_gait; i++)
  {
    // tentukan step langkah
    // step awal
    if (i == 1)
    {
      step_kanan = 0.5 * body_step;
      step_kiri = -0.5 * body_step;
    }
    // step genap
    else if ((i % 2 == 0) && (i != 1) && (i != step_gait))
    {
      step_kanan = -body_step;
      step_kiri = body_step;
    }
    // step ganjil
    else if ((i % 2 != 0) && (i != 1) && (i != step_gait))
    {
      step_kanan = body_step;
      step_kiri = -body_step;
    }
    // step akhir
    else if (i == step_gait)
    {
      // ganjil
      if (i % 2 != 0)
      {
        step_kanan = 0.5 * body_step;
        step_kiri = -0.5 * body_step;
      }
      // genap
      else if (i % 2 == 0)
      {
        step_kanan = -0.5 * body_step;
        step_kiri = 0.5 * body_step;
      }
    }

    // ubah ke rad
    step_kanan = step_kanan * M_PI / 180;
    step_kiri = step_kiri * M_PI / 180;

    // rotasi Z (putar arah bodi)
    e_theta = 0;
    e_alpha = 0;
    e_beta = step_kanan;

    // hitung matriks rotasi kaki kanan
    double matriks_rotasi_kanan[3][3] =
        {
            {cos(e_alpha) * cos(e_beta), -sin(e_beta) * cos(e_alpha), sin(e_alpha)},
            {sin(e_theta) * sin(e_alpha) * cos(e_beta) + cos(e_theta) * sin(e_beta), -sin(e_theta) * sin(e_beta) * sin(e_alpha) + cos(e_theta) * cos(e_beta), -sin(e_theta) * cos(e_alpha)},
            {-(cos(e_theta) * sin(e_alpha) * cos(e_beta)) + sin(e_theta) * sin(e_beta), cos(e_theta) * sin(e_alpha) * sin(e_beta) + sin(e_theta) * cos(e_beta), cos(e_theta) * cos(e_alpha)}};

    // rotasi Z (putar arah bodi)
    e_theta = 0;
    e_alpha = 0;
    e_beta = step_kiri;

    // hitung matriks rotasi
    double matriks_rotasi_kiri[3][3] =
        {
            {cos(e_alpha) * cos(e_beta), -sin(e_beta) * cos(e_alpha), sin(e_alpha)},
            {sin(e_theta) * sin(e_alpha) * cos(e_beta) + cos(e_theta) * sin(e_beta), -sin(e_theta) * sin(e_beta) * sin(e_alpha) + cos(e_theta) * cos(e_beta), -sin(e_theta) * cos(e_alpha)},
            {-(cos(e_theta) * sin(e_alpha) * cos(e_beta)) + sin(e_theta) * sin(e_beta), cos(e_theta) * sin(e_alpha) * sin(e_beta) + sin(e_theta) * cos(e_beta), cos(e_theta) * cos(e_alpha)}};

    // posisi kordinat IK lama
    simpan_IK_lama();

    // hitung posisi relatif thdp bodi
    kaki_to_body();

    // hitung posisi setelah rotasi
    // langkah kiri
    // kaki L1
    n_body_L1_x = matriks_rotasi_kiri[0][0] * body_L1_x + matriks_rotasi_kiri[0][1] * body_L1_y + matriks_rotasi_kiri[0][2] * body_L1_z;
    n_body_L1_y = matriks_rotasi_kiri[1][0] * body_L1_x + matriks_rotasi_kiri[1][1] * body_L1_y + matriks_rotasi_kiri[1][2] * body_L1_z;
    n_body_L1_z = matriks_rotasi_kiri[2][0] * body_L1_x + matriks_rotasi_kiri[2][1] * body_L1_y + matriks_rotasi_kiri[2][2] * body_L1_z;
    // kaki R2
    n_body_R2_x = matriks_rotasi_kiri[0][0] * body_R2_x + matriks_rotasi_kiri[0][1] * body_R2_y + matriks_rotasi_kiri[0][2] * body_R2_z;
    n_body_R2_y = matriks_rotasi_kiri[1][0] * body_R2_x + matriks_rotasi_kiri[1][1] * body_R2_y + matriks_rotasi_kiri[1][2] * body_R2_z;
    n_body_R2_z = matriks_rotasi_kiri[2][0] * body_R2_x + matriks_rotasi_kiri[2][1] * body_R2_y + matriks_rotasi_kiri[2][2] * body_R2_z;

    // langkah kanan
    // kaki R1
    n_body_R1_x = matriks_rotasi_kanan[0][0] * body_R1_x + matriks_rotasi_kanan[0][1] * body_R1_y + matriks_rotasi_kanan[0][2] * body_R1_z;
    n_body_R1_y = matriks_rotasi_kanan[1][0] * body_R1_x + matriks_rotasi_kanan[1][1] * body_R1_y + matriks_rotasi_kanan[1][2] * body_R1_z;
    n_body_R1_z = matriks_rotasi_kanan[2][0] * body_R1_x + matriks_rotasi_kanan[2][1] * body_R1_y + matriks_rotasi_kanan[2][2] * body_R1_z;
    // kaki L2
    n_body_L2_x = matriks_rotasi_kanan[0][0] * body_L2_x + matriks_rotasi_kanan[0][1] * body_L2_y + matriks_rotasi_kanan[0][2] * body_L2_z;
    n_body_L2_y = matriks_rotasi_kanan[1][0] * body_L2_x + matriks_rotasi_kanan[1][1] * body_L2_y + matriks_rotasi_kanan[1][2] * body_L2_z;
    n_body_L2_z = matriks_rotasi_kanan[2][0] * body_L2_x + matriks_rotasi_kanan[2][1] * body_L2_y + matriks_rotasi_kanan[2][2] * body_L2_z;

    // kembalikan koordinat ke pusat coxa
    kaki_to_coxa();

    // gerakkan kaki
    tripod_gait(i, dly_trj);
  }
}

/// @category gerakan kaki ////////////////////////////////////////////
// gerak lurus body sumbu y
void gerak_body_y(float pos_y, int dly_trj)
{

  //    updateCMPSROLLPITCH();
  //     rotasi_body(Output, 0, 0);
  // hitung step, banyak step
  for (int n = 1; n <= 1000; n++)
  {
    body_step = pos_y / n;
    if (abs(body_step) <= max_step)
    {
      jumlah_step = n;
      break;
    }
  }

  // hitung total step gait
  step_gait = jumlah_step + 1;

  // hitung body kinematic, gait
  for (int i = 1; i <= step_gait; i++)
  {

    // tentukan step langkah
    // step awal
    if (i == 1)
    {
      step_kanan = step_cal * body_step;
      step_kiri = -step_cal * body_step;
    }
    // step genap
    else if ((i % 2 == 0) && (i != 1) && (i != step_gait))
    {
      step_kanan = -body_step;
      step_kiri = body_step;
    }
    // step ganjil
    else if ((i % 2 != 0) && (i != 1) && (i != step_gait))
    {
      step_kanan = body_step;
      step_kiri = -body_step;
    }
    // step akhir
    else if (i == step_gait)
    {
      // ganjil
      if (i % 2 != 0)
      {
        step_kanan = step_cal * body_step;
        step_kiri = -step_cal * body_step;
      }
      // genap
      else if (i % 2 == 0)
      {
        step_kanan = -step_cal * body_step;
        step_kiri = step_cal * body_step;
      }
    }

    // posisi kordinat IK lama
    simpan_IK_lama();

    // hitung posisi relatif thdp bodi
    kaki_to_body();

    // hitung koordinat selanjutnya
    // langkah kiri
    // kaki L1
    n_body_L1_x = body_L1_x;
    n_body_L1_y = body_L1_y + step_kiri;
    n_body_L1_z = body_L1_z;
    // kaki R2
    n_body_R2_x = body_R2_x;
    n_body_R2_y = body_R2_y + step_kiri;
    n_body_R2_z = body_R2_z;

    // langkah kanan
    // kaki R1
    n_body_R1_x = body_R1_x;
    n_body_R1_y = body_R1_y + step_kanan;
    n_body_R1_z = body_R1_z;
    // kaki L2
    n_body_L2_x = body_L2_x;
    n_body_L2_y = body_L2_y + step_kanan;
    n_body_L2_z = body_L2_z;

    // kembalikan koordinat ke pusat coxa
    kaki_to_coxa();

    // gerakkan kaki
    tripod_gait(i, dly_trj);
  }
}

// gerak lurus body sumbu x
void gerak_body_x(float pos_x, int dly_trj)
{

  //  rotasi_body(pitch, 0, 0);

  // hitung step, banyak step
  for (int n = 1; n <= 1000; n++)
  {
    body_step = pos_x / n;
    if (abs(body_step) <= max_step)
    {
      jumlah_step = n;
      break;
    }
  }

  // hitung total step gait
  step_gait = jumlah_step + 1;

  // hitung body kinematic, gait
  for (int i = 1; i <= step_gait; i++)
  { // kondisi awal i=1
    // tentukan step langkah
    // step awal
    if (i == 1)
    {
      step_kanan = step_cal * body_step;
      step_kiri = -step_cal * body_step;
    }
    // step genap
    else if ((i % 2 == 0) && (i != 1) && (i != step_gait))
    {
      step_kanan = -body_step;
      step_kiri = body_step;
    }
    // step ganjil
    else if ((i % 2 != 0) && (i != 1) && (i != step_gait))
    {
      step_kanan = body_step;
      step_kiri = -body_step;
    }
    // step akhir
    else if (i == step_gait)
    {
      // ganjil
      if (i % 2 != 0)
      {
        step_kanan = step_cal * body_step;
        step_kiri = -step_cal * body_step;
      }
      // genap
      else if (i % 2 == 0)
      {
        step_kanan = -step_cal * body_step;
        step_kiri = step_cal * body_step;
      }
    }

    // posisi kordinat IK lama
    simpan_IK_lama();

    // hitung posisi relatif thdp bodi
    kaki_to_body();

    // hitung koordinat selanjutnya
    // langkah kiri
    // kaki L1
    n_body_L1_x = body_L1_x + step_kiri;
    n_body_L1_y = body_L1_y;
    n_body_L1_z = body_L1_z;
    // kaki R2
    n_body_R2_x = body_R2_x + step_kiri;
    n_body_R2_y = body_R2_y;
    n_body_R2_z = body_R2_z;

    // langkah kanan
    // kaki R1
    n_body_R1_x = body_R1_x + step_kanan;
    n_body_R1_y = body_R1_y;
    n_body_R1_z = body_R1_z;
    // kaki L2
    n_body_L2_x = body_L2_x + step_kanan;
    n_body_L2_y = body_L2_y;
    n_body_L2_z = body_L2_z;

    // kembalikan koordinat ke pusat coxa
    kaki_to_coxa();

    // gerakkan kaki
    tripod_gait(i, dly_trj);
  }
}

// translasi pusat body
void translasi_body(float pos_x, float pos_y, float pos_z)
{

  // hitung posisi relatif thdp bodi
  kaki_to_body();

  // hitung koordinat selanjutnya
  // kaki L1
  n_body_L1_x = body_L1_x + pos_x;
  n_body_L1_y = body_L1_y + pos_y;
  n_body_L1_z = body_L1_z + pos_z;
  // kaki L2
  n_body_L2_x = body_L2_x + pos_x;
  n_body_L2_y = body_L2_y + pos_y;
  n_body_L2_z = body_L2_z + pos_z;
  // kaki R1
  n_body_R1_x = body_R1_x + pos_x;
  n_body_R1_y = body_R1_y + pos_y;
  n_body_R1_z = body_R1_z + pos_z;
  // kaki R2
  n_body_R2_x = body_R2_x + pos_x;
  n_body_R2_y = body_R2_y + pos_y;
  n_body_R2_z = body_R2_z + pos_z;

  // kembalikan koordinat ke pusat coxa
  kaki_to_coxa();

  // gerakkan kaki (pos sekarang ke pos baru)
  trj_lurus(L1, L1_x, L1_y, L1_z, n_L1_x, n_L1_y, n_L1_z);
  trj_lurus(L2, L2_x, L2_y, L2_z, n_L2_x, n_L2_y, n_L2_z);
  trj_lurus(R1, R1_x, R1_y, R1_z, n_R1_x, n_R1_y, n_R1_z);
  trj_lurus(R2, R2_x, R2_y, R2_z, n_R2_x, n_R2_y, n_R2_z);
  trj_start(L1, L2, R1, R2, dly_trj);
}

// rotasi terhadap pusat bodi
void rotasi_body(float theta, float alpha, float beta)
{
  float e_theta, e_alpha, e_beta; // selisih sudut (error)
  // batasi utk mencegah error mekanik
  theta = constrain(theta, -15, 15);
  alpha = constrain(alpha, -15, 15);
  beta = constrain(beta, -15, 15);

  //  theta=constrain(theta,-20,20);
  //  alpha=constrain(alpha,-20,20);
  //  beta=constrain(beta,-20,20);

  // hitung perbedaan sudut bodi (SV-PV)
  e_theta = theta - body_theta;
  e_alpha = alpha - body_alpha;
  e_beta = beta - body_beta;

  // ubah ke rad
  e_theta = e_theta * M_PI / 180; // sudut rotasi sb X
  e_alpha = e_alpha * M_PI / 180; // sudut rotasi sb Y
  e_beta = e_beta * M_PI / 180;   // sudut rotasi sb Z

  // hitung matriks rotasi
  double matriks_rotasi[3][3] =
      {
          {cos(e_alpha) * cos(e_beta), -sin(e_beta) * cos(e_alpha), sin(e_alpha)},
          {sin(e_theta) * sin(e_alpha) * cos(e_beta) + cos(e_theta) * sin(e_beta), -sin(e_theta) * sin(e_beta) * sin(e_alpha) + cos(e_theta) * cos(e_beta), -sin(e_theta) * cos(e_alpha)},
          {-(cos(e_theta) * sin(e_alpha) * cos(e_beta)) + sin(e_theta) * sin(e_beta), cos(e_theta) * sin(e_alpha) * sin(e_beta) + sin(e_theta) * cos(e_beta), cos(e_theta) * cos(e_alpha)}};

  // hitung posisi relatif thdp bodi
  kaki_to_body();

  // hitung posisi setelah rotasi
  // kaki L1
  n_body_L1_x = matriks_rotasi[0][0] * body_L1_x + matriks_rotasi[0][1] * body_L1_y + matriks_rotasi[0][2] * body_L1_z;
  n_body_L1_y = matriks_rotasi[1][0] * body_L1_x + matriks_rotasi[1][1] * body_L1_y + matriks_rotasi[1][2] * body_L1_z;
  n_body_L1_z = matriks_rotasi[2][0] * body_L1_x + matriks_rotasi[2][1] * body_L1_y + matriks_rotasi[2][2] * body_L1_z;
  // kaki L2
  n_body_L2_x = matriks_rotasi[0][0] * body_L2_x + matriks_rotasi[0][1] * body_L2_y + matriks_rotasi[0][2] * body_L2_z;
  n_body_L2_y = matriks_rotasi[1][0] * body_L2_x + matriks_rotasi[1][1] * body_L2_y + matriks_rotasi[1][2] * body_L2_z;
  n_body_L2_z = matriks_rotasi[2][0] * body_L2_x + matriks_rotasi[2][1] * body_L2_y + matriks_rotasi[2][2] * body_L2_z;
  // kaki R1
  n_body_R1_x = matriks_rotasi[0][0] * body_R1_x + matriks_rotasi[0][1] * body_R1_y + matriks_rotasi[0][2] * body_R1_z;
  n_body_R1_y = matriks_rotasi[1][0] * body_R1_x + matriks_rotasi[1][1] * body_R1_y + matriks_rotasi[1][2] * body_R1_z;
  n_body_R1_z = matriks_rotasi[2][0] * body_R1_x + matriks_rotasi[2][1] * body_R1_y + matriks_rotasi[2][2] * body_R1_z;
  // kaki R2
  n_body_R2_x = matriks_rotasi[0][0] * body_R2_x + matriks_rotasi[0][1] * body_R2_y + matriks_rotasi[0][2] * body_R2_z;
  n_body_R2_y = matriks_rotasi[1][0] * body_R2_x + matriks_rotasi[1][1] * body_R2_y + matriks_rotasi[1][2] * body_R2_z;
  n_body_R2_z = matriks_rotasi[2][0] * body_R2_x + matriks_rotasi[2][1] * body_R2_y + matriks_rotasi[2][2] * body_R2_z;

  // kembalikan koordinat ke pusat coxa, koordinat baru
  kaki_to_coxa();

  // gerakkan kaki (pos sekarang ke pos baru)
  trj_lurus(L1, L1_x, L1_y, L1_z, n_L1_x, n_L1_y, n_L1_z);
  trj_lurus(L2, L2_x, L2_y, L2_z, n_L2_x, n_L2_y, n_L2_z);
  trj_lurus(R1, R1_x, R1_y, R1_z, n_R1_x, n_R1_y, n_R1_z);
  trj_lurus(R2, R2_x, R2_y, R2_z, n_R2_x, n_R2_y, n_R2_z);
  trj_start(L1, L2, R1, R2, dly_trj);

  // simpan sudut sekarang
  body_theta = theta;
  body_alpha = alpha;
  body_beta = beta;
}

// gerak lurus body sumbu xy
void gerak_body_xy(float pos_x, float pos_y, int dly_trj)
{

  for (int m = 1; m <= 1000; m++)
  {
    body_step_x = pos_x / m;
    body_step_y = pos_y / m;
    if (abs(body_step_x) <= max_step && abs(body_step_y) <= max_step)
    {
      jumlah_step_x = m;
      jumlah_step_y = m;
      break;
    }
  }

  // hitung total step gait yx
  step_gait_y = jumlah_step_y + 1;
  step_gait_x = jumlah_step_x + 1;

  // hitung body kinematic, gait y
  for (int i = 1; i <= step_gait_y; i++)
  {

    // tentukan step langkah
    // step awal
    if (i == 1)
    {
      step_kanan_x = step_cal * body_step_x;
      step_kiri_x = -step_cal * body_step_x;
      step_kanan_y = step_cal * body_step_y;
      step_kiri_y = -step_cal * body_step_y;
    }
    // step genap
    else if ((i % 2 == 0) && (i != 1) && (i != step_gait_y))
    {
      step_kanan_y = -body_step_y;
      step_kiri_y = body_step_y;
      step_kanan_x = -body_step_x;
      step_kiri_x = body_step_x;
    }
    // step ganjil
    else if ((i % 2 != 0) && (i != 1) && (i != step_gait_y))
    {
      step_kanan_y = body_step_y;
      step_kiri_y = -body_step_y;
      step_kanan_x = body_step_x;
      step_kiri_x = -body_step_x;
    }
    // step akhir
    else if (i == step_gait_y)
    {
      // ganjil
      if (i % 2 != 0)
      {
        step_kanan_y = step_cal * body_step_y;
        step_kiri_y = -step_cal * body_step_y;
        step_kanan_x = step_cal * body_step_x;
        step_kiri_x = -step_cal * body_step_x;
      }
      // genap
      else if (i % 2 == 0)
      {
        step_kanan_y = -step_cal * body_step_y;
        step_kiri_y = step_cal * body_step_y;
        step_kanan_x = -step_cal * body_step_x;
        step_kiri_x = step_cal * body_step_x;
      }
    }

    // hitung body kinematic, gait x
    for (int j = 1; j <= step_gait_x; j++)
    {

      // tentukan step langkah
      // step awal
      if (j == 1)
      {
        step_kanan_x = step_cal * body_step_x;
        step_kiri_x = -step_cal * body_step_x;
      }
      // step genap
      else if ((j % 2 == 0) && (j != 1) && (j != step_gait_x))
      {
        step_kanan_x = -body_step_x;
        step_kiri_x = body_step_x;
      }
      // step ganjil
      else if ((j % 2 != 0) && (j != 1) && (j != step_gait_x))
      {
        step_kanan_x = body_step_x;
        step_kiri_x = -body_step_x;
      }
      // step akhir
      else if (j == step_gait_x)
      {
        // ganjil
        if (j % 2 != 0)
        {
          step_kanan_x = step_cal * body_step_x;
          step_kiri_x = -step_cal * body_step_x;
        }
        // genap
        else if (j % 2 == 0)
        {
          step_kanan_x = -step_cal * body_step_x;
          step_kiri_x = step_cal * body_step_x;
        }
      }

      // posisi kordinat IK lama
      simpan_IK_lama();

      // hitung posisi relatif thdp bodi
      kaki_to_body();

      // hitung koordinat selanjutnya
      // langkah kiri
      // kaki L1
      n_body_L1_x = body_L1_x + step_kiri_x;
      n_body_L1_y = body_L1_y + step_kiri_y;
      n_body_L1_z = body_L1_z;
      // kaki R2
      n_body_R2_x = body_R2_x + step_kiri_x;
      n_body_R2_y = body_R2_y + step_kiri_y;
      n_body_R2_z = body_R2_z;

      // langkah kanan
      // kaki R1
      n_body_R1_x = body_R1_x + step_kanan_x;
      n_body_R1_y = body_R1_y + step_kanan_y;
      n_body_R1_z = body_R1_z;
      // kaki L2
      n_body_L2_x = body_L2_x + step_kanan_x;
      n_body_L2_y = body_L2_y + step_kanan_y;
      n_body_L2_z = body_L2_z;

      // kembalikan koordinat ke pusat coxa
      kaki_to_coxa();

      // gerakkan kaki
      tripod_gait(i, dly_trj);
    }
  }
}