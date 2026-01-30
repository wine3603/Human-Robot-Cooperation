#include<baichuan_arm_dynamics_models/elfin/elfin10l_pose1_dynamics_model.h>


namespace baichuan {

namespace arm {

namespace elfin{

void Elfin10lPose1DynamicsModel::inertiaMatrixB( double* b_out, const double* parms, const double* q )
{
  double x0 = cos(q[1]);
  double x1 = sin(q[1]);
  double x2 = x1*(-0.48*((x0)*(x0)) - 0.48*((x1)*(x1)));
  double x3 = sin(q[2]);
  double x4 = -x3;
  double x5 = cos(q[2]);
  double x6 = x0*x4 + x1*x5;
  double x7 = cos(q[3]);
  double x8 = x6*x7;
  double x9 = x0*x5 + x1*x3;
  double x10 = -x9;
  double x11 = sin(q[3]);
  double x12 = x11*x6;
  double x13 = -x12;
  double x14 = x11*x2 - 0.52*x12;
  double x15 = cos(q[4]);
  double x16 = sin(q[4]);
  double x17 = x10*x16 + x15*x8;
  double x18 = -x15;
  double x19 = x10*x18 + x16*x8;
  double x20 = x14*x16;
  double x21 = cos(q[5]);
  double x22 = sin(q[5]);
  double x23 = x13*x22;
  double x24 = x17*x21;
  double x25 = x23 + x24;
  double x26 = x13*x21;
  double x27 = x17*x22;
  double x28 = x26 - x27;
  double x29 = x2*x7 - 0.52*x8;
  double x30 = x14*x15;
  double x31 = -x22;
  double x32 = x21*x29 - 0.2*x23 - 0.2*x24 + x30*x31;
  double x33 = -parms[73];
  double x34 = parms[65]*x25 + parms[66]*x28 + parms[67]*x19 + parms[72]*x20 + x32*x33;
  double x35 = -parms[60];
  double x36 = x21*x30 + x22*x29 + 0.2*x26 - 0.2*x27;
  double x37 = -parms[71];
  double x38 = parms[66]*x25 + parms[68]*x28 + parms[69]*x19 + parms[73]*x36 + x20*x37;
  double x39 = -0.2*x22;
  double x40 = -parms[72];
  double x41 = parms[73]*x28 + parms[74]*x36 + x19*x40;
  double x42 = parms[71]*x19 + parms[74]*x32 + x25*x33;
  double x43 = x21*x42;
  double x44 = parms[52]*x17 + parms[53]*x13 + parms[54]*x19 + parms[59]*x20 + x21*x34 + x29*x35 + x31*x38 + x39*x41 - 0.2*x43;
  double x45 = -parms[45];
  double x46 = -parms[59];
  double x47 = parms[67]*x25 + parms[69]*x28 + parms[70]*x19 + parms[71]*x32 + x36*x40;
  double x48 = parms[54]*x17 + parms[56]*x13 + parms[57]*x19 + parms[58]*x29 + x30*x46 + x47;
  double x49 = -parms[40]*x8 - parms[42]*x10 - parms[43]*x13 - parms[47]*x14 - x16*x44 - x18*x48 - x29*x45;
  double x50 = parms[28]*x6 + parms[31]*x9 + parms[32]*x2 + x49;
  double x51 = parms[39]*x8 + parms[40]*x10 + parms[41]*x13 + parms[46]*x29 + x15*x44 + x16*x48;
  double x52 = -parms[34];
  double x53 = -parms[58];
  double x54 = x21*x41;
  double x55 = parms[53]*x17 + parms[55]*x13 + parms[56]*x19 + parms[60]*x30 + x20*x53 + x21*x38 + x22*x34 + x39*x42 + 0.2*x54;
  double x56 = -parms[46];
  double x57 = parms[41]*x8 + parms[43]*x10 + parms[44]*x13 + x14*x56 + x55;
  double x58 = -x11;
  double x59 = parms[60]*x13 + parms[61]*x30 + x19*x46 + x31*x42 + x54;
  double x60 = parms[59]*x17 + parms[61]*x20 + parms[72]*x25 + parms[74]*x20 + x13*x53 + x28*x37;
  double x61 = parms[47]*x10 + parms[48]*x14 + x13*x56 + x15*x59 + x16*x60;
  double x62 = -0.52*x11;
  double x63 = parms[46]*x8 + parms[48]*x29 + parms[58]*x19 + parms[61]*x29 + x10*x45 + x17*x35 + x22*x41 + x43;
  double x64 = x63*x7;
  double x65 = parms[26]*x6 + parms[28]*x9 + x2*x52 + x51*x7 + x57*x58 + x61*x62 - 0.52*x64;
  double x66 = -parms[19];
  double x67 = x61*x7;
  double x68 = parms[27]*x6 + parms[30]*x9 + x11*x51 + x57*x7 + x62*x63 + 0.52*x67;
  double x69 = -parms[47];
  double x70 = -parms[15]*x0 - parms[17]*x1 + 0.48*parms[21]*x0 - 0.48*x3*(parms[33]*x6 - parms[45]*x13 - x16*x59 - x18*x60 - x69*x8) - 0.48*x5*(parms[33]*x10 + x58*x63 + x67) - x68;
  double x71 = -0.48*x5;
  double x72 = -x7;
  double x73 = 0.52*x11 + x58*x71;
  double x74 = x15*x58;
  double x75 = x16*x58;
  double x76 = x7*x71 - 0.52*x7;
  double x77 = -0.48*x3;
  double x78 = -x77;
  double x79 = x16*x76 + x18*x78;
  double x80 = x21*x74;
  double x81 = x22*x72 + x80;
  double x82 = x21*x72;
  double x83 = x31*x74 + x82;
  double x84 = x15*x76 + x16*x78;
  double x85 = x21*x73 + x31*x84 + x39*x72 - 0.2*x80;
  double x86 = parms[65]*x81 + parms[66]*x83 + parms[67]*x75 + parms[72]*x79 + x33*x85;
  double x87 = x21*x84 + x22*x73 + x39*x74 + 0.2*x82;
  double x88 = parms[66]*x81 + parms[68]*x83 + parms[69]*x75 + parms[73]*x87 + x37*x79;
  double x89 = parms[73]*x83 + parms[74]*x87 + x40*x75;
  double x90 = parms[71]*x75 + parms[74]*x85 + x33*x81;
  double x91 = x21*x90;
  double x92 = parms[52]*x74 + parms[53]*x72 + parms[54]*x75 + parms[59]*x79 + x21*x86 + x31*x88 + x35*x73 + x39*x89 - 0.2*x91;
  double x93 = parms[67]*x81 + parms[69]*x83 + parms[70]*x75 + parms[71]*x85 + x40*x87;
  double x94 = parms[54]*x74 + parms[56]*x72 + parms[57]*x75 + parms[58]*x73 + x46*x84 + x93;
  double x95 = x21*x89;
  double x96 = parms[53]*x74 + parms[55]*x72 + parms[56]*x75 + parms[60]*x84 + x21*x88 + x22*x86 + x39*x90 + x53*x79 + 0.2*x95;
  double x97 = parms[60]*x72 + parms[61]*x84 + x31*x90 + x46*x75 + x95;
  double x98 = parms[59]*x74 + parms[61]*x79 + parms[72]*x81 + parms[74]*x79 + x37*x83 + x53*x72;
  double x99 = x7*(parms[48]*x76 + x15*x97 + x16*x98 + x56*x72);
  double x100 = parms[46]*x58 + parms[48]*x73 + parms[58]*x75 + parms[61]*x73 + x22*x89 + x35*x74 + x91;
  double x101 = -parms[29] + parms[32]*x78 + parms[34]*x71 + x100*x62 + x11*(parms[39]*x58 + parms[41]*x72 + parms[46]*x73 + x15*x92 + x16*x94 + x69*x78) + x7*(parms[41]*x58 + parms[44]*x72 + parms[45]*x78 + x56*x76 + x96) + 0.52*x99;
  double x102 = -parms[40]*x58 - parms[43]*x72 - parms[47]*x76 - x16*x92 - x18*x94 - x45*x73;
  double x103 = x11*x15;
  double x104 = x11*x16;
  double x105 = 0.52*x7;
  double x106 = x105*x16;
  double x107 = x103*x21;
  double x108 = x107 + x22*x7;
  double x109 = x21*x7;
  double x110 = x103*x31 + x109;
  double x111 = x105*x15;
  double x112 = -0.2*x107 + x111*x31 + x21*x62 + x39*x7;
  double x113 = parms[65]*x108 + parms[66]*x110 + parms[67]*x104 + parms[72]*x106 + x112*x33;
  double x114 = x103*x39 + 0.2*x109 + x111*x21 + x22*x62;
  double x115 = parms[66]*x108 + parms[68]*x110 + parms[69]*x104 + parms[73]*x114 + x106*x37;
  double x116 = parms[73]*x110 + parms[74]*x114 + x104*x40;
  double x117 = parms[71]*x104 + parms[74]*x112 + x108*x33;
  double x118 = x117*x21;
  double x119 = parms[52]*x103 + parms[53]*x7 + parms[54]*x104 + parms[59]*x106 + x113*x21 + x115*x31 + x116*x39 - 0.2*x118 + x35*x62;
  double x120 = parms[67]*x108 + parms[69]*x110 + parms[70]*x104 + parms[71]*x112 + x114*x40;
  double x121 = parms[54]*x103 + parms[56]*x7 + parms[57]*x104 + parms[58]*x62 + x111*x46 + x120;
  double x122 = x116*x21;
  double x123 = parms[53]*x103 + parms[55]*x7 + parms[56]*x104 + parms[60]*x111 + x106*x53 + x113*x22 + x115*x21 + x117*x39 + 0.2*x122;
  double x124 = -parms[40]*x11 - parms[43]*x7 - parms[47]*x105 - x119*x16 - x121*x18 - x45*x62;
  double x125 = -x16;
  double x126 = x125*x21;
  double x127 = x125*x31;
  double x128 = -0.2*x126;
  double x129 = parms[65]*x126 + parms[66]*x127 + parms[67]*x15 + x128*x33;
  double x130 = x125*x39;
  double x131 = parms[66]*x126 + parms[68]*x127 + parms[69]*x15 + parms[73]*x130;
  double x132 = parms[73]*x127 + parms[74]*x130 + x15*x40;
  double x133 = parms[71]*x15 + parms[74]*x128 + x126*x33;
  double x134 = parms[67]*x126 + parms[69]*x127 + parms[70]*x15 + parms[71]*x128 + x130*x40;
  double x135 = parms[53]*x125 + parms[56]*x15 + x129*x22 + x131*x21 + 0.2*x132*x21 + x133*x39;
  double x136 = 0.2*x21;
  double x137 = parms[67]*x22 + parms[69]*x21 + parms[71]*x39 + x136*x40;
//
  b_out[0] = parms[10] + parms[3] + x0*(parms[13]*x0 + parms[14]*x1 + parms[20]*x2 + x4*x65 + x5*x50) + x1*(parms[14]*x0 + parms[16]*x1 + x2*x66 + x3*x50 + x5*x65) + x2*(parms[20]*x0 + parms[22]*x2 + parms[32]*x9 + parms[35]*x2 + x1*x66 + x11*x61 + x52*x6 + x64);
  b_out[1] = x70;
  b_out[2] = x68;
  b_out[3] = x49;
  b_out[4] = x55;
  b_out[5] = x47;
  b_out[6] = x70;
  b_out[7] = parms[18] + 0.48*parms[19] + 0.2304*parms[22] + parms[23] - x101 - 0.48*x3*(parms[32] + parms[35]*x77 - parms[45]*x72 - parms[48]*x78 - x16*x97 - x18*x98 - x58*x69) - 0.48*x5*(parms[35]*x71 + x100*x58 + x52 + x99) - 0.48*x66;
  b_out[8] = x101;
  b_out[9] = x102;
  b_out[10] = x96;
  b_out[11] = x93;
  b_out[12] = x68;
  b_out[13] = x101;
  b_out[14] = parms[29] + parms[36] + x105*(parms[48]*x105 + x15*(parms[60]*x7 + parms[61]*x111 + x104*x46 + x117*x31 + x122) + x16*(parms[59]*x103 + parms[61]*x106 + parms[72]*x108 + parms[74]*x106 + x110*x37 + x53*x7) + x56*x7) + x11*(parms[39]*x11 + parms[41]*x7 + parms[46]*x62 + x119*x15 + x121*x16) + x62*(parms[46]*x11 + parms[48]*x62 + parms[58]*x104 + parms[61]*x62 + x103*x35 + x116*x22 + x118) + x7*(parms[41]*x11 + parms[44]*x7 + x105*x56 + x123);
  b_out[15] = x124;
  b_out[16] = x123;
  b_out[17] = x120;
  b_out[18] = x49;
  b_out[19] = x102;
  b_out[20] = x124;
  b_out[21] = parms[42] + parms[49] - x16*(parms[52]*x125 + parms[54]*x15 + x129*x21 + x131*x31 + x132*x39 - 0.2*x133*x21) - x18*(parms[54]*x125 + parms[57]*x15 + x134);
  b_out[22] = x135;
  b_out[23] = x134;
  b_out[24] = x55;
  b_out[25] = x96;
  b_out[26] = x123;
  b_out[27] = x135;
  b_out[28] = parms[55] + parms[62] + x136*(parms[73]*x21 + parms[74]*x136) + x21*(parms[66]*x22 + parms[68]*x21 + parms[73]*x136) + x22*(parms[65]*x22 + parms[66]*x21 + x33*x39) + x39*(parms[74]*x39 + x22*x33);
  b_out[29] = x137;
  b_out[30] = x47;
  b_out[31] = x93;
  b_out[32] = x120;
  b_out[33] = x134;
  b_out[34] = x137;
  b_out[35] = parms[70] + parms[75];
//
  return;
}

} // end namespace robot_dynamics
}
} // end namespace moying

