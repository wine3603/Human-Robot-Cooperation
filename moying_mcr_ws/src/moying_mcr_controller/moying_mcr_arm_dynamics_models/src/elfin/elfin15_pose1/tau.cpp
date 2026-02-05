#include<moying_mcr_arm_dynamics_models/elfin/elfin15_pose1_dynamics_model.h>


namespace baichuan {

namespace arm {

namespace elfin{

void Elfin15Pose1DynamicsModel::tau( double* tau_out, const double* parms, const double* q, const double* dq, const double* ddq )
{
  double x0 = cos(q[1]);
  double x1 = -dq[1];
  double x2 = sin(q[1]);
  double x3 = dq[0]*x2;
  double x4 = ddq[0]*x0 + x1*x3;
  double x5 = ddq[0]*x2;
  double x6 = dq[0]*x0;
  double x7 = -x6;
  double x8 = x1*x7 + x5;
  double x9 = -ddq[1];
  double x10 = -0.73*((x0)*(x0)) - 0.73*((x2)*(x2));
  double x11 = -0.73*dq[1];
  double x12 = x10*x5 + x11*x6;
  double x13 = x10*x3;
  double x14 = -parms[19];
  double x15 = parms[20]*x6 + parms[22]*x13 + x14*x3;
  double x16 = parms[15]*x6 + parms[17]*x3 + parms[18]*x1 + parms[19]*x11;
  double x17 = cos(q[2]);
  double x18 = -dq[3];
  double x19 = -x18;
  double x20 = dq[2] + x1;
  double x21 = cos(q[3]);
  double x22 = x20*x21;
  double x23 = sin(q[2]);
  double x24 = x17*x3 + x23*x7;
  double x25 = sin(q[3]);
  double x26 = x24*x25;
  double x27 = x22 - x26;
  double x28 = -x23;
  double x29 = x17*x6 + x23*x3;
  double x30 = -dq[2];
  double x31 = x17*x8 + x28*x4 + x29*x30;
  double x32 = x21*x31;
  double x33 = ddq[2] + x9;
  double x34 = x25*x33;
  double x35 = x19*x27 + x32 + x34;
  double x36 = dq[2]*x24 + x17*x4 + x23*x8;
  double x37 = -x36;
  double x38 = -ddq[3] + x37;
  double x39 = x21*x24;
  double x40 = x20*x25;
  double x41 = x39 + x40;
  double x42 = x21*x33;
  double x43 = -x25;
  double x44 = x18*x41 + x31*x43 + x42;
  double x45 = x11*x17;
  double x46 = x13*x21 - 0.56747*x39 - 0.56747*x40 + x43*x45;
  double x47 = -0.73*ddq[1] + 9.788*x2;
  double x48 = 9.788*x0;
  double x49 = x11*x23;
  double x50 = x17*x47 + x28*x48 + x30*x49;
  double x51 = -0.56747*x25;
  double x52 = x12*x25 + x19*x46 + x21*x50 + x31*x51 + 0.56747*x42;
  double x53 = sin(q[4]);
  double x54 = cos(q[4]);
  double x55 = -x54;
  double x56 = -x29;
  double x57 = x18 + x56;
  double x58 = x41*x53 + x55*x57;
  double x59 = -dq[4];
  double x60 = x35*x54 + x38*x53 + x58*x59;
  double x61 = ddq[4] + x44;
  double x62 = x41*x54 + x53*x57;
  double x63 = dq[4]*x62 + x35*x53 + x38*x55;
  double x64 = dq[2]*x45 + x17*x48 + x23*x47;
  double x65 = -x64;
  double x66 = x13*x25 + x21*x45 + 0.56747*x22 - 0.56747*x26;
  double x67 = -x49;
  double x68 = x53*x67 + x54*x66;
  double x69 = dq[4]*x68 + x52*x53 + x55*x65;
  double x70 = dq[4] + x27;
  double x71 = -parms[59];
  double x72 = parms[54]*x62 + parms[56]*x70 + parms[57]*x58 + parms[58]*x46 + x68*x71;
  double x73 = cos(q[5]);
  double x74 = x60*x73;
  double x75 = sin(q[5]);
  double x76 = x61*x75;
  double x77 = x70*x73;
  double x78 = x62*x75;
  double x79 = x77 - x78;
  double x80 = dq[5]*x79 + x74 + x76;
  double x81 = x61*x73;
  double x82 = -x75;
  double x83 = x62*x73;
  double x84 = x70*x75;
  double x85 = x83 + x84;
  double x86 = -dq[5];
  double x87 = x60*x82 + x81 + x85*x86;
  double x88 = ddq[5] + x63;
  double x89 = dq[5] + x58;
  double x90 = x46*x73 + x68*x82 - 0.17353*x83 - 0.17353*x84;
  double x91 = x46*x75 + x68*x73 + 0.17353*x77 - 0.17353*x78;
  double x92 = -parms[72];
  double x93 = parms[67]*x85 + parms[69]*x79 + parms[70]*x89 + parms[71]*x90 + x91*x92;
  double x94 = x53*x66 + x55*x67;
  double x95 = -parms[71];
  double x96 = parms[72]*x85 + parms[74]*x94 + x79*x95;
  double x97 = x12*x21 + x18*x66 - 0.56747*x32 - 0.56747*x34 + x43*x50;
  double x98 = x52*x54 + x53*x65 + x59*x94;
  double x99 = x73*x97 - 0.17353*x74 - 0.17353*x76 + x82*x98 + x86*x91;
  double x100 = -parms[73];
  double x101 = parms[71]*x89 + parms[74]*x90 + x100*x85;
  double x102 = -x101;
  double x103 = parms[66]*x85 + parms[68]*x79 + parms[69]*x89 + parms[73]*x91 + x94*x95;
  double x104 = parms[65]*x80 + parms[66]*x87 + parms[67]*x88 + parms[72]*x69 + x100*x99 + x102*x94 - x103*x89 + x79*x93 + x90*x96;
  double x105 = -parms[58];
  double x106 = parms[59]*x62 + parms[61]*x94 + x105*x70;
  double x107 = -parms[60];
  double x108 = parms[53]*x62 + parms[55]*x70 + parms[56]*x58 + parms[60]*x68 + x105*x94;
  double x109 = -x58;
  double x110 = parms[58]*x58 + parms[61]*x46 + x107*x62;
  double x111 = -0.17353*x75;
  double x112 = dq[5]*x90 + x111*x60 + x73*x98 + x75*x97 + 0.17353*x81;
  double x113 = parms[73]*x79 + parms[74]*x91 + x89*x92;
  double x114 = parms[65]*x85 + parms[66]*x79 + parms[67]*x89 + parms[72]*x94 + x100*x90;
  double x115 = -x85;
  double x116 = parms[66]*x80 + parms[68]*x87 + parms[69]*x88 + parms[73]*x112 + x113*x94 + x114*x89 + x115*x93 + x69*x95 - x91*x96;
  double x117 = parms[73]*x87 + parms[74]*x112 + x102*x89 + x79*x96 + x88*x92;
  double x118 = parms[71]*x88 + parms[74]*x99 + x100*x80 + x113*x89 + x115*x96;
  double x119 = x118*x73;
  double x120 = parms[52]*x60 + parms[53]*x61 + parms[54]*x63 + parms[59]*x69 + x104*x73 + x106*x46 + x107*x97 + x108*x109 - x110*x94 + x111*x117 + x116*x82 - 0.17353*x119 + x70*x72;
  double x121 = -parms[47];
  double x122 = parms[39]*x41 + parms[40]*x57 + parms[41]*x27 + parms[46]*x46 + x121*x67;
  double x123 = -parms[46];
  double x124 = parms[47]*x57 + parms[48]*x66 + x123*x27;
  double x125 = -parms[45];
  double x126 = -x79;
  double x127 = parms[67]*x80 + parms[69]*x87 + parms[70]*x88 + parms[71]*x99 + x101*x91 + x103*x85 + x112*x92 - x113*x90 + x114*x126;
  double x128 = parms[52]*x62 + parms[53]*x70 + parms[54]*x58 + parms[59]*x94 + x107*x46;
  double x129 = -x70;
  double x130 = parms[60]*x70 + parms[61]*x68 + x58*x71;
  double x131 = -x46;
  double x132 = parms[54]*x60 + parms[56]*x61 + parms[57]*x63 + parms[58]*x97 + x108*x62 + x110*x68 + x127 + x128*x129 + x130*x131 + x71*x98;
  double x133 = parms[41]*x41 + parms[43]*x57 + parms[44]*x27 + parms[45]*x67 + x123*x66;
  double x134 = -x41;
  double x135 = parms[46]*x41 + parms[48]*x46 + x125*x57;
  double x136 = -parms[40]*x35 - parms[42]*x38 - parms[43]*x44 - parms[47]*x52 - x120*x53 - x122*x27 - x124*x46 - x125*x97 - x132*x55 - x133*x134 + x135*x66;
  double x137 = parms[27]*x24 + parms[29]*x20 + parms[30]*x29 + parms[32]*x67 + parms[34]*x45;
  double x138 = -parms[34];
  double x139 = parms[32]*x29 + parms[35]*x13 + x138*x24;
  double x140 = parms[26]*x24 + parms[27]*x20 + parms[28]*x29 + parms[33]*x49 + x13*x138;
  double x141 = -x20;
  double x142 = parms[33]*x56 + parms[34]*x20 + parms[35]*x45;
  double x143 = -x13;
  double x144 = parms[28]*x31 + parms[30]*x33 + parms[31]*x36 + parms[32]*x12 - parms[33]*x50 + x136 + x137*x24 + x139*x45 + x140*x141 + x142*x143;
  double x145 = -parms[21];
  double x146 = parms[14]*x6 + parms[16]*x3 + parms[17]*x1 + parms[19]*x143;
  double x147 = -x1;
  double x148 = -x45;
  double x149 = parms[28]*x24 + parms[30]*x20 + parms[31]*x29 + parms[32]*x13 + parms[33]*x148;
  double x150 = parms[40]*x41 + parms[42]*x57 + parms[43]*x27 + parms[45]*x131 + parms[47]*x66;
  double x151 = -x27;
  double x152 = parms[45]*x27 + parms[48]*x67 + x121*x41;
  double x153 = parms[39]*x35 + parms[40]*x38 + parms[41]*x44 + parms[46]*x97 + x120*x54 + x121*x65 + x131*x152 + x132*x53 + x133*x57 + x135*x67 + x150*x151;
  double x154 = parms[32]*x141 + parms[33]*x24 + parms[35]*x49;
  double x155 = -x62;
  double x156 = x117*x73;
  double x157 = parms[53]*x60 + parms[55]*x61 + parms[56]*x63 + parms[60]*x98 + x104*x75 + x105*x69 - x106*x68 + x111*x118 + x116*x73 + x128*x58 + x130*x94 + x155*x72 + 0.17353*x156;
  double x158 = -x124;
  double x159 = parms[41]*x35 + parms[43]*x38 + parms[44]*x44 + parms[45]*x65 - x122*x57 + x123*x52 + x150*x41 + x152*x66 + x157 + x158*x67;
  double x160 = parms[60]*x61 + parms[61]*x98 + x106*x70 + x109*x110 + x118*x82 + x156 + x63*x71;
  double x161 = parms[59]*x60 + parms[61]*x69 + parms[72]*x80 + parms[74]*x69 + x101*x85 + x105*x61 + x110*x62 + x113*x126 + x129*x130 + x87*x95;
  double x162 = parms[47]*x38 + parms[48]*x52 + x123*x44 + x135*x57 + x151*x152 + x160*x54 + x161*x53;
  double x163 = parms[46]*x35 + parms[48]*x97 + parms[58]*x63 + parms[61]*x97 + x106*x155 + x107*x60 + x117*x75 + x119 + x125*x38 + x130*x58 + x152*x41 + x158*x57;
  double x164 = x163*x21;
  double x165 = parms[26]*x31 + parms[27]*x33 + parms[28]*x36 + parms[33]*x64 + x12*x138 + x13*x154 + x137*x56 + x139*x67 + x149*x20 + x153*x21 + x159*x43 + x162*x51 - 0.56747*x164;
  double x166 = parms[19]*x1 + parms[21]*x7 + parms[22]*x11;
  double x167 = parms[13]*x6 + parms[14]*x3 + parms[15]*x1 + parms[20]*x13 + x11*x145;
  double x168 = parms[20]*x147 + parms[21]*x3;
  double x169 = -x24;
  double x170 = -x168;
  double x171 = x162*x21;
  double x172 = parms[27]*x31 + parms[29]*x33 + parms[30]*x36 + parms[32]*x65 + parms[34]*x50 + x140*x29 + x142*x49 + x148*x154 + x149*x169 + x153*x25 + x159*x21 + x163*x51 + 0.56747*x171;
//
  tau_out[0] = ddq[0]*parms[10] + ddq[0]*parms[3] + dq[0]*parms[11] + parms[12]*(((dq[0]) > 0) - ((dq[0]) < 0)) + x0*(parms[13]*x4 + parms[14]*x8 + parms[15]*x9 + parms[20]*x12 + x11*x15 + x143*x166 + x144*x17 + x145*x47 + x146*x147 + x16*x3 + x165*x28) + x10*x2*(parms[20]*x4 + parms[22]*x12 + parms[32]*x36 + parms[35]*x12 + x138*x31 + x14*x8 + x142*x29 + x154*x169 + x162*x25 + x164 + x166*x6 + x170*x3) + x2*(parms[14]*x4 + parms[16]*x8 + parms[17]*x9 + parms[21]*x48 + x1*x167 + x12*x14 + x13*x168 + x144*x23 + x16*x7 + x165*x17);
  tau_out[1] = ddq[1]*parms[23] + dq[1]*parms[24] - parms[15]*x4 - parms[17]*x8 - parms[18]*x9 - parms[19]*x47 - 0.73*parms[19]*x9 + parms[20]*x48 - 0.73*parms[22]*x47 + parms[25]*(((dq[1]) > 0) - ((dq[1]) < 0)) - 0.73*x1*x168 - x11*x170 - 0.73*x145*x4 - x146*x6 - 0.73*x15*x7 + x167*x3 - 0.73*x17*(parms[33]*x37 + parms[34]*x33 + parms[35]*x50 + x139*x56 + x154*x20 + x163*x43 + x171) - x172 - 0.73*x23*(-parms[32]*x33 + parms[33]*x31 + parms[35]*x64 - parms[45]*x44 - parms[48]*x65 - x121*x35 - x124*x27 - x134*x135 + x139*x24 + x141*x142 - x160*x53 - x161*x55);
  tau_out[2] = ddq[2]*parms[36] + dq[2]*parms[37] + parms[38]*(((dq[2]) > 0) - ((dq[2]) < 0)) + x172;
  tau_out[3] = ddq[3]*parms[49] + dq[3]*parms[50] + parms[51]*(((dq[3]) > 0) - ((dq[3]) < 0)) + x136;
  tau_out[4] = ddq[4]*parms[62] + dq[4]*parms[63] + parms[64]*(((dq[4]) > 0) - ((dq[4]) < 0)) + x157;
  tau_out[5] = ddq[5]*parms[75] + dq[5]*parms[76] + parms[77]*(((dq[5]) > 0) - ((dq[5]) < 0)) + x127;
//
  return;
}

} // end namespace robot_dynamics
}
} // end namespace moying

