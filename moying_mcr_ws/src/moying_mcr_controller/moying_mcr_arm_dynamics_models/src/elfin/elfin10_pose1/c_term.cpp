#include<moying_mcr_arm_dynamics_models/elfin/elfin10_pose1_dynamics_model.h>


namespace baichuan {

namespace arm {

namespace elfin{

void Elfin10Pose1DynamicsModel::coriolisTerm( double* c_out, const double* parms, const double* q, const double* dq )
{
  double x0 = cos(q[1]);
  double x1 = -dq[1];
  double x2 = sin(q[1]);
  double x3 = dq[0]*x2;
  double x4 = x1*x3;
  double x5 = dq[0]*x0;
  double x6 = -x5;
  double x7 = x1*x6;
  double x8 = -0.48*dq[1];
  double x9 = x5*x8;
  double x10 = -0.48*((x0)*(x0)) - 0.48*((x2)*(x2));
  double x11 = x10*x3;
  double x12 = -parms[19];
  double x13 = parms[20]*x5 + parms[22]*x11 + x12*x3;
  double x14 = parms[15]*x5 + parms[17]*x3 + parms[18]*x1 + parms[19]*x8;
  double x15 = cos(q[2]);
  double x16 = sin(q[2]);
  double x17 = -x16;
  double x18 = x15*x5 + x16*x3;
  double x19 = -dq[2];
  double x20 = x15*x7 + x17*x4 + x18*x19;
  double x21 = cos(q[3]);
  double x22 = x20*x21;
  double x23 = dq[2] + x1;
  double x24 = x21*x23;
  double x25 = x15*x3 + x16*x6;
  double x26 = sin(q[3]);
  double x27 = x25*x26;
  double x28 = x24 - x27;
  double x29 = -dq[3];
  double x30 = -x29;
  double x31 = x22 + x28*x30;
  double x32 = dq[2]*x25 + x15*x4 + x16*x7;
  double x33 = -x32;
  double x34 = -x26;
  double x35 = x21*x25;
  double x36 = x23*x26;
  double x37 = x35 + x36;
  double x38 = x20*x34 + x29*x37;
  double x39 = x16*x8;
  double x40 = x19*x39;
  double x41 = -0.52*x26;
  double x42 = x15*x8;
  double x43 = x11*x21 + x34*x42 - 0.52*x35 - 0.52*x36;
  double x44 = x20*x41 + x21*x40 + x26*x9 + x30*x43;
  double x45 = sin(q[4]);
  double x46 = cos(q[4]);
  double x47 = -dq[4];
  double x48 = -x18;
  double x49 = x29 + x48;
  double x50 = -x46;
  double x51 = x37*x45 + x49*x50;
  double x52 = x31*x46 + x33*x45 + x47*x51;
  double x53 = x37*x46 + x45*x49;
  double x54 = dq[4]*x53 + x31*x45 + x33*x50;
  double x55 = dq[2]*x42;
  double x56 = -x55;
  double x57 = x11*x26 + x21*x42 + 0.52*x24 - 0.52*x27;
  double x58 = -x39;
  double x59 = x45*x58 + x46*x57;
  double x60 = dq[4]*x59 + x44*x45 + x50*x56;
  double x61 = dq[4] + x28;
  double x62 = -parms[59];
  double x63 = parms[54]*x53 + parms[56]*x61 + parms[57]*x51 + parms[58]*x43 + x59*x62;
  double x64 = cos(q[5]);
  double x65 = x52*x64;
  double x66 = sin(q[5]);
  double x67 = x38*x66;
  double x68 = x61*x64;
  double x69 = x53*x66;
  double x70 = x68 - x69;
  double x71 = dq[5]*x70 + x65 + x67;
  double x72 = x38*x64;
  double x73 = -x66;
  double x74 = x61*x66;
  double x75 = x53*x64;
  double x76 = x74 + x75;
  double x77 = -dq[5];
  double x78 = x52*x73 + x72 + x76*x77;
  double x79 = dq[5] + x51;
  double x80 = x43*x64 + x59*x73 - 0.2*x74 - 0.2*x75;
  double x81 = x43*x66 + x59*x64 + 0.2*x68 - 0.2*x69;
  double x82 = -parms[72];
  double x83 = parms[67]*x76 + parms[69]*x70 + parms[70]*x79 + parms[71]*x80 + x81*x82;
  double x84 = x45*x57 + x50*x58;
  double x85 = -parms[71];
  double x86 = parms[72]*x76 + parms[74]*x84 + x70*x85;
  double x87 = x21*x9 - 0.52*x22 + x29*x57 + x34*x40;
  double x88 = x44*x46 + x45*x56 + x47*x84;
  double x89 = x64*x87 - 0.2*x65 - 0.2*x67 + x73*x88 + x77*x81;
  double x90 = -parms[73];
  double x91 = parms[66]*x76 + parms[68]*x70 + parms[69]*x79 + parms[73]*x81 + x84*x85;
  double x92 = -x79;
  double x93 = parms[71]*x79 + parms[74]*x80 + x76*x90;
  double x94 = -x84;
  double x95 = parms[65]*x71 + parms[66]*x78 + parms[67]*x54 + parms[72]*x60 + x70*x83 + x80*x86 + x89*x90 + x91*x92 + x93*x94;
  double x96 = -parms[58];
  double x97 = parms[59]*x53 + parms[61]*x84 + x61*x96;
  double x98 = -parms[60];
  double x99 = -0.2*x66;
  double x100 = dq[5]*x80 + x52*x99 + x64*x88 + x66*x87 + 0.2*x72;
  double x101 = parms[65]*x76 + parms[66]*x70 + parms[67]*x79 + parms[72]*x84 + x80*x90;
  double x102 = parms[73]*x70 + parms[74]*x81 + x79*x82;
  double x103 = -x76;
  double x104 = parms[66]*x71 + parms[68]*x78 + parms[69]*x54 + parms[73]*x100 + x101*x79 + x102*x84 + x103*x83 + x60*x85 - x81*x86;
  double x105 = parms[53]*x53 + parms[55]*x61 + parms[56]*x51 + parms[58]*x94 + parms[60]*x59;
  double x106 = -x51;
  double x107 = parms[58]*x51 + parms[61]*x43 + x53*x98;
  double x108 = parms[73]*x78 + parms[74]*x100 + x54*x82 + x70*x86 + x92*x93;
  double x109 = parms[71]*x54 + parms[74]*x89 + x102*x79 + x103*x86 + x71*x90;
  double x110 = x109*x64;
  double x111 = parms[52]*x52 + parms[53]*x38 + parms[54]*x54 + parms[59]*x60 + x104*x73 + x105*x106 + x107*x94 + x108*x99 - 0.2*x110 + x43*x97 + x61*x63 + x64*x95 + x87*x98;
  double x112 = -parms[47];
  double x113 = parms[39]*x37 + parms[40]*x49 + parms[41]*x28 + parms[46]*x43 + x112*x58;
  double x114 = -parms[46];
  double x115 = parms[47]*x49 + parms[48]*x57 + x114*x28;
  double x116 = -parms[45];
  double x117 = -x70;
  double x118 = parms[67]*x71 + parms[69]*x78 + parms[70]*x54 + parms[71]*x89 + x100*x82 + x101*x117 - x102*x80 + x76*x91 + x81*x93;
  double x119 = parms[52]*x53 + parms[53]*x61 + parms[54]*x51 + parms[59]*x84 + x43*x98;
  double x120 = -x61;
  double x121 = parms[60]*x61 + parms[61]*x59 + x51*x62;
  double x122 = -x43;
  double x123 = parms[54]*x52 + parms[56]*x38 + parms[57]*x54 + parms[58]*x87 + x105*x53 + x107*x59 + x118 + x119*x120 + x121*x122 + x62*x88;
  double x124 = parms[41]*x37 + parms[43]*x49 + parms[44]*x28 + parms[45]*x58 + x114*x57;
  double x125 = -x37;
  double x126 = parms[46]*x37 + parms[48]*x43 + x116*x49;
  double x127 = -parms[40]*x31 - parms[42]*x33 - parms[43]*x38 - parms[47]*x44 - x111*x45 - x113*x28 - x115*x43 - x116*x87 - x123*x50 - x124*x125 + x126*x57;
  double x128 = parms[27]*x25 + parms[29]*x23 + parms[30]*x18 + parms[32]*x58 + parms[34]*x42;
  double x129 = -parms[34];
  double x130 = parms[32]*x18 + parms[35]*x11 + x129*x25;
  double x131 = parms[26]*x25 + parms[27]*x23 + parms[28]*x18 + parms[33]*x39 + x11*x129;
  double x132 = -x23;
  double x133 = parms[33]*x48 + parms[34]*x23 + parms[35]*x42;
  double x134 = -x11;
  double x135 = parms[28]*x20 + parms[31]*x32 + parms[32]*x9 - parms[33]*x40 + x127 + x128*x25 + x130*x42 + x131*x132 + x133*x134;
  double x136 = parms[14]*x5 + parms[16]*x3 + parms[17]*x1 + parms[19]*x134;
  double x137 = -x1;
  double x138 = -x42;
  double x139 = parms[28]*x25 + parms[30]*x23 + parms[31]*x18 + parms[32]*x11 + parms[33]*x138;
  double x140 = parms[40]*x37 + parms[42]*x49 + parms[43]*x28 + parms[45]*x122 + parms[47]*x57;
  double x141 = -x28;
  double x142 = parms[45]*x28 + parms[48]*x58 + x112*x37;
  double x143 = parms[39]*x31 + parms[40]*x33 + parms[41]*x38 + parms[46]*x87 + x111*x46 + x112*x56 + x122*x142 + x123*x45 + x124*x49 + x126*x58 + x140*x141;
  double x144 = parms[32]*x132 + parms[33]*x25 + parms[35]*x39;
  double x145 = -x53;
  double x146 = x108*x64;
  double x147 = parms[53]*x52 + parms[55]*x38 + parms[56]*x54 + parms[60]*x88 + x104*x64 + x109*x99 + x119*x51 + x121*x84 + x145*x63 + 0.2*x146 - x59*x97 + x60*x96 + x66*x95;
  double x148 = -x115;
  double x149 = parms[41]*x31 + parms[43]*x33 + parms[44]*x38 + parms[45]*x56 - x113*x49 + x114*x44 + x140*x37 + x142*x57 + x147 + x148*x58;
  double x150 = parms[60]*x38 + parms[61]*x88 + x106*x107 + x109*x73 + x146 + x54*x62 + x61*x97;
  double x151 = parms[59]*x52 + parms[61]*x60 + parms[72]*x71 + parms[74]*x60 + x102*x117 + x107*x53 + x120*x121 + x38*x96 + x76*x93 + x78*x85;
  double x152 = parms[47]*x33 + parms[48]*x44 + x114*x38 + x126*x49 + x141*x142 + x150*x46 + x151*x45;
  double x153 = parms[46]*x31 + parms[48]*x87 + parms[58]*x54 + parms[61]*x87 + x108*x66 + x110 + x116*x33 + x121*x51 + x142*x37 + x145*x97 + x148*x49 + x52*x98;
  double x154 = x153*x21;
  double x155 = parms[26]*x20 + parms[28]*x32 + parms[33]*x55 + x11*x144 + x128*x48 + x129*x9 + x130*x58 + x139*x23 + x143*x21 + x149*x34 + x152*x41 - 0.52*x154;
  double x156 = parms[19]*x1 + parms[21]*x6 + parms[22]*x8;
  double x157 = -x8;
  double x158 = parms[13]*x5 + parms[14]*x3 + parms[15]*x1 + parms[20]*x11 + parms[21]*x157;
  double x159 = parms[20]*x137 + parms[21]*x3;
  double x160 = -x25;
  double x161 = -x3;
  double x162 = x152*x21;
  double x163 = parms[27]*x20 + parms[30]*x32 + parms[32]*x56 + parms[34]*x40 + x131*x18 + x133*x39 + x138*x144 + x139*x160 + x143*x26 + x149*x21 + x153*x41 + 0.52*x162;
//
  c_out[0] = x0*(parms[13]*x4 + parms[14]*x7 + parms[20]*x9 + x13*x8 + x134*x156 + x135*x15 + x136*x137 + x14*x3 + x155*x17) + x10*x2*(parms[20]*x4 + parms[22]*x9 + parms[32]*x32 + parms[35]*x9 + x12*x7 + x129*x20 + x133*x18 + x144*x160 + x152*x26 + x154 + x156*x5 + x159*x161) + x2*(parms[14]*x4 + parms[16]*x7 + x1*x158 + x11*x159 + x12*x9 + x135*x16 + x14*x6 + x15*x155);
  c_out[1] = -parms[15]*x4 - parms[17]*x7 + 0.48*parms[21]*x4 - 0.48*x1*x159 - 0.48*x13*x6 - x136*x5 - 0.48*x15*(parms[33]*x33 + parms[35]*x40 + x130*x48 + x144*x23 + x153*x34 + x162) - x157*x159 - x158*x161 - 0.48*x16*(parms[33]*x20 + parms[35]*x55 - parms[45]*x38 - parms[48]*x56 - x112*x31 - x115*x28 - x125*x126 + x130*x25 + x132*x133 - x150*x45 - x151*x50) - x163;
  c_out[2] = x163;
  c_out[3] = x127;
  c_out[4] = x147;
  c_out[5] = x118;
//
  return;
}

} // end namespace robot_dynamics
}
} // end namespace moying

