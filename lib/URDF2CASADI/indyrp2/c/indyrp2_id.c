/* This file was automatically generated by CasADi 3.6.3+.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) indyrp2_id_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};

/* rnea:(q[7],dq[7],ddq[7])->(tau[7]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a12, a120, a121, a122, a123, a124, a125, a126, a127, a128, a129, a13, a130, a131, a132, a133, a134, a135, a136, a137, a138, a139, a14, a140, a141, a142, a143, a144, a145, a146, a147, a148, a149, a15, a150, a151, a152, a153, a154, a155, a156, a157, a158, a159, a16, a160, a161, a162, a163, a164, a165, a166, a167, a168, a169, a17, a170, a171, a172, a173, a174, a175, a176, a177, a178, a179, a18, a180, a181, a182, a183, a184, a185, a186, a187, a188, a189, a19, a190, a191, a192, a193, a194, a195, a196, a197, a198, a199, a2, a20, a200, a201, a202, a203, a204, a205, a206, a207, a208, a209, a21, a210, a211, a212, a213, a214, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=6.0031470000000003e-02;
  a1=arg[2]? arg[2][0] : 0;
  a0=(a0*a1);
  a2=-2.3749000000000000e-04;
  a3=1.1444445350000001e+01;
  a4=(a2*a1);
  a4=(a3*a4);
  a2=(a2*a4);
  a4=-4.3103130000000003e-02;
  a5=(a4*a1);
  a3=(a3*a5);
  a4=(a4*a3);
  a2=(a2+a4);
  a0=(a0+a2);
  a2=2.2204460492503131e-16;
  a4=arg[0]? arg[0][1] : 0;
  a3=sin(a4);
  a5=(a2*a3);
  a4=cos(a4);
  a5=(a5-a4);
  a6=3.7426809999999977e-02;
  a7=arg[1]? arg[1][1] : 0;
  a8=(a2*a4);
  a8=(a3+a8);
  a9=arg[1]? arg[1][0] : 0;
  a10=(a8*a9);
  a11=(a7*a10);
  a12=(a5*a1);
  a11=(a11+a12);
  a12=(a6*a11);
  a13=3.1760000000000001e-05;
  a14=(a8*a1);
  a15=(a5*a9);
  a16=(a7*a15);
  a14=(a14-a16);
  a16=(a13*a14);
  a12=(a12+a16);
  a16=-7.9067340000000000e-02;
  a17=arg[2]? arg[2][1] : 0;
  a18=(a16*a17);
  a12=(a12+a18);
  a18=-1.7150000000000000e-05;
  a19=4.4330818499999998e+00;
  a20=-1.0929999999999999e-01;
  a1=(a20*a1);
  a21=(a2*a1);
  a22=-2.1209188000000001e-01;
  a23=(a22*a14);
  a24=(a18*a11);
  a23=(a23-a24);
  a23=(a21+a23);
  a23=(a19*a23);
  a24=(a18*a23);
  a25=7.9011910000000005e-02;
  a26=(a2*a4);
  a26=(a26+a3);
  a9=(a20*a9);
  a27=(a26*a9);
  a28=(a7*a27);
  a29=9.8100000000000005e+00;
  a30=(a29*a8);
  a3=(a2*a3);
  a4=(a4-a3);
  a3=(a4*a1);
  a30=(a30-a3);
  a28=(a28+a30);
  a30=(a25*a11);
  a3=(a22*a17);
  a30=(a30-a3);
  a30=(a28-a30);
  a30=(a19*a30);
  a3=(a25*a30);
  a24=(a24+a3);
  a12=(a12-a24);
  a24=(a16*a15);
  a3=-8.0800000000000006e-06;
  a31=(a3*a10);
  a24=(a24+a31);
  a31=2.9396155000000002e-01;
  a32=(a31*a7);
  a24=(a24+a32);
  a32=(a18*a7);
  a33=(a25*a10);
  a32=(a32-a33);
  a32=(a27+a32);
  a32=(a19*a32);
  a33=(a18*a32);
  a34=(a4*a9);
  a35=(a25*a15);
  a36=(a22*a7);
  a35=(a35-a36);
  a35=(a34+a35);
  a35=(a19*a35);
  a36=(a22*a35);
  a33=(a33-a36);
  a24=(a24+a33);
  a33=(a10*a24);
  a36=(a13*a15);
  a37=3.2119653999999997e-01;
  a38=(a37*a10);
  a36=(a36+a38);
  a38=(a3*a7);
  a36=(a36+a38);
  a9=(a2*a9);
  a38=(a22*a10);
  a39=(a18*a15);
  a38=(a38-a39);
  a38=(a9+a38);
  a38=(a19*a38);
  a39=(a22*a38);
  a40=(a25*a32);
  a39=(a39-a40);
  a36=(a36+a39);
  a39=(a7*a36);
  a33=(a33-a39);
  a39=(a34*a38);
  a40=(a9*a35);
  a39=(a39-a40);
  a33=(a33+a39);
  a12=(a12+a33);
  a33=arg[0]? arg[0][2] : 0;
  a39=cos(a33);
  a40=(a2*a39);
  a41=-2.2204460492503131e-16;
  a33=sin(a33);
  a42=(a41*a33);
  a40=(a40+a42);
  a42=4.0981600000000000e-02;
  a43=arg[1]? arg[1][2] : 0;
  a44=(a41*a39);
  a45=(a2*a33);
  a44=(a44-a45);
  a45=(a44*a15);
  a46=(a33*a10);
  a47=(a39*a7);
  a46=(a46+a47);
  a45=(a45-a46);
  a46=(a43*a45);
  a47=(a40*a11);
  a48=(a39*a14);
  a49=(a33*a17);
  a48=(a48-a49);
  a47=(a47+a48);
  a46=(a46+a47);
  a47=(a42*a46);
  a48=7.4699999999999996e-06;
  a49=(a44*a11);
  a50=(a33*a14);
  a51=(a39*a17);
  a50=(a50+a51);
  a49=(a49-a50);
  a50=(a40*a15);
  a51=(a39*a10);
  a52=(a33*a7);
  a51=(a51-a52);
  a50=(a50+a51);
  a51=(a43*a50);
  a49=(a49-a51);
  a51=(a48*a49);
  a47=(a47+a51);
  a51=9.1600000000000004e-06;
  a52=arg[2]? arg[2][2] : 0;
  a53=(a2*a14);
  a54=(a2*a17);
  a53=(a53+a54);
  a53=(a53-a11);
  a52=(a52+a53);
  a53=(a51*a52);
  a47=(a47+a53);
  a53=7.1430549999999968e-02;
  a54=2.8704592799999999e+00;
  a55=8.4699999999999998e-02;
  a56=(a55*a11);
  a57=-3.8400000000000001e-01;
  a58=(a57*a17);
  a56=(a56-a58);
  a28=(a28-a56);
  a56=(a2*a28);
  a58=(a57*a14);
  a21=(a21+a58);
  a58=(a2*a21);
  a56=(a56-a58);
  a29=(a29*a5);
  a1=(a26*a1);
  a29=(a29-a1);
  a1=(a7*a34);
  a29=(a29-a1);
  a1=(a55*a14);
  a1=(a29+a1);
  a56=(a56-a1);
  a58=3.6369999999999986e-05;
  a59=(a58*a49);
  a60=(a53*a46);
  a59=(a59-a60);
  a59=(a56-a59);
  a59=(a54*a59);
  a60=(a53*a59);
  a61=7.7460499999999974e-02;
  a62=(a44*a1);
  a63=(a39*a21);
  a64=(a33*a28);
  a63=(a63-a64);
  a62=(a62+a63);
  a63=(a55*a10);
  a63=(a63-a27);
  a64=(a40*a63);
  a65=(a57*a10);
  a65=(a9+a65);
  a66=(a33*a65);
  a67=(a55*a15);
  a68=(a57*a7);
  a67=(a67-a68);
  a67=(a34+a67);
  a68=(a39*a67);
  a66=(a66-a68);
  a64=(a64+a66);
  a66=(a43*a64);
  a62=(a62-a66);
  a66=(a61*a46);
  a68=(a58*a52);
  a66=(a66-a68);
  a66=(a62-a66);
  a66=(a54*a66);
  a68=(a61*a66);
  a60=(a60-a68);
  a47=(a47+a60);
  a60=(a51*a50);
  a68=1.8171570000000001e-02;
  a69=(a68*a45);
  a60=(a60+a69);
  a69=2.2774770000000000e-02;
  a70=(a2*a10);
  a71=(a2*a7);
  a70=(a70+a71);
  a70=(a70-a15);
  a70=(a43+a70);
  a71=(a69*a70);
  a60=(a60+a71);
  a71=(a44*a63);
  a72=(a33*a67);
  a73=(a39*a65);
  a72=(a72+a73);
  a71=(a71+a72);
  a72=(a61*a50);
  a73=(a58*a70);
  a72=(a72-a73);
  a72=(a71-a72);
  a72=(a54*a72);
  a73=(a58*a72);
  a74=(a53*a70);
  a75=(a61*a45);
  a74=(a74-a75);
  a74=(a64-a74);
  a74=(a54*a74);
  a75=(a53*a74);
  a73=(a73-a75);
  a60=(a60+a73);
  a73=(a45*a60);
  a75=(a48*a50);
  a76=2.1067730000000000e-02;
  a77=(a76*a45);
  a75=(a75+a77);
  a77=(a68*a70);
  a75=(a75+a77);
  a77=(a61*a74);
  a67=(a2*a67);
  a65=(a2*a65);
  a67=(a67+a65);
  a63=(a63+a67);
  a67=(a58*a45);
  a65=(a53*a50);
  a67=(a67-a65);
  a67=(a63+a67);
  a67=(a54*a67);
  a65=(a58*a67);
  a77=(a77+a65);
  a75=(a75+a77);
  a77=(a70*a75);
  a73=(a73-a77);
  a77=(a63*a72);
  a65=(a71*a67);
  a77=(a77-a65);
  a73=(a73+a77);
  a47=(a47+a73);
  a73=arg[0]? arg[0][3] : 0;
  a77=cos(a73);
  a65=(a2*a77);
  a73=sin(a73);
  a65=(a65+a73);
  a78=1.6172099999999995e-02;
  a79=arg[1]? arg[1][3] : 0;
  a80=(a2*a73);
  a80=(a77-a80);
  a81=(a80*a50);
  a82=(a2*a77);
  a83=(a82*a45);
  a84=(a2*a77);
  a84=(a73+a84);
  a85=(a84*a70);
  a83=(a83+a85);
  a81=(a81+a83);
  a83=(a79*a81);
  a85=(a65*a46);
  a86=(a2*a73);
  a87=(a86*a49);
  a73=(a2*a73);
  a73=(a73-a77);
  a77=(a73*a52);
  a87=(a87+a77);
  a85=(a85+a87);
  a83=(a83+a85);
  a85=(a78*a83);
  a87=-1.1817000000000000e-04;
  a77=(a80*a46);
  a88=(a82*a49);
  a89=(a84*a52);
  a88=(a88+a89);
  a77=(a77+a88);
  a88=(a65*a50);
  a89=(a86*a45);
  a90=(a73*a70);
  a89=(a89+a90);
  a88=(a88+a89);
  a89=(a79*a88);
  a77=(a77-a89);
  a89=(a87*a77);
  a85=(a85+a89);
  a89=3.3418820000000002e-02;
  a90=arg[2]? arg[2][3] : 0;
  a91=(a2*a46);
  a91=(a91-a49);
  a90=(a90+a91);
  a91=(a89*a90);
  a85=(a85+a91);
  a91=2.1420999999999994e-04;
  a92=2.6820606400000000e+00;
  a43=(a43*a71);
  a1=(a40*a1);
  a28=(a39*a28);
  a21=(a33*a21);
  a28=(a28+a21);
  a1=(a1+a28);
  a43=(a43+a1);
  a1=1.1530000000000000e-01;
  a28=(a1*a52);
  a21=6.5500000000000003e-02;
  a93=(a21*a49);
  a28=(a28-a93);
  a28=(a43-a28);
  a93=(a2*a28);
  a94=(a21*a46);
  a62=(a62-a94);
  a93=(a93-a62);
  a94=-1.6804015999999994e-01;
  a95=(a94*a77);
  a96=(a91*a83);
  a95=(a95-a96);
  a95=(a93-a95);
  a95=(a92*a95);
  a96=(a91*a95);
  a97=-7.0003829999999975e-02;
  a98=(a80*a28);
  a99=(a82*a62);
  a100=(a1*a46);
  a56=(a56+a100);
  a100=(a84*a56);
  a99=(a99+a100);
  a98=(a98+a99);
  a99=(a1*a70);
  a100=(a21*a45);
  a99=(a99-a100);
  a99=(a64-a99);
  a100=(a65*a99);
  a101=(a21*a50);
  a101=(a71-a101);
  a102=(a86*a101);
  a103=(a1*a50);
  a103=(a103-a63);
  a104=(a73*a103);
  a102=(a102+a104);
  a100=(a100+a102);
  a102=(a79*a100);
  a98=(a98-a102);
  a102=(a97*a83);
  a104=(a94*a90);
  a102=(a102-a104);
  a102=(a98-a102);
  a102=(a92*a102);
  a104=(a97*a102);
  a96=(a96-a104);
  a85=(a85+a96);
  a96=(a89*a88);
  a104=-4.3710000000000000e-05;
  a105=(a104*a81);
  a96=(a96+a105);
  a105=1.0022522000000000e-01;
  a106=(a2*a50);
  a106=(a106-a45);
  a106=(a79+a106);
  a107=(a105*a106);
  a96=(a96+a107);
  a107=(a80*a99);
  a108=(a82*a101);
  a103=(a84*a103);
  a108=(a108+a103);
  a107=(a107+a108);
  a108=(a97*a88);
  a103=(a94*a106);
  a108=(a108-a103);
  a108=(a107-a108);
  a108=(a92*a108);
  a103=(a94*a108);
  a109=(a91*a106);
  a110=(a97*a81);
  a109=(a109-a110);
  a109=(a100-a109);
  a109=(a92*a109);
  a110=(a91*a109);
  a103=(a103-a110);
  a96=(a96+a103);
  a103=(a81*a96);
  a110=(a87*a88);
  a111=1.1364055000000001e-01;
  a112=(a111*a81);
  a110=(a110+a112);
  a112=(a104*a106);
  a110=(a110+a112);
  a112=(a97*a109);
  a99=(a2*a99);
  a99=(a99-a101);
  a101=(a94*a81);
  a113=(a91*a88);
  a101=(a101-a113);
  a101=(a99-a101);
  a101=(a92*a101);
  a113=(a94*a101);
  a112=(a112-a113);
  a110=(a110+a112);
  a112=(a106*a110);
  a103=(a103-a112);
  a112=(a107*a101);
  a113=(a99*a108);
  a112=(a112-a113);
  a103=(a103+a112);
  a85=(a85+a103);
  a103=arg[0]? arg[0][4] : 0;
  a112=cos(a103);
  a113=(a2*a112);
  a103=sin(a103);
  a114=(a41*a103);
  a113=(a113+a114);
  a114=2.7988909999999999e-02;
  a115=arg[1]? arg[1][4] : 0;
  a116=(a41*a112);
  a117=(a2*a103);
  a116=(a116-a117);
  a117=(a116*a88);
  a118=(a103*a81);
  a119=(a112*a106);
  a118=(a118+a119);
  a117=(a117-a118);
  a118=(a115*a117);
  a119=(a113*a83);
  a120=(a112*a77);
  a121=(a103*a90);
  a120=(a120-a121);
  a119=(a119+a120);
  a118=(a118+a119);
  a119=(a114*a118);
  a120=3.8930000000000002e-05;
  a121=(a116*a83);
  a122=(a103*a77);
  a123=(a112*a90);
  a122=(a122+a123);
  a121=(a121-a122);
  a122=(a113*a88);
  a123=(a112*a81);
  a124=(a103*a106);
  a123=(a123-a124);
  a122=(a122+a123);
  a123=(a115*a122);
  a121=(a121-a123);
  a123=(a120*a121);
  a119=(a119+a123);
  a123=-4.7679999999999998e-05;
  a124=arg[2]? arg[2][4] : 0;
  a125=(a2*a77);
  a126=(a2*a90);
  a125=(a125+a126);
  a125=(a125-a83);
  a124=(a124+a125);
  a125=(a123*a124);
  a119=(a119+a125);
  a125=-7.0984400000000003e-02;
  a126=2.1298737100000000e+00;
  a127=-7.4700000000000003e-02;
  a128=(a127*a83);
  a129=-2.6650000000000001e-01;
  a130=(a129*a90);
  a128=(a128-a130);
  a98=(a98-a128);
  a128=(a2*a98);
  a130=(a129*a77);
  a93=(a93-a130);
  a130=(a2*a93);
  a128=(a128+a130);
  a79=(a79*a107);
  a28=(a65*a28);
  a62=(a86*a62);
  a56=(a73*a56);
  a62=(a62+a56);
  a28=(a28+a62);
  a79=(a79+a28);
  a28=(a127*a77);
  a28=(a79+a28);
  a128=(a128-a28);
  a62=-2.6846999999999998e-04;
  a56=(a62*a121);
  a130=(a125*a118);
  a56=(a56-a130);
  a56=(a128-a56);
  a56=(a126*a56);
  a130=(a125*a56);
  a131=7.6491279999999995e-02;
  a132=(a116*a28);
  a133=(a103*a98);
  a134=(a112*a93);
  a133=(a133+a134);
  a132=(a132-a133);
  a133=(a127*a81);
  a133=(a100+a133);
  a134=(a113*a133);
  a135=(a127*a88);
  a136=(a129*a106);
  a135=(a135-a136);
  a135=(a107-a135);
  a136=(a112*a135);
  a137=(a129*a81);
  a137=(a99-a137);
  a138=(a103*a137);
  a136=(a136-a138);
  a134=(a134+a136);
  a136=(a115*a134);
  a132=(a132-a136);
  a136=(a131*a118);
  a138=(a62*a124);
  a136=(a136-a138);
  a136=(a132-a136);
  a136=(a126*a136);
  a138=(a131*a136);
  a130=(a130-a138);
  a119=(a119+a130);
  a130=(a123*a122);
  a138=-1.2662959999999999e-02;
  a139=(a138*a117);
  a130=(a130+a139);
  a139=1.4962110000000001e-02;
  a140=(a2*a81);
  a141=(a2*a106);
  a140=(a140+a141);
  a140=(a140-a88);
  a140=(a115+a140);
  a141=(a139*a140);
  a130=(a130+a141);
  a141=(a116*a133);
  a142=(a103*a135);
  a143=(a112*a137);
  a142=(a142+a143);
  a141=(a141-a142);
  a142=(a131*a122);
  a143=(a62*a140);
  a142=(a142-a143);
  a142=(a141-a142);
  a142=(a126*a142);
  a143=(a62*a142);
  a144=(a125*a140);
  a145=(a131*a117);
  a144=(a144-a145);
  a144=(a134-a144);
  a144=(a126*a144);
  a145=(a125*a144);
  a143=(a143-a145);
  a130=(a130+a143);
  a143=(a117*a130);
  a145=(a120*a122);
  a146=1.4430760000000001e-02;
  a147=(a146*a117);
  a145=(a145+a147);
  a147=(a138*a140);
  a145=(a145+a147);
  a147=(a131*a144);
  a135=(a2*a135);
  a137=(a2*a137);
  a135=(a135+a137);
  a135=(a135-a133);
  a133=(a62*a117);
  a137=(a125*a122);
  a133=(a133-a137);
  a133=(a135-a133);
  a133=(a126*a133);
  a137=(a62*a133);
  a147=(a147-a137);
  a145=(a145+a147);
  a147=(a140*a145);
  a143=(a143-a147);
  a147=(a141*a133);
  a137=(a135*a142);
  a147=(a147-a137);
  a143=(a143+a147);
  a119=(a119+a143);
  a143=arg[0]? arg[0][5] : 0;
  a147=cos(a143);
  a137=(a2*a147);
  a143=sin(a143);
  a137=(a137+a143);
  a148=1.1052970000000002e-02;
  a149=arg[1]? arg[1][5] : 0;
  a150=(a2*a143);
  a150=(a147-a150);
  a151=(a150*a122);
  a152=(a2*a147);
  a153=(a152*a117);
  a154=(a2*a147);
  a154=(a143+a154);
  a155=(a154*a140);
  a153=(a153+a155);
  a151=(a151+a153);
  a153=(a149*a151);
  a155=(a137*a118);
  a156=(a2*a143);
  a157=(a156*a121);
  a143=(a2*a143);
  a143=(a143-a147);
  a147=(a143*a124);
  a157=(a157+a147);
  a155=(a155+a157);
  a153=(a153+a155);
  a155=(a148*a153);
  a157=5.5170000000000002e-05;
  a147=(a150*a118);
  a158=(a152*a121);
  a159=(a154*a124);
  a158=(a158+a159);
  a147=(a147+a158);
  a158=(a137*a122);
  a159=(a156*a117);
  a160=(a143*a140);
  a159=(a159+a160);
  a158=(a158+a159);
  a159=(a149*a158);
  a147=(a147-a159);
  a159=(a157*a147);
  a155=(a155+a159);
  a159=-1.4819769999999999e-02;
  a160=arg[2]? arg[2][5] : 0;
  a161=(a2*a118);
  a161=(a161-a121);
  a160=(a160+a161);
  a161=(a159*a160);
  a155=(a155+a161);
  a161=-2.3113999999999996e-04;
  a162=2.2241227100000001e+00;
  a115=(a115*a141);
  a28=(a113*a28);
  a98=(a112*a98);
  a93=(a103*a93);
  a98=(a98-a93);
  a28=(a28+a98);
  a115=(a115+a28);
  a28=-1.1430000000000000e-01;
  a98=(a28*a124);
  a93=8.3500000000000005e-02;
  a163=(a93*a121);
  a98=(a98-a163);
  a98=(a115-a98);
  a163=(a2*a98);
  a164=(a93*a118);
  a132=(a132-a164);
  a163=(a163-a132);
  a164=-9.7962319999999978e-02;
  a165=(a164*a147);
  a166=(a161*a153);
  a165=(a165-a166);
  a165=(a163-a165);
  a165=(a162*a165);
  a166=(a161*a165);
  a167=6.4458919999999975e-02;
  a168=(a150*a98);
  a169=(a152*a132);
  a170=(a28*a118);
  a128=(a128+a170);
  a170=(a154*a128);
  a169=(a169+a170);
  a168=(a168+a169);
  a169=(a28*a140);
  a170=(a93*a117);
  a169=(a169-a170);
  a169=(a134-a169);
  a170=(a137*a169);
  a171=(a93*a122);
  a171=(a141-a171);
  a172=(a156*a171);
  a173=(a28*a122);
  a173=(a135+a173);
  a174=(a143*a173);
  a172=(a172+a174);
  a170=(a170+a172);
  a172=(a149*a170);
  a168=(a168-a172);
  a172=(a167*a153);
  a174=(a164*a160);
  a172=(a172-a174);
  a172=(a168-a172);
  a172=(a162*a172);
  a174=(a167*a172);
  a166=(a166-a174);
  a155=(a155+a166);
  a166=(a159*a158);
  a174=-3.7400000000000001e-05;
  a175=(a174*a151);
  a166=(a166+a175);
  a175=2.7547950000000002e-02;
  a176=(a2*a122);
  a176=(a176-a117);
  a176=(a149+a176);
  a177=(a175*a176);
  a166=(a166+a177);
  a177=(a150*a169);
  a178=(a152*a171);
  a173=(a154*a173);
  a178=(a178+a173);
  a177=(a177+a178);
  a178=(a167*a158);
  a173=(a164*a176);
  a178=(a178-a173);
  a178=(a177-a178);
  a178=(a162*a178);
  a173=(a164*a178);
  a179=(a161*a176);
  a180=(a167*a151);
  a179=(a179-a180);
  a179=(a170-a179);
  a179=(a162*a179);
  a180=(a161*a179);
  a173=(a173-a180);
  a166=(a166+a173);
  a173=(a151*a166);
  a180=(a157*a158);
  a181=3.6982910000000001e-02;
  a182=(a181*a151);
  a180=(a180+a182);
  a182=(a174*a176);
  a180=(a180+a182);
  a182=(a167*a179);
  a169=(a2*a169);
  a169=(a169-a171);
  a171=(a164*a151);
  a183=(a161*a158);
  a171=(a171-a183);
  a171=(a169-a171);
  a171=(a162*a171);
  a183=(a164*a171);
  a182=(a182-a183);
  a180=(a180+a182);
  a182=(a176*a180);
  a173=(a173-a182);
  a182=(a177*a171);
  a183=(a169*a178);
  a182=(a182-a183);
  a173=(a173+a182);
  a155=(a155+a173);
  a173=arg[0]? arg[0][6] : 0;
  a182=cos(a173);
  a183=(a2*a182);
  a173=sin(a173);
  a184=(a41*a173);
  a183=(a183+a184);
  a184=7.9082085338194890e-04;
  a185=arg[1]? arg[1][6] : 0;
  a41=(a41*a182);
  a186=(a2*a173);
  a41=(a41-a186);
  a186=(a41*a158);
  a187=(a173*a151);
  a188=(a182*a176);
  a187=(a187+a188);
  a186=(a186-a187);
  a187=(a185*a186);
  a188=(a183*a153);
  a189=(a182*a147);
  a190=(a173*a160);
  a189=(a189-a190);
  a188=(a188+a189);
  a187=(a187+a188);
  a188=(a184*a187);
  a189=-3.3999996207092593e-07;
  a190=(a41*a153);
  a191=(a173*a147);
  a192=(a182*a160);
  a191=(a191+a192);
  a190=(a190-a191);
  a191=(a183*a158);
  a192=(a182*a151);
  a193=(a173*a176);
  a192=(a192-a193);
  a191=(a191+a192);
  a192=(a185*a191);
  a190=(a190-a192);
  a192=(a189*a190);
  a188=(a188+a192);
  a192=8.3000237965345357e-07;
  a193=arg[2]? arg[2][6] : 0;
  a194=(a2*a147);
  a195=(a2*a160);
  a194=(a194+a195);
  a194=(a194-a153);
  a193=(a193+a194);
  a194=(a192*a193);
  a188=(a188+a194);
  a194=-4.6555878300977510e-04;
  a195=3.8255032000000000e-01;
  a196=6.8699999999999997e-02;
  a197=(a196*a153);
  a198=-1.6800000000000001e-01;
  a199=(a198*a160);
  a197=(a197-a199);
  a168=(a168-a197);
  a197=(a2*a168);
  a199=(a198*a147);
  a163=(a163-a199);
  a199=(a2*a163);
  a197=(a197+a199);
  a149=(a149*a177);
  a98=(a137*a98);
  a132=(a156*a132);
  a128=(a143*a128);
  a132=(a132+a128);
  a98=(a98+a132);
  a149=(a149+a98);
  a98=(a196*a147);
  a98=(a149+a98);
  a197=(a197-a98);
  a132=8.1469787034552752e-05;
  a128=(a132*a190);
  a199=(a194*a187);
  a128=(a128-a199);
  a197=(a197-a128);
  a197=(a195*a197);
  a128=(a194*a197);
  a199=3.0791046353432411e-02;
  a200=(a41*a98);
  a201=(a173*a168);
  a202=(a182*a163);
  a201=(a201+a202);
  a200=(a200-a201);
  a201=(a196*a151);
  a201=(a170+a201);
  a202=(a183*a201);
  a203=(a196*a158);
  a204=(a198*a176);
  a203=(a203-a204);
  a203=(a177-a203);
  a204=(a182*a203);
  a205=(a198*a151);
  a205=(a169-a205);
  a206=(a173*a205);
  a204=(a204-a206);
  a202=(a202+a204);
  a204=(a185*a202);
  a200=(a200-a204);
  a204=(a199*a187);
  a206=(a132*a193);
  a204=(a204-a206);
  a200=(a200-a204);
  a200=(a195*a200);
  a204=(a199*a200);
  a128=(a128-a204);
  a188=(a188+a128);
  a128=(a192*a191);
  a204=-5.0800135985204594e-06;
  a206=(a204*a186);
  a128=(a128+a206);
  a206=5.8419000022338300e-04;
  a207=(a2*a151);
  a208=(a2*a176);
  a207=(a207+a208);
  a207=(a207-a158);
  a207=(a185+a207);
  a208=(a206*a207);
  a128=(a128+a208);
  a208=(a41*a201);
  a209=(a173*a203);
  a210=(a182*a205);
  a209=(a209+a210);
  a208=(a208-a209);
  a209=(a199*a191);
  a210=(a132*a207);
  a209=(a209-a210);
  a209=(a208-a209);
  a209=(a195*a209);
  a210=(a132*a209);
  a211=(a194*a207);
  a212=(a199*a186);
  a211=(a211-a212);
  a211=(a202-a211);
  a211=(a195*a211);
  a212=(a194*a211);
  a210=(a210-a212);
  a128=(a128+a210);
  a210=(a186*a128);
  a212=(a189*a191);
  a213=7.9864085317184067e-04;
  a214=(a213*a186);
  a212=(a212+a214);
  a214=(a204*a207);
  a212=(a212+a214);
  a214=(a199*a211);
  a203=(a2*a203);
  a205=(a2*a205);
  a203=(a203+a205);
  a203=(a203-a201);
  a201=(a132*a186);
  a205=(a194*a191);
  a201=(a201-a205);
  a201=(a203-a201);
  a201=(a195*a201);
  a205=(a132*a201);
  a214=(a214-a205);
  a212=(a212+a214);
  a214=(a207*a212);
  a210=(a210-a214);
  a214=(a208*a201);
  a205=(a203*a209);
  a214=(a214-a205);
  a210=(a210+a214);
  a188=(a188+a210);
  a210=(a183*a188);
  a214=(a189*a187);
  a213=(a213*a190);
  a214=(a214+a213);
  a213=(a204*a193);
  a214=(a214+a213);
  a185=(a185*a208);
  a98=(a183*a98);
  a168=(a182*a168);
  a163=(a173*a163);
  a168=(a168-a163);
  a98=(a98+a168);
  a185=(a185+a98);
  a98=(a194*a193);
  a168=(a199*a190);
  a98=(a98-a168);
  a185=(a185-a98);
  a195=(a195*a185);
  a185=(a199*a195);
  a98=(a132*a197);
  a185=(a185-a98);
  a214=(a214+a185);
  a184=(a184*a191);
  a189=(a189*a186);
  a184=(a184+a189);
  a189=(a192*a207);
  a184=(a184+a189);
  a189=(a194*a201);
  a199=(a199*a209);
  a189=(a189-a199);
  a184=(a184+a189);
  a189=(a207*a184);
  a128=(a191*a128);
  a189=(a189-a128);
  a203=(a203*a211);
  a128=(a202*a201);
  a203=(a203-a128);
  a189=(a189+a203);
  a214=(a214+a189);
  a189=(a41*a214);
  a192=(a192*a187);
  a204=(a204*a190);
  a192=(a192+a204);
  a206=(a206*a193);
  a192=(a192+a206);
  a132=(a132*a200);
  a194=(a194*a195);
  a132=(a132-a194);
  a192=(a192+a132);
  a212=(a191*a212);
  a184=(a186*a184);
  a212=(a212-a184);
  a202=(a202*a209);
  a208=(a208*a211);
  a202=(a202-a208);
  a212=(a212+a202);
  a192=(a192+a212);
  a189=(a189-a192);
  a210=(a210+a189);
  a189=(a186*a201);
  a212=(a207*a209);
  a189=(a189-a212);
  a195=(a195+a189);
  a189=(a182*a195);
  a209=(a191*a209);
  a186=(a186*a211);
  a209=(a209-a186);
  a197=(a197+a209);
  a209=(a2*a197);
  a207=(a207*a211);
  a191=(a191*a201);
  a207=(a207-a191);
  a200=(a200+a207);
  a207=(a173*a200);
  a209=(a209-a207);
  a189=(a189+a209);
  a209=(a196*a189);
  a210=(a210-a209);
  a155=(a155+a210);
  a210=(a137*a155);
  a209=(a157*a153);
  a181=(a181*a147);
  a209=(a209+a181);
  a181=(a174*a160);
  a209=(a209+a181);
  a181=(a161*a160);
  a207=(a167*a147);
  a181=(a181-a207);
  a149=(a149-a181);
  a162=(a162*a149);
  a149=(a167*a162);
  a181=(a164*a165);
  a149=(a149-a181);
  a209=(a209+a149);
  a148=(a148*a158);
  a157=(a157*a151);
  a148=(a148+a157);
  a157=(a159*a176);
  a148=(a148+a157);
  a157=(a161*a171);
  a167=(a167*a178);
  a157=(a157-a167);
  a148=(a148+a157);
  a157=(a176*a148);
  a166=(a158*a166);
  a157=(a157-a166);
  a169=(a169*a179);
  a166=(a170*a171);
  a169=(a169-a166);
  a157=(a157+a169);
  a209=(a209+a157);
  a157=(a182*a188);
  a169=(a2*a192);
  a166=(a173*a214);
  a169=(a169-a166);
  a157=(a157+a169);
  a183=(a183*a195);
  a41=(a41*a200);
  a41=(a41-a197);
  a183=(a183+a41);
  a196=(a196*a183);
  a197=(a2*a197);
  a200=(a182*a200);
  a197=(a197-a200);
  a195=(a173*a195);
  a197=(a197-a195);
  a195=(a198*a197);
  a196=(a196-a195);
  a157=(a157+a196);
  a209=(a209+a157);
  a157=(a150*a209);
  a159=(a159*a153);
  a174=(a174*a147);
  a159=(a159+a174);
  a175=(a175*a160);
  a159=(a159+a175);
  a164=(a164*a172);
  a161=(a161*a162);
  a164=(a164-a161);
  a159=(a159+a164);
  a180=(a158*a180);
  a148=(a151*a148);
  a180=(a180-a148);
  a170=(a170*a178);
  a177=(a177*a179);
  a170=(a170-a177);
  a180=(a180+a170);
  a159=(a159+a180);
  a180=(a2*a192);
  a182=(a182*a214);
  a180=(a180-a182);
  a173=(a173*a188);
  a180=(a180-a173);
  a198=(a198*a189);
  a180=(a180+a198);
  a159=(a159+a180);
  a180=(a2*a159);
  a157=(a157+a180);
  a210=(a210+a157);
  a157=(a151*a171);
  a180=(a176*a178);
  a157=(a157-a180);
  a162=(a162+a157);
  a162=(a162+a183);
  a183=(a143*a162);
  a176=(a176*a179);
  a171=(a158*a171);
  a176=(a176-a171);
  a172=(a172+a176);
  a172=(a172+a189);
  a189=(a154*a172);
  a183=(a183+a189);
  a189=(a28*a183);
  a176=(a156*a162);
  a171=(a152*a172);
  a158=(a158*a178);
  a151=(a151*a179);
  a158=(a158-a151);
  a165=(a165+a158);
  a165=(a165+a197);
  a171=(a171-a165);
  a176=(a176+a171);
  a171=(a93*a176);
  a189=(a189-a171);
  a210=(a210+a189);
  a119=(a119+a210);
  a210=(a113*a119);
  a189=(a120*a118);
  a146=(a146*a121);
  a189=(a189+a146);
  a146=(a138*a124);
  a189=(a189+a146);
  a146=(a125*a124);
  a171=(a131*a121);
  a146=(a146-a171);
  a115=(a115-a146);
  a126=(a126*a115);
  a115=(a131*a126);
  a146=(a62*a56);
  a115=(a115-a146);
  a189=(a189+a115);
  a114=(a114*a122);
  a120=(a120*a117);
  a114=(a114+a120);
  a120=(a123*a140);
  a114=(a114+a120);
  a120=(a125*a133);
  a131=(a131*a142);
  a120=(a120-a131);
  a114=(a114+a120);
  a120=(a140*a114);
  a130=(a122*a130);
  a120=(a120-a130);
  a135=(a135*a144);
  a130=(a134*a133);
  a135=(a135-a130);
  a120=(a120+a135);
  a189=(a189+a120);
  a156=(a156*a155);
  a152=(a152*a209);
  a152=(a152-a159);
  a156=(a156+a152);
  a137=(a137*a162);
  a150=(a150*a172);
  a165=(a2*a165);
  a150=(a150+a165);
  a137=(a137+a150);
  a93=(a93*a137);
  a156=(a156+a93);
  a189=(a189+a156);
  a156=(a116*a189);
  a123=(a123*a118);
  a138=(a138*a121);
  a123=(a123+a138);
  a139=(a139*a124);
  a123=(a123+a139);
  a62=(a62*a136);
  a125=(a125*a126);
  a62=(a62-a125);
  a123=(a123+a62);
  a145=(a122*a145);
  a114=(a117*a114);
  a145=(a145-a114);
  a134=(a134*a142);
  a141=(a141*a144);
  a134=(a134-a141);
  a145=(a145+a134);
  a123=(a123+a145);
  a143=(a143*a155);
  a154=(a154*a209);
  a143=(a143+a154);
  a28=(a28*a137);
  a143=(a143-a28);
  a123=(a123+a143);
  a156=(a156-a123);
  a210=(a210+a156);
  a156=(a117*a133);
  a143=(a140*a142);
  a156=(a156-a143);
  a126=(a126+a156);
  a126=(a126+a137);
  a137=(a112*a126);
  a142=(a122*a142);
  a117=(a117*a144);
  a142=(a142-a117);
  a56=(a56+a142);
  a56=(a56+a183);
  a183=(a2*a56);
  a140=(a140*a144);
  a122=(a122*a133);
  a140=(a140-a122);
  a136=(a136+a140);
  a136=(a136+a176);
  a176=(a103*a136);
  a183=(a183-a176);
  a137=(a137+a183);
  a183=(a127*a137);
  a210=(a210-a183);
  a85=(a85+a210);
  a210=(a65*a85);
  a183=(a87*a83);
  a111=(a111*a77);
  a183=(a183+a111);
  a111=(a104*a90);
  a183=(a183+a111);
  a111=(a91*a90);
  a176=(a97*a77);
  a111=(a111-a176);
  a79=(a79-a111);
  a92=(a92*a79);
  a79=(a97*a92);
  a111=(a94*a95);
  a79=(a79-a111);
  a183=(a183+a79);
  a78=(a78*a88);
  a87=(a87*a81);
  a78=(a78+a87);
  a87=(a89*a106);
  a78=(a78+a87);
  a87=(a91*a101);
  a97=(a97*a108);
  a87=(a87-a97);
  a78=(a78+a87);
  a87=(a106*a78);
  a96=(a88*a96);
  a87=(a87-a96);
  a99=(a99*a109);
  a96=(a100*a101);
  a99=(a99-a96);
  a87=(a87+a99);
  a183=(a183+a87);
  a87=(a112*a119);
  a99=(a2*a123);
  a96=(a103*a189);
  a99=(a99-a96);
  a87=(a87+a99);
  a113=(a113*a126);
  a116=(a116*a136);
  a116=(a116-a56);
  a113=(a113+a116);
  a127=(a127*a113);
  a56=(a2*a56);
  a136=(a112*a136);
  a56=(a56-a136);
  a126=(a103*a126);
  a56=(a56-a126);
  a126=(a129*a56);
  a127=(a127-a126);
  a87=(a87+a127);
  a183=(a183+a87);
  a87=(a80*a183);
  a89=(a89*a83);
  a104=(a104*a77);
  a89=(a89+a104);
  a105=(a105*a90);
  a89=(a89+a105);
  a94=(a94*a102);
  a91=(a91*a92);
  a94=(a94-a91);
  a89=(a89+a94);
  a110=(a88*a110);
  a78=(a81*a78);
  a110=(a110-a78);
  a100=(a100*a108);
  a107=(a107*a109);
  a100=(a100-a107);
  a110=(a110+a100);
  a89=(a89+a110);
  a110=(a2*a123);
  a112=(a112*a189);
  a110=(a110-a112);
  a103=(a103*a119);
  a110=(a110-a103);
  a129=(a129*a137);
  a110=(a110+a129);
  a89=(a89+a110);
  a110=(a2*a89);
  a87=(a87+a110);
  a210=(a210+a87);
  a87=(a81*a101);
  a110=(a106*a108);
  a87=(a87-a110);
  a92=(a92+a87);
  a92=(a92+a113);
  a113=(a73*a92);
  a106=(a106*a109);
  a101=(a88*a101);
  a106=(a106-a101);
  a102=(a102+a106);
  a102=(a102+a137);
  a137=(a84*a102);
  a113=(a113+a137);
  a137=(a1*a113);
  a106=(a86*a92);
  a101=(a82*a102);
  a88=(a88*a108);
  a81=(a81*a109);
  a88=(a88-a81);
  a95=(a95+a88);
  a95=(a95+a56);
  a101=(a101-a95);
  a106=(a106+a101);
  a101=(a21*a106);
  a137=(a137-a101);
  a210=(a210+a137);
  a47=(a47+a210);
  a210=(a40*a47);
  a137=(a48*a46);
  a76=(a76*a49);
  a137=(a137+a76);
  a76=(a68*a52);
  a137=(a137+a76);
  a76=(a53*a52);
  a101=(a61*a49);
  a76=(a76-a101);
  a43=(a43-a76);
  a54=(a54*a43);
  a43=(a61*a54);
  a76=(a58*a59);
  a43=(a43-a76);
  a137=(a137+a43);
  a42=(a42*a50);
  a48=(a48*a45);
  a42=(a42+a48);
  a48=(a51*a70);
  a42=(a42+a48);
  a48=(a53*a67);
  a61=(a61*a72);
  a48=(a48+a61);
  a42=(a42-a48);
  a48=(a70*a42);
  a60=(a50*a60);
  a48=(a48-a60);
  a60=(a64*a67);
  a63=(a63*a74);
  a60=(a60-a63);
  a48=(a48+a60);
  a137=(a137+a48);
  a86=(a86*a85);
  a82=(a82*a183);
  a82=(a82-a89);
  a86=(a86+a82);
  a65=(a65*a92);
  a80=(a80*a102);
  a95=(a2*a95);
  a80=(a80+a95);
  a65=(a65+a80);
  a21=(a21*a65);
  a86=(a86+a21);
  a137=(a137+a86);
  a86=(a44*a137);
  a51=(a51*a46);
  a68=(a68*a49);
  a51=(a51+a68);
  a69=(a69*a52);
  a51=(a51+a69);
  a58=(a58*a66);
  a53=(a53*a54);
  a58=(a58-a53);
  a51=(a51+a58);
  a75=(a50*a75);
  a42=(a45*a42);
  a75=(a75-a42);
  a64=(a64*a72);
  a71=(a71*a74);
  a64=(a64-a71);
  a75=(a75+a64);
  a51=(a51+a75);
  a73=(a73*a85);
  a84=(a84*a183);
  a73=(a73+a84);
  a1=(a1*a65);
  a73=(a73-a1);
  a51=(a51+a73);
  a86=(a86-a51);
  a210=(a210+a86);
  a86=(a45*a67);
  a73=(a70*a72);
  a86=(a86+a73);
  a54=(a54-a86);
  a54=(a54+a65);
  a65=(a39*a54);
  a72=(a50*a72);
  a45=(a45*a74);
  a72=(a72-a45);
  a59=(a59+a72);
  a59=(a59+a113);
  a113=(a2*a59);
  a70=(a70*a74);
  a50=(a50*a67);
  a70=(a70+a50);
  a66=(a66+a70);
  a66=(a66+a106);
  a106=(a33*a66);
  a113=(a113-a106);
  a65=(a65+a113);
  a113=(a55*a65);
  a210=(a210-a113);
  a12=(a12+a210);
  a5=(a5*a12);
  a12=(a13*a11);
  a37=(a37*a14);
  a12=(a12+a37);
  a37=(a3*a17);
  a12=(a12+a37);
  a37=(a18*a17);
  a210=(a25*a14);
  a37=(a37-a210);
  a29=(a29-a37);
  a19=(a19*a29);
  a29=(a25*a19);
  a37=(a22*a23);
  a29=(a29+a37);
  a12=(a12+a29);
  a6=(a6*a15);
  a13=(a13*a10);
  a6=(a6+a13);
  a13=(a16*a7);
  a6=(a6+a13);
  a25=(a25*a35);
  a13=(a18*a38);
  a25=(a25-a13);
  a6=(a6+a25);
  a25=(a7*a6);
  a24=(a15*a24);
  a25=(a25-a24);
  a9=(a9*a32);
  a24=(a27*a38);
  a9=(a9-a24);
  a25=(a25+a9);
  a12=(a12+a25);
  a25=(a39*a47);
  a9=(a2*a51);
  a24=(a33*a137);
  a9=(a9-a24);
  a25=(a25+a9);
  a40=(a40*a54);
  a44=(a44*a66);
  a44=(a44-a59);
  a40=(a40+a44);
  a55=(a55*a40);
  a59=(a2*a59);
  a66=(a39*a66);
  a59=(a59-a66);
  a54=(a33*a54);
  a59=(a59-a54);
  a54=(a57*a59);
  a55=(a55-a54);
  a25=(a25+a55);
  a12=(a12+a25);
  a8=(a8*a12);
  a5=(a5+a8);
  a8=(a7*a35);
  a12=(a10*a38);
  a8=(a8-a12);
  a8=(a19+a8);
  a8=(a8+a40);
  a26=(a26*a8);
  a38=(a15*a38);
  a7=(a7*a32);
  a38=(a38-a7);
  a38=(a30+a38);
  a38=(a38+a65);
  a4=(a4*a38);
  a38=(a10*a32);
  a7=(a15*a35);
  a38=(a38-a7);
  a38=(a38-a23);
  a38=(a38+a59);
  a38=(a2*a38);
  a4=(a4+a38);
  a26=(a26+a4);
  a20=(a20*a26);
  a5=(a5-a20);
  a0=(a0+a5);
  if (res[0]!=0) res[0][0]=a0;
  a16=(a16*a11);
  a3=(a3*a14);
  a16=(a16+a3);
  a31=(a31*a17);
  a16=(a16+a31);
  a22=(a22*a30);
  a18=(a18*a19);
  a22=(a22-a18);
  a16=(a16+a22);
  a15=(a15*a36);
  a10=(a10*a6);
  a15=(a15-a10);
  a27=(a27*a35);
  a34=(a34*a32);
  a27=(a27-a34);
  a15=(a15+a27);
  a16=(a16+a15);
  a2=(a2*a51);
  a39=(a39*a137);
  a2=(a2-a39);
  a33=(a33*a47);
  a2=(a2-a33);
  a57=(a57*a65);
  a2=(a2+a57);
  a16=(a16+a2);
  if (res[0]!=0) res[0][1]=a16;
  if (res[0]!=0) res[0][2]=a51;
  if (res[0]!=0) res[0][3]=a89;
  if (res[0]!=0) res[0][4]=a123;
  if (res[0]!=0) res[0][5]=a159;
  if (res[0]!=0) res[0][6]=a192;
  return 0;
}

CASADI_SYMBOL_EXPORT int rnea(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int rnea_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int rnea_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void rnea_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int rnea_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void rnea_release(int mem) {
}

CASADI_SYMBOL_EXPORT void rnea_incref(void) {
}

CASADI_SYMBOL_EXPORT void rnea_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int rnea_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int rnea_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real rnea_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rnea_name_in(casadi_int i) {
  switch (i) {
    case 0: return "q";
    case 1: return "dq";
    case 2: return "ddq";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rnea_name_out(casadi_int i) {
  switch (i) {
    case 0: return "tau";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rnea_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rnea_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int rnea_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
