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
  #define CASADI_PREFIX(ID) indyrp2_fkrot_ee_ ## ID
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
#define casadi_s1 CASADI_PREFIX(s1)

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
static const casadi_int casadi_s1[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};

/* fk_rot:(q[7])->(Rot_indyrp2_tcp[3x3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=cos(a0);
  a2=2.2204460492503131e-16;
  a3=arg[0]? arg[0][1] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a3=sin(a3);
  a5=(a5+a3);
  a6=(a1*a5);
  a0=sin(a0);
  a7=(a2*a3);
  a8=(a0*a7);
  a6=(a6-a8);
  a8=arg[0]? arg[0][2] : 0;
  a9=cos(a8);
  a10=(a2*a9);
  a11=-2.2204460492503131e-16;
  a8=sin(a8);
  a12=(a11*a8);
  a10=(a10+a12);
  a12=(a6*a10);
  a13=(a2*a3);
  a13=(a4-a13);
  a14=(a1*a13);
  a15=(a2*a4);
  a16=(a0*a15);
  a14=(a14-a16);
  a16=(a14*a9);
  a17=(a2*a1);
  a17=(a17+a0);
  a18=(a17*a8);
  a16=(a16-a18);
  a12=(a12+a16);
  a16=arg[0]? arg[0][3] : 0;
  a18=cos(a16);
  a19=(a2*a18);
  a16=sin(a16);
  a19=(a19+a16);
  a20=(a12*a19);
  a21=(a11*a9);
  a22=(a2*a8);
  a21=(a21-a22);
  a22=(a6*a21);
  a23=(a14*a8);
  a24=(a17*a9);
  a23=(a23+a24);
  a22=(a22-a23);
  a23=(a2*a16);
  a24=(a22*a23);
  a14=(a2*a14);
  a17=(a2*a17);
  a14=(a14+a17);
  a14=(a14-a6);
  a6=(a2*a16);
  a6=(a6-a18);
  a17=(a14*a6);
  a24=(a24+a17);
  a20=(a20+a24);
  a24=arg[0]? arg[0][4] : 0;
  a17=cos(a24);
  a25=(a2*a17);
  a24=sin(a24);
  a26=(a11*a24);
  a25=(a25+a26);
  a26=(a20*a25);
  a27=(a2*a16);
  a27=(a18-a27);
  a28=(a12*a27);
  a29=(a2*a18);
  a30=(a22*a29);
  a18=(a2*a18);
  a16=(a16+a18);
  a14=(a14*a16);
  a30=(a30+a14);
  a28=(a28+a30);
  a30=(a28*a17);
  a12=(a2*a12);
  a12=(a12-a22);
  a22=(a12*a24);
  a30=(a30-a22);
  a26=(a26+a30);
  a30=arg[0]? arg[0][5] : 0;
  a22=cos(a30);
  a14=(a2*a22);
  a30=sin(a30);
  a14=(a14+a30);
  a18=(a26*a14);
  a31=(a11*a17);
  a32=(a2*a24);
  a31=(a31-a32);
  a32=(a20*a31);
  a33=(a28*a24);
  a34=(a12*a17);
  a33=(a33+a34);
  a32=(a32-a33);
  a33=(a2*a30);
  a34=(a32*a33);
  a28=(a2*a28);
  a12=(a2*a12);
  a28=(a28+a12);
  a28=(a28-a20);
  a20=(a2*a30);
  a20=(a20-a22);
  a12=(a28*a20);
  a34=(a34+a12);
  a18=(a18+a34);
  a34=arg[0]? arg[0][6] : 0;
  a12=cos(a34);
  a35=(a2*a12);
  a34=sin(a34);
  a36=(a11*a34);
  a35=(a35+a36);
  a36=(a18*a35);
  a37=(a2*a30);
  a37=(a22-a37);
  a38=(a26*a37);
  a39=(a2*a22);
  a40=(a32*a39);
  a22=(a2*a22);
  a30=(a30+a22);
  a28=(a28*a30);
  a40=(a40+a28);
  a38=(a38+a40);
  a40=(a38*a12);
  a26=(a2*a26);
  a26=(a26-a32);
  a32=(a26*a34);
  a40=(a40-a32);
  a36=(a36+a40);
  if (res[0]!=0) res[0][0]=a36;
  a5=(a0*a5);
  a7=(a1*a7);
  a5=(a5+a7);
  a7=(a5*a10);
  a13=(a0*a13);
  a15=(a1*a15);
  a13=(a13+a15);
  a15=(a13*a9);
  a0=(a2*a0);
  a0=(a0-a1);
  a1=(a0*a8);
  a15=(a15-a1);
  a7=(a7+a15);
  a15=(a7*a19);
  a1=(a5*a21);
  a36=(a13*a8);
  a40=(a0*a9);
  a36=(a36+a40);
  a1=(a1-a36);
  a36=(a1*a23);
  a13=(a2*a13);
  a0=(a2*a0);
  a13=(a13+a0);
  a13=(a13-a5);
  a5=(a13*a6);
  a36=(a36+a5);
  a15=(a15+a36);
  a36=(a15*a25);
  a5=(a7*a27);
  a0=(a1*a29);
  a13=(a13*a16);
  a0=(a0+a13);
  a5=(a5+a0);
  a0=(a5*a17);
  a7=(a2*a7);
  a7=(a7-a1);
  a1=(a7*a24);
  a0=(a0-a1);
  a36=(a36+a0);
  a0=(a36*a14);
  a1=(a15*a31);
  a13=(a5*a24);
  a40=(a7*a17);
  a13=(a13+a40);
  a1=(a1-a13);
  a13=(a1*a33);
  a5=(a2*a5);
  a7=(a2*a7);
  a5=(a5+a7);
  a5=(a5-a15);
  a15=(a5*a20);
  a13=(a13+a15);
  a0=(a0+a13);
  a13=(a0*a35);
  a15=(a36*a37);
  a7=(a1*a39);
  a5=(a5*a30);
  a7=(a7+a5);
  a15=(a15+a7);
  a7=(a15*a12);
  a36=(a2*a36);
  a36=(a36-a1);
  a1=(a36*a34);
  a7=(a7-a1);
  a13=(a13+a7);
  if (res[0]!=0) res[0][1]=a13;
  a13=(a2*a3);
  a13=(a13-a4);
  a10=(a13*a10);
  a4=(a2*a4);
  a3=(a3+a4);
  a9=(a3*a9);
  a10=(a10+a9);
  a19=(a10*a19);
  a21=(a13*a21);
  a8=(a3*a8);
  a21=(a21-a8);
  a23=(a21*a23);
  a3=(a2*a3);
  a3=(a3-a13);
  a6=(a3*a6);
  a23=(a23+a6);
  a19=(a19+a23);
  a25=(a19*a25);
  a27=(a10*a27);
  a29=(a21*a29);
  a3=(a3*a16);
  a29=(a29+a3);
  a27=(a27+a29);
  a29=(a27*a17);
  a10=(a2*a10);
  a10=(a10-a21);
  a21=(a10*a24);
  a29=(a29-a21);
  a25=(a25+a29);
  a14=(a25*a14);
  a31=(a19*a31);
  a24=(a27*a24);
  a17=(a10*a17);
  a24=(a24+a17);
  a31=(a31-a24);
  a33=(a31*a33);
  a27=(a2*a27);
  a10=(a2*a10);
  a27=(a27+a10);
  a27=(a27-a19);
  a20=(a27*a20);
  a33=(a33+a20);
  a14=(a14+a33);
  a35=(a14*a35);
  a37=(a25*a37);
  a39=(a31*a39);
  a27=(a27*a30);
  a39=(a39+a27);
  a37=(a37+a39);
  a39=(a37*a12);
  a25=(a2*a25);
  a25=(a25-a31);
  a31=(a25*a34);
  a39=(a39-a31);
  a35=(a35+a39);
  if (res[0]!=0) res[0][2]=a35;
  a11=(a11*a12);
  a35=(a2*a34);
  a11=(a11-a35);
  a35=(a18*a11);
  a39=(a38*a34);
  a31=(a26*a12);
  a39=(a39+a31);
  a35=(a35-a39);
  if (res[0]!=0) res[0][3]=a35;
  a35=(a0*a11);
  a39=(a15*a34);
  a31=(a36*a12);
  a39=(a39+a31);
  a35=(a35-a39);
  if (res[0]!=0) res[0][4]=a35;
  a11=(a14*a11);
  a34=(a37*a34);
  a12=(a25*a12);
  a34=(a34+a12);
  a11=(a11-a34);
  if (res[0]!=0) res[0][5]=a11;
  a38=(a2*a38);
  a26=(a2*a26);
  a38=(a38+a26);
  a38=(a38-a18);
  if (res[0]!=0) res[0][6]=a38;
  a15=(a2*a15);
  a36=(a2*a36);
  a15=(a15+a36);
  a15=(a15-a0);
  if (res[0]!=0) res[0][7]=a15;
  a37=(a2*a37);
  a2=(a2*a25);
  a37=(a37+a2);
  a37=(a37-a14);
  if (res[0]!=0) res[0][8]=a37;
  return 0;
}

CASADI_SYMBOL_EXPORT int fk_rot(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int fk_rot_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int fk_rot_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fk_rot_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int fk_rot_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fk_rot_release(int mem) {
}

CASADI_SYMBOL_EXPORT void fk_rot_incref(void) {
}

CASADI_SYMBOL_EXPORT void fk_rot_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int fk_rot_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int fk_rot_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real fk_rot_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fk_rot_name_in(casadi_int i) {
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fk_rot_name_out(casadi_int i) {
  switch (i) {
    case 0: return "Rot_indyrp2_tcp";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fk_rot_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fk_rot_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int fk_rot_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
