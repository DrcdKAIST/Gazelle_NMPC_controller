/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) ceq0_func_ ## ID
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>
#ifdef MATLAB_MEX_FILE
#include <mex.h>
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_from_mex CASADI_PREFIX(from_mex)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_to_mex CASADI_PREFIX(to_mex)

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

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

#ifdef MATLAB_MEX_FILE
casadi_real* casadi_from_mex(const mxArray* p, casadi_real* y, const casadi_int* sp, casadi_real* w) {
  casadi_int nrow, ncol, is_sparse, c, k, p_nrow, p_ncol;
  const casadi_int *colind, *row;
  mwIndex *Jc, *Ir;
  const double* p_data;
  if (!mxIsDouble(p) || mxGetNumberOfDimensions(p)!=2)
    mexErrMsgIdAndTxt("Casadi:RuntimeError",
      "\"from_mex\" failed: Not a two-dimensional matrix of double precision.");
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
  p_nrow = mxGetM(p);
  p_ncol = mxGetN(p);
  is_sparse = mxIsSparse(p);
  Jc = 0;
  Ir = 0;
  if (is_sparse) {
    Jc = mxGetJc(p);
    Ir = mxGetIr(p);
  }
  p_data = (const double*)mxGetData(p);
  if (p_nrow==1 && p_ncol==1) {
    casadi_int nnz;
    double v = is_sparse && Jc[1]==0 ? 0 : *p_data;
    nnz = sp[ncol];
    casadi_fill(y, nnz, v);
  } else {
    casadi_int tr = 0;
    if (nrow!=p_nrow || ncol!=p_ncol) {
      tr = nrow==p_ncol && ncol==p_nrow && (nrow==1 || ncol==1);
      if (!tr) mexErrMsgIdAndTxt("Casadi:RuntimeError",
                                 "\"from_mex\" failed: Dimension mismatch. "
                                 "Expected %d-by-%d, got %d-by-%d instead.",
                                 nrow, ncol, p_nrow, p_ncol);
    }
    if (is_sparse) {
      if (tr) {
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]+c*nrow]=0;
        for (c=0; c<p_ncol; ++c)
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[c+Ir[k]*p_ncol] = p_data[k];
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) y[k] = w[row[k]+c*nrow];
      } else {
        for (c=0; c<ncol; ++c) {
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]]=0;
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[Ir[k]]=p_data[k];
          for (k=colind[c]; k<colind[c+1]; ++k) y[k]=w[row[k]];
        }
      }
    } else {
      for (c=0; c<ncol; ++c) {
        for (k=colind[c]; k<colind[c+1]; ++k) {
          y[k] = p_data[row[k]+c*nrow];
        }
      }
    }
  }
  return y;
}

#endif

#define casadi_to_double(x) ((double) x)

#ifdef MATLAB_MEX_FILE
mxArray* casadi_to_mex(const casadi_int* sp, const casadi_real* x) {
  casadi_int nrow, ncol, c, k;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int nnz;
#endif
  const casadi_int *colind, *row;
  mxArray *p;
  double *d;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int i;
  mwIndex *j;
#endif /* CASADI_MEX_NO_SPARSE */
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
#ifndef CASADI_MEX_NO_SPARSE
  nnz = sp[ncol];
  if (nnz!=nrow*ncol) {
    p = mxCreateSparse(nrow, ncol, nnz, mxREAL);
    for (i=0, j=mxGetJc(p); i<=ncol; ++i) *j++ = *colind++;
    for (i=0, j=mxGetIr(p); i<nnz; ++i) *j++ = *row++;
    if (x) {
      d = (double*)mxGetData(p);
      for (i=0; i<nnz; ++i) *d++ = casadi_to_double(*x++);
    }
    return p;
  }
#endif /* CASADI_MEX_NO_SPARSE */
  p = mxCreateDoubleMatrix(nrow, ncol, mxREAL);
  if (x) {
    d = (double*)mxGetData(p);
    for (c=0; c<ncol; ++c) {
      for (k=colind[c]; k<colind[c+1]; ++k) {
        d[row[k]+c*nrow] = casadi_to_double(*x++);
      }
    }
  }
  return p;
}

#endif

#ifndef CASADI_PRINTF
#ifdef MATLAB_MEX_FILE
  #define CASADI_PRINTF mexPrintf
#else
  #define CASADI_PRINTF printf
#endif
#endif

static const casadi_int casadi_s0[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s1[44] = {40, 1, 0, 40, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
static const casadi_int casadi_s2[184] = {180, 1, 0, 180, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* ceq0_func:(i0[2],i1[40],i2[180],i3,i4,i5,i6,i7,i8)->(o0[40]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][0] : 0;
  a1=1.;
  a2=arg[5]? arg[5][0] : 0;
  a3=arg[6]? arg[6][0] : 0;
  a4=(a2*a3);
  a4=(a1+a4);
  a5=arg[0]? arg[0][0] : 0;
  a5=(a4*a5);
  a6=(a2*a3);
  a7=arg[2]? arg[2][0] : 0;
  a8=arg[8]? arg[8][0] : 0;
  a9=arg[2]? arg[2][8] : 0;
  a9=(a8*a9);
  a10=arg[3]? arg[3][0] : 0;
  a11=arg[4]? arg[4][0] : 0;
  a12=(a10*a11);
  a9=(a9/a12);
  a7=(a7+a9);
  a7=(a6*a7);
  a5=(a5-a7);
  a0=(a0-a5);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a7=arg[0]? arg[0][1] : 0;
  a4=(a4*a7);
  a7=arg[2]? arg[2][1] : 0;
  a9=arg[7]? arg[7][0] : 0;
  a12=arg[2]? arg[2][7] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a7=(a7+a12);
  a6=(a6*a7);
  a4=(a4-a6);
  a0=(a0-a4);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[1]? arg[1][2] : 0;
  a6=(a2*a3);
  a6=(a1+a6);
  a5=(a6*a5);
  a7=(a2*a3);
  a12=arg[2]? arg[2][9] : 0;
  a13=arg[2]? arg[2][17] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a7*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[1]? arg[1][3] : 0;
  a6=(a6*a4);
  a4=arg[2]? arg[2][10] : 0;
  a12=arg[2]? arg[2][16] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a4=(a4+a12);
  a7=(a7*a4);
  a6=(a6-a7);
  a0=(a0-a6);
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[1]? arg[1][4] : 0;
  a7=(a2*a3);
  a7=(a1+a7);
  a5=(a7*a5);
  a4=(a2*a3);
  a12=arg[2]? arg[2][18] : 0;
  a13=arg[2]? arg[2][26] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a4*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[1]? arg[1][5] : 0;
  a7=(a7*a6);
  a6=arg[2]? arg[2][19] : 0;
  a12=arg[2]? arg[2][25] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a6=(a6+a12);
  a4=(a4*a6);
  a7=(a7-a4);
  a0=(a0-a7);
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[1]? arg[1][6] : 0;
  a4=(a2*a3);
  a4=(a1+a4);
  a5=(a4*a5);
  a6=(a2*a3);
  a12=arg[2]? arg[2][27] : 0;
  a13=arg[2]? arg[2][35] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a6*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][6]=a0;
  a0=arg[1]? arg[1][7] : 0;
  a4=(a4*a7);
  a7=arg[2]? arg[2][28] : 0;
  a12=arg[2]? arg[2][34] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a7=(a7+a12);
  a6=(a6*a7);
  a4=(a4-a6);
  a0=(a0-a4);
  if (res[0]!=0) res[0][7]=a0;
  a0=arg[1]? arg[1][8] : 0;
  a6=(a2*a3);
  a6=(a1+a6);
  a5=(a6*a5);
  a7=(a2*a3);
  a12=arg[2]? arg[2][36] : 0;
  a13=arg[2]? arg[2][44] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a7*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][8]=a0;
  a0=arg[1]? arg[1][9] : 0;
  a6=(a6*a4);
  a4=arg[2]? arg[2][37] : 0;
  a12=arg[2]? arg[2][43] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a4=(a4+a12);
  a7=(a7*a4);
  a6=(a6-a7);
  a0=(a0-a6);
  if (res[0]!=0) res[0][9]=a0;
  a0=arg[1]? arg[1][10] : 0;
  a7=(a2*a3);
  a7=(a1+a7);
  a5=(a7*a5);
  a4=(a2*a3);
  a12=arg[2]? arg[2][45] : 0;
  a13=arg[2]? arg[2][53] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a4*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][10]=a0;
  a0=arg[1]? arg[1][11] : 0;
  a7=(a7*a6);
  a6=arg[2]? arg[2][46] : 0;
  a12=arg[2]? arg[2][52] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a6=(a6+a12);
  a4=(a4*a6);
  a7=(a7-a4);
  a0=(a0-a7);
  if (res[0]!=0) res[0][11]=a0;
  a0=arg[1]? arg[1][12] : 0;
  a4=(a2*a3);
  a4=(a1+a4);
  a5=(a4*a5);
  a6=(a2*a3);
  a12=arg[2]? arg[2][54] : 0;
  a13=arg[2]? arg[2][62] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a6*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][12]=a0;
  a0=arg[1]? arg[1][13] : 0;
  a4=(a4*a7);
  a7=arg[2]? arg[2][55] : 0;
  a12=arg[2]? arg[2][61] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a7=(a7+a12);
  a6=(a6*a7);
  a4=(a4-a6);
  a0=(a0-a4);
  if (res[0]!=0) res[0][13]=a0;
  a0=arg[1]? arg[1][14] : 0;
  a6=(a2*a3);
  a6=(a1+a6);
  a5=(a6*a5);
  a7=(a2*a3);
  a12=arg[2]? arg[2][63] : 0;
  a13=arg[2]? arg[2][71] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a7*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][14]=a0;
  a0=arg[1]? arg[1][15] : 0;
  a6=(a6*a4);
  a4=arg[2]? arg[2][64] : 0;
  a12=arg[2]? arg[2][70] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a4=(a4+a12);
  a7=(a7*a4);
  a6=(a6-a7);
  a0=(a0-a6);
  if (res[0]!=0) res[0][15]=a0;
  a0=arg[1]? arg[1][16] : 0;
  a7=(a2*a3);
  a7=(a1+a7);
  a5=(a7*a5);
  a4=(a2*a3);
  a12=arg[2]? arg[2][72] : 0;
  a13=arg[2]? arg[2][80] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a4*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][16]=a0;
  a0=arg[1]? arg[1][17] : 0;
  a7=(a7*a6);
  a6=arg[2]? arg[2][73] : 0;
  a12=arg[2]? arg[2][79] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a6=(a6+a12);
  a4=(a4*a6);
  a7=(a7-a4);
  a0=(a0-a7);
  if (res[0]!=0) res[0][17]=a0;
  a0=arg[1]? arg[1][18] : 0;
  a4=(a2*a3);
  a4=(a1+a4);
  a5=(a4*a5);
  a6=(a2*a3);
  a12=arg[2]? arg[2][81] : 0;
  a13=arg[2]? arg[2][89] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a6*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][18]=a0;
  a0=arg[1]? arg[1][19] : 0;
  a4=(a4*a7);
  a7=arg[2]? arg[2][82] : 0;
  a12=arg[2]? arg[2][88] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a7=(a7+a12);
  a6=(a6*a7);
  a4=(a4-a6);
  a0=(a0-a4);
  if (res[0]!=0) res[0][19]=a0;
  a0=arg[1]? arg[1][20] : 0;
  a6=(a2*a3);
  a6=(a1+a6);
  a5=(a6*a5);
  a7=(a2*a3);
  a12=arg[2]? arg[2][90] : 0;
  a13=arg[2]? arg[2][98] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a7*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][20]=a0;
  a0=arg[1]? arg[1][21] : 0;
  a6=(a6*a4);
  a4=arg[2]? arg[2][91] : 0;
  a12=arg[2]? arg[2][97] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a4=(a4+a12);
  a7=(a7*a4);
  a6=(a6-a7);
  a0=(a0-a6);
  if (res[0]!=0) res[0][21]=a0;
  a0=arg[1]? arg[1][22] : 0;
  a7=(a2*a3);
  a7=(a1+a7);
  a5=(a7*a5);
  a4=(a2*a3);
  a12=arg[2]? arg[2][99] : 0;
  a13=arg[2]? arg[2][107] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a4*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][22]=a0;
  a0=arg[1]? arg[1][23] : 0;
  a7=(a7*a6);
  a6=arg[2]? arg[2][100] : 0;
  a12=arg[2]? arg[2][106] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a6=(a6+a12);
  a4=(a4*a6);
  a7=(a7-a4);
  a0=(a0-a7);
  if (res[0]!=0) res[0][23]=a0;
  a0=arg[1]? arg[1][24] : 0;
  a4=(a2*a3);
  a4=(a1+a4);
  a5=(a4*a5);
  a6=(a2*a3);
  a12=arg[2]? arg[2][108] : 0;
  a13=arg[2]? arg[2][116] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a6*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][24]=a0;
  a0=arg[1]? arg[1][25] : 0;
  a4=(a4*a7);
  a7=arg[2]? arg[2][109] : 0;
  a12=arg[2]? arg[2][115] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a7=(a7+a12);
  a6=(a6*a7);
  a4=(a4-a6);
  a0=(a0-a4);
  if (res[0]!=0) res[0][25]=a0;
  a0=arg[1]? arg[1][26] : 0;
  a6=(a2*a3);
  a6=(a1+a6);
  a5=(a6*a5);
  a7=(a2*a3);
  a12=arg[2]? arg[2][117] : 0;
  a13=arg[2]? arg[2][125] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a7*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][26]=a0;
  a0=arg[1]? arg[1][27] : 0;
  a6=(a6*a4);
  a4=arg[2]? arg[2][118] : 0;
  a12=arg[2]? arg[2][124] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a4=(a4+a12);
  a7=(a7*a4);
  a6=(a6-a7);
  a0=(a0-a6);
  if (res[0]!=0) res[0][27]=a0;
  a0=arg[1]? arg[1][28] : 0;
  a7=(a2*a3);
  a7=(a1+a7);
  a5=(a7*a5);
  a4=(a2*a3);
  a12=arg[2]? arg[2][126] : 0;
  a13=arg[2]? arg[2][134] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a4*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][28]=a0;
  a0=arg[1]? arg[1][29] : 0;
  a7=(a7*a6);
  a6=arg[2]? arg[2][127] : 0;
  a12=arg[2]? arg[2][133] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a6=(a6+a12);
  a4=(a4*a6);
  a7=(a7-a4);
  a0=(a0-a7);
  if (res[0]!=0) res[0][29]=a0;
  a0=arg[1]? arg[1][30] : 0;
  a4=(a2*a3);
  a4=(a1+a4);
  a5=(a4*a5);
  a6=(a2*a3);
  a12=arg[2]? arg[2][135] : 0;
  a13=arg[2]? arg[2][143] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a6*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][30]=a0;
  a0=arg[1]? arg[1][31] : 0;
  a4=(a4*a7);
  a7=arg[2]? arg[2][136] : 0;
  a12=arg[2]? arg[2][142] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a7=(a7+a12);
  a6=(a6*a7);
  a4=(a4-a6);
  a0=(a0-a4);
  if (res[0]!=0) res[0][31]=a0;
  a0=arg[1]? arg[1][32] : 0;
  a6=(a2*a3);
  a6=(a1+a6);
  a5=(a6*a5);
  a7=(a2*a3);
  a12=arg[2]? arg[2][144] : 0;
  a13=arg[2]? arg[2][152] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a7*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][32]=a0;
  a0=arg[1]? arg[1][33] : 0;
  a6=(a6*a4);
  a4=arg[2]? arg[2][145] : 0;
  a12=arg[2]? arg[2][151] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a4=(a4+a12);
  a7=(a7*a4);
  a6=(a6-a7);
  a0=(a0-a6);
  if (res[0]!=0) res[0][33]=a0;
  a0=arg[1]? arg[1][34] : 0;
  a7=(a2*a3);
  a7=(a1+a7);
  a5=(a7*a5);
  a4=(a2*a3);
  a12=arg[2]? arg[2][153] : 0;
  a13=arg[2]? arg[2][161] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a4*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][34]=a0;
  a0=arg[1]? arg[1][35] : 0;
  a7=(a7*a6);
  a6=arg[2]? arg[2][154] : 0;
  a12=arg[2]? arg[2][160] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a6=(a6+a12);
  a4=(a4*a6);
  a7=(a7-a4);
  a0=(a0-a7);
  if (res[0]!=0) res[0][35]=a0;
  a0=arg[1]? arg[1][36] : 0;
  a4=(a2*a3);
  a4=(a1+a4);
  a5=(a4*a5);
  a6=(a2*a3);
  a12=arg[2]? arg[2][162] : 0;
  a13=arg[2]? arg[2][170] : 0;
  a13=(a8*a13);
  a14=(a10*a11);
  a13=(a13/a14);
  a12=(a12+a13);
  a12=(a6*a12);
  a5=(a5-a12);
  a0=(a0-a5);
  if (res[0]!=0) res[0][36]=a0;
  a0=arg[1]? arg[1][37] : 0;
  a4=(a4*a7);
  a7=arg[2]? arg[2][163] : 0;
  a12=arg[2]? arg[2][169] : 0;
  a12=(a9*a12);
  a13=(a10*a11);
  a12=(a12/a13);
  a7=(a7+a12);
  a6=(a6*a7);
  a4=(a4-a6);
  a0=(a0-a4);
  if (res[0]!=0) res[0][37]=a0;
  a0=arg[1]? arg[1][38] : 0;
  a6=(a2*a3);
  a1=(a1+a6);
  a5=(a1*a5);
  a2=(a2*a3);
  a3=arg[2]? arg[2][171] : 0;
  a6=arg[2]? arg[2][179] : 0;
  a8=(a8*a6);
  a6=(a10*a11);
  a8=(a8/a6);
  a3=(a3+a8);
  a3=(a2*a3);
  a5=(a5-a3);
  a0=(a0-a5);
  if (res[0]!=0) res[0][38]=a0;
  a0=arg[1]? arg[1][39] : 0;
  a1=(a1*a4);
  a4=arg[2]? arg[2][172] : 0;
  a5=arg[2]? arg[2][178] : 0;
  a9=(a9*a5);
  a10=(a10*a11);
  a9=(a9/a10);
  a4=(a4+a9);
  a2=(a2*a4);
  a1=(a1-a2);
  a0=(a0-a1);
  if (res[0]!=0) res[0][39]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int ceq0_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int ceq0_func_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int ceq0_func_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ceq0_func_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int ceq0_func_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void ceq0_func_release(int mem) {
}

CASADI_SYMBOL_EXPORT void ceq0_func_incref(void) {
}

CASADI_SYMBOL_EXPORT void ceq0_func_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int ceq0_func_n_in(void) { return 9;}

CASADI_SYMBOL_EXPORT casadi_int ceq0_func_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real ceq0_func_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ceq0_func_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    case 6: return "i6";
    case 7: return "i7";
    case 8: return "i8";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* ceq0_func_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ceq0_func_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s3;
    case 5: return casadi_s3;
    case 6: return casadi_s3;
    case 7: return casadi_s3;
    case 8: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* ceq0_func_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int ceq0_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 9;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

#ifdef MATLAB_MEX_FILE
void mex_ceq0_func(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  casadi_int i;
  casadi_real w[448];
  casadi_int *iw = 0;
  const casadi_real* arg[9] = {0};
  casadi_real* res[1] = {0};
  if (argc>9) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"ceq0_func\" failed. Too many input arguments (%d, max 9)", argc);
  if (resc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"ceq0_func\" failed. Too many output arguments (%d, max 1)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+268);
  if (--argc>=0) arg[1] = casadi_from_mex(argv[1], w+2, casadi_s1, w+268);
  if (--argc>=0) arg[2] = casadi_from_mex(argv[2], w+42, casadi_s2, w+268);
  if (--argc>=0) arg[3] = casadi_from_mex(argv[3], w+222, casadi_s3, w+268);
  if (--argc>=0) arg[4] = casadi_from_mex(argv[4], w+223, casadi_s3, w+268);
  if (--argc>=0) arg[5] = casadi_from_mex(argv[5], w+224, casadi_s3, w+268);
  if (--argc>=0) arg[6] = casadi_from_mex(argv[6], w+225, casadi_s3, w+268);
  if (--argc>=0) arg[7] = casadi_from_mex(argv[7], w+226, casadi_s3, w+268);
  if (--argc>=0) arg[8] = casadi_from_mex(argv[8], w+227, casadi_s3, w+268);
  --resc;
  res[0] = w+228;
  i = ceq0_func(arg, res, iw, w+268, 0);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"ceq0_func\" failed.");
  if (res[0]) resv[0] = casadi_to_mex(casadi_s1, res[0]);
}
#endif

casadi_int main_ceq0_func(casadi_int argc, char* argv[]) {
  casadi_int j;
  casadi_real* a;
  const casadi_real* r;
  casadi_int flag;
  casadi_int *iw = 0;
  casadi_real w[283];
  const casadi_real* arg[9];
  casadi_real* res[1];
  arg[0] = w+0;
  arg[1] = w+2;
  arg[2] = w+42;
  arg[3] = w+222;
  arg[4] = w+223;
  arg[5] = w+224;
  arg[6] = w+225;
  arg[7] = w+226;
  arg[8] = w+227;
  res[0] = w+228;
  a = w;
  for (j=0; j<228; ++j) if (scanf("%lg", a++)<=0) return 2;
  flag = ceq0_func(arg, res, iw, w+268, 0);
  if (flag) return flag;
  r = w+228;
  for (j=0; j<40; ++j) CASADI_PRINTF("%g ", *r++);
  CASADI_PRINTF("\n");
  return 0;
}


#ifdef MATLAB_MEX_FILE
void mexFunction(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  char buf[10];
  int buf_ok = argc > 0 && !mxGetString(*argv, buf, sizeof(buf));
  if (!buf_ok) {
    mex_ceq0_func(resc, resv, argc, argv);
    return;
  } else if (strcmp(buf, "ceq0_func")==0) {
    mex_ceq0_func(resc, resv, argc-1, argv+1);
    return;
  }
  mexErrMsgTxt("First input should be a command string. Possible values: 'ceq0_func'");
}
#endif
int main(int argc, char* argv[]) {
  if (argc<2) {
    /* name error */
  } else if (strcmp(argv[1], "ceq0_func")==0) {
    return main_ceq0_func(argc-2, argv+2);
  }
  fprintf(stderr, "First input should be a command string. Possible values: 'ceq0_func'\nNote: you may use function.generate_input to create a command string.\n");
  return 1;
}
#ifdef __cplusplus
} /* extern "C" */
#endif
