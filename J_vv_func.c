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
  #define CASADI_PREFIX(ID) J_vv_func_ ## ID
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

static const casadi_int casadi_s0[44] = {40, 1, 0, 40, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
static const casadi_int casadi_s1[184] = {180, 1, 0, 180, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179};
static const casadi_int casadi_s2[443] = {220, 220, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219};

/* J_vv_func:(i0[40],i1[180],i2[40],i3[40],i4[180])->(o0[220x220,220nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0;
  a0=arg[0]? arg[0][0] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][1] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][2] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[0]? arg[0][3] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[0]? arg[0][4] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[0]? arg[0][5] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[0]? arg[0][6] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][6]=a0;
  a0=arg[0]? arg[0][7] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][7]=a0;
  a0=arg[0]? arg[0][8] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][8]=a0;
  a0=arg[0]? arg[0][9] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][9]=a0;
  a0=arg[0]? arg[0][10] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][10]=a0;
  a0=arg[0]? arg[0][11] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][11]=a0;
  a0=arg[0]? arg[0][12] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][12]=a0;
  a0=arg[0]? arg[0][13] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][13]=a0;
  a0=arg[0]? arg[0][14] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][14]=a0;
  a0=arg[0]? arg[0][15] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][15]=a0;
  a0=arg[0]? arg[0][16] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][16]=a0;
  a0=arg[0]? arg[0][17] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][17]=a0;
  a0=arg[0]? arg[0][18] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][18]=a0;
  a0=arg[0]? arg[0][19] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][19]=a0;
  a0=arg[0]? arg[0][20] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][20]=a0;
  a0=arg[0]? arg[0][21] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][21]=a0;
  a0=arg[0]? arg[0][22] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][22]=a0;
  a0=arg[0]? arg[0][23] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][23]=a0;
  a0=arg[0]? arg[0][24] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][24]=a0;
  a0=arg[0]? arg[0][25] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][25]=a0;
  a0=arg[0]? arg[0][26] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][26]=a0;
  a0=arg[0]? arg[0][27] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][27]=a0;
  a0=arg[0]? arg[0][28] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][28]=a0;
  a0=arg[0]? arg[0][29] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][29]=a0;
  a0=arg[0]? arg[0][30] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][30]=a0;
  a0=arg[0]? arg[0][31] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][31]=a0;
  a0=arg[0]? arg[0][32] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][32]=a0;
  a0=arg[0]? arg[0][33] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][33]=a0;
  a0=arg[0]? arg[0][34] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][34]=a0;
  a0=arg[0]? arg[0][35] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][35]=a0;
  a0=arg[0]? arg[0][36] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][36]=a0;
  a0=arg[0]? arg[0][37] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][37]=a0;
  a0=arg[0]? arg[0][38] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][38]=a0;
  a0=arg[0]? arg[0][39] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][39]=a0;
  a0=arg[1]? arg[1][0] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][40]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][41]=a0;
  a0=arg[1]? arg[1][2] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][42]=a0;
  a0=arg[1]? arg[1][3] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][43]=a0;
  a0=arg[1]? arg[1][4] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][44]=a0;
  a0=arg[1]? arg[1][5] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][45]=a0;
  a0=arg[1]? arg[1][6] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][46]=a0;
  a0=arg[1]? arg[1][7] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][47]=a0;
  a0=arg[1]? arg[1][8] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][48]=a0;
  a0=arg[1]? arg[1][9] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][49]=a0;
  a0=arg[1]? arg[1][10] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][50]=a0;
  a0=arg[1]? arg[1][11] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][51]=a0;
  a0=arg[1]? arg[1][12] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][52]=a0;
  a0=arg[1]? arg[1][13] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][53]=a0;
  a0=arg[1]? arg[1][14] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][54]=a0;
  a0=arg[1]? arg[1][15] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][55]=a0;
  a0=arg[1]? arg[1][16] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][56]=a0;
  a0=arg[1]? arg[1][17] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][57]=a0;
  a0=arg[1]? arg[1][18] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][58]=a0;
  a0=arg[1]? arg[1][19] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][59]=a0;
  a0=arg[1]? arg[1][20] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][60]=a0;
  a0=arg[1]? arg[1][21] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][61]=a0;
  a0=arg[1]? arg[1][22] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][62]=a0;
  a0=arg[1]? arg[1][23] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][63]=a0;
  a0=arg[1]? arg[1][24] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][64]=a0;
  a0=arg[1]? arg[1][25] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][65]=a0;
  a0=arg[1]? arg[1][26] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][66]=a0;
  a0=arg[1]? arg[1][27] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][67]=a0;
  a0=arg[1]? arg[1][28] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][68]=a0;
  a0=arg[1]? arg[1][29] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][69]=a0;
  a0=arg[1]? arg[1][30] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][70]=a0;
  a0=arg[1]? arg[1][31] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][71]=a0;
  a0=arg[1]? arg[1][32] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][72]=a0;
  a0=arg[1]? arg[1][33] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][73]=a0;
  a0=arg[1]? arg[1][34] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][74]=a0;
  a0=arg[1]? arg[1][35] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][75]=a0;
  a0=arg[1]? arg[1][36] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][76]=a0;
  a0=arg[1]? arg[1][37] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][77]=a0;
  a0=arg[1]? arg[1][38] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][78]=a0;
  a0=arg[1]? arg[1][39] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][79]=a0;
  a0=arg[1]? arg[1][40] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][80]=a0;
  a0=arg[1]? arg[1][41] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][81]=a0;
  a0=arg[1]? arg[1][42] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][82]=a0;
  a0=arg[1]? arg[1][43] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][83]=a0;
  a0=arg[1]? arg[1][44] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][84]=a0;
  a0=arg[1]? arg[1][45] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][85]=a0;
  a0=arg[1]? arg[1][46] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][86]=a0;
  a0=arg[1]? arg[1][47] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][87]=a0;
  a0=arg[1]? arg[1][48] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][88]=a0;
  a0=arg[1]? arg[1][49] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][89]=a0;
  a0=arg[1]? arg[1][50] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][90]=a0;
  a0=arg[1]? arg[1][51] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][91]=a0;
  a0=arg[1]? arg[1][52] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][92]=a0;
  a0=arg[1]? arg[1][53] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][93]=a0;
  a0=arg[1]? arg[1][54] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][94]=a0;
  a0=arg[1]? arg[1][55] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][95]=a0;
  a0=arg[1]? arg[1][56] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][96]=a0;
  a0=arg[1]? arg[1][57] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][97]=a0;
  a0=arg[1]? arg[1][58] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][98]=a0;
  a0=arg[1]? arg[1][59] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][99]=a0;
  a0=arg[1]? arg[1][60] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][100]=a0;
  a0=arg[1]? arg[1][61] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][101]=a0;
  a0=arg[1]? arg[1][62] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][102]=a0;
  a0=arg[1]? arg[1][63] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][103]=a0;
  a0=arg[1]? arg[1][64] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][104]=a0;
  a0=arg[1]? arg[1][65] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][105]=a0;
  a0=arg[1]? arg[1][66] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][106]=a0;
  a0=arg[1]? arg[1][67] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][107]=a0;
  a0=arg[1]? arg[1][68] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][108]=a0;
  a0=arg[1]? arg[1][69] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][109]=a0;
  a0=arg[1]? arg[1][70] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][110]=a0;
  a0=arg[1]? arg[1][71] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][111]=a0;
  a0=arg[1]? arg[1][72] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][112]=a0;
  a0=arg[1]? arg[1][73] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][113]=a0;
  a0=arg[1]? arg[1][74] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][114]=a0;
  a0=arg[1]? arg[1][75] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][115]=a0;
  a0=arg[1]? arg[1][76] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][116]=a0;
  a0=arg[1]? arg[1][77] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][117]=a0;
  a0=arg[1]? arg[1][78] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][118]=a0;
  a0=arg[1]? arg[1][79] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][119]=a0;
  a0=arg[1]? arg[1][80] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][120]=a0;
  a0=arg[1]? arg[1][81] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][121]=a0;
  a0=arg[1]? arg[1][82] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][122]=a0;
  a0=arg[1]? arg[1][83] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][123]=a0;
  a0=arg[1]? arg[1][84] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][124]=a0;
  a0=arg[1]? arg[1][85] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][125]=a0;
  a0=arg[1]? arg[1][86] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][126]=a0;
  a0=arg[1]? arg[1][87] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][127]=a0;
  a0=arg[1]? arg[1][88] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][128]=a0;
  a0=arg[1]? arg[1][89] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][129]=a0;
  a0=arg[1]? arg[1][90] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][130]=a0;
  a0=arg[1]? arg[1][91] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][131]=a0;
  a0=arg[1]? arg[1][92] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][132]=a0;
  a0=arg[1]? arg[1][93] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][133]=a0;
  a0=arg[1]? arg[1][94] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][134]=a0;
  a0=arg[1]? arg[1][95] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][135]=a0;
  a0=arg[1]? arg[1][96] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][136]=a0;
  a0=arg[1]? arg[1][97] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][137]=a0;
  a0=arg[1]? arg[1][98] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][138]=a0;
  a0=arg[1]? arg[1][99] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][139]=a0;
  a0=arg[1]? arg[1][100] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][140]=a0;
  a0=arg[1]? arg[1][101] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][141]=a0;
  a0=arg[1]? arg[1][102] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][142]=a0;
  a0=arg[1]? arg[1][103] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][143]=a0;
  a0=arg[1]? arg[1][104] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][144]=a0;
  a0=arg[1]? arg[1][105] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][145]=a0;
  a0=arg[1]? arg[1][106] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][146]=a0;
  a0=arg[1]? arg[1][107] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][147]=a0;
  a0=arg[1]? arg[1][108] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][148]=a0;
  a0=arg[1]? arg[1][109] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][149]=a0;
  a0=arg[1]? arg[1][110] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][150]=a0;
  a0=arg[1]? arg[1][111] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][151]=a0;
  a0=arg[1]? arg[1][112] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][152]=a0;
  a0=arg[1]? arg[1][113] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][153]=a0;
  a0=arg[1]? arg[1][114] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][154]=a0;
  a0=arg[1]? arg[1][115] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][155]=a0;
  a0=arg[1]? arg[1][116] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][156]=a0;
  a0=arg[1]? arg[1][117] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][157]=a0;
  a0=arg[1]? arg[1][118] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][158]=a0;
  a0=arg[1]? arg[1][119] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][159]=a0;
  a0=arg[1]? arg[1][120] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][160]=a0;
  a0=arg[1]? arg[1][121] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][161]=a0;
  a0=arg[1]? arg[1][122] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][162]=a0;
  a0=arg[1]? arg[1][123] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][163]=a0;
  a0=arg[1]? arg[1][124] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][164]=a0;
  a0=arg[1]? arg[1][125] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][165]=a0;
  a0=arg[1]? arg[1][126] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][166]=a0;
  a0=arg[1]? arg[1][127] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][167]=a0;
  a0=arg[1]? arg[1][128] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][168]=a0;
  a0=arg[1]? arg[1][129] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][169]=a0;
  a0=arg[1]? arg[1][130] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][170]=a0;
  a0=arg[1]? arg[1][131] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][171]=a0;
  a0=arg[1]? arg[1][132] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][172]=a0;
  a0=arg[1]? arg[1][133] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][173]=a0;
  a0=arg[1]? arg[1][134] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][174]=a0;
  a0=arg[1]? arg[1][135] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][175]=a0;
  a0=arg[1]? arg[1][136] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][176]=a0;
  a0=arg[1]? arg[1][137] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][177]=a0;
  a0=arg[1]? arg[1][138] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][178]=a0;
  a0=arg[1]? arg[1][139] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][179]=a0;
  a0=arg[1]? arg[1][140] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][180]=a0;
  a0=arg[1]? arg[1][141] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][181]=a0;
  a0=arg[1]? arg[1][142] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][182]=a0;
  a0=arg[1]? arg[1][143] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][183]=a0;
  a0=arg[1]? arg[1][144] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][184]=a0;
  a0=arg[1]? arg[1][145] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][185]=a0;
  a0=arg[1]? arg[1][146] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][186]=a0;
  a0=arg[1]? arg[1][147] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][187]=a0;
  a0=arg[1]? arg[1][148] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][188]=a0;
  a0=arg[1]? arg[1][149] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][189]=a0;
  a0=arg[1]? arg[1][150] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][190]=a0;
  a0=arg[1]? arg[1][151] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][191]=a0;
  a0=arg[1]? arg[1][152] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][192]=a0;
  a0=arg[1]? arg[1][153] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][193]=a0;
  a0=arg[1]? arg[1][154] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][194]=a0;
  a0=arg[1]? arg[1][155] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][195]=a0;
  a0=arg[1]? arg[1][156] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][196]=a0;
  a0=arg[1]? arg[1][157] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][197]=a0;
  a0=arg[1]? arg[1][158] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][198]=a0;
  a0=arg[1]? arg[1][159] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][199]=a0;
  a0=arg[1]? arg[1][160] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][200]=a0;
  a0=arg[1]? arg[1][161] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][201]=a0;
  a0=arg[1]? arg[1][162] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][202]=a0;
  a0=arg[1]? arg[1][163] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][203]=a0;
  a0=arg[1]? arg[1][164] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][204]=a0;
  a0=arg[1]? arg[1][165] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][205]=a0;
  a0=arg[1]? arg[1][166] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][206]=a0;
  a0=arg[1]? arg[1][167] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][207]=a0;
  a0=arg[1]? arg[1][168] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][208]=a0;
  a0=arg[1]? arg[1][169] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][209]=a0;
  a0=arg[1]? arg[1][170] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][210]=a0;
  a0=arg[1]? arg[1][171] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][211]=a0;
  a0=arg[1]? arg[1][172] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][212]=a0;
  a0=arg[1]? arg[1][173] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][213]=a0;
  a0=arg[1]? arg[1][174] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][214]=a0;
  a0=arg[1]? arg[1][175] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][215]=a0;
  a0=arg[1]? arg[1][176] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][216]=a0;
  a0=arg[1]? arg[1][177] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][217]=a0;
  a0=arg[1]? arg[1][178] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][218]=a0;
  a0=arg[1]? arg[1][179] : 0;
  a0=(a0+a0);
  if (res[0]!=0) res[0][219]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int J_vv_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int J_vv_func_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int J_vv_func_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void J_vv_func_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int J_vv_func_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void J_vv_func_release(int mem) {
}

CASADI_SYMBOL_EXPORT void J_vv_func_incref(void) {
}

CASADI_SYMBOL_EXPORT void J_vv_func_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int J_vv_func_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int J_vv_func_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real J_vv_func_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* J_vv_func_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* J_vv_func_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* J_vv_func_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s0;
    case 3: return casadi_s0;
    case 4: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* J_vv_func_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int J_vv_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

#ifdef MATLAB_MEX_FILE
void mex_J_vv_func(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  casadi_int i;
  casadi_real w[880];
  casadi_int *iw = 0;
  const casadi_real* arg[5] = {0};
  casadi_real* res[1] = {0};
  if (argc>5) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"J_vv_func\" failed. Too many input arguments (%d, max 5)", argc);
  if (resc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"J_vv_func\" failed. Too many output arguments (%d, max 1)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+700);
  if (--argc>=0) arg[1] = casadi_from_mex(argv[1], w+40, casadi_s1, w+700);
  if (--argc>=0) arg[2] = casadi_from_mex(argv[2], w+220, casadi_s0, w+700);
  if (--argc>=0) arg[3] = casadi_from_mex(argv[3], w+260, casadi_s0, w+700);
  if (--argc>=0) arg[4] = casadi_from_mex(argv[4], w+300, casadi_s1, w+700);
  --resc;
  res[0] = w+480;
  i = J_vv_func(arg, res, iw, w+700, 0);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"J_vv_func\" failed.");
  if (res[0]) resv[0] = casadi_to_mex(casadi_s2, res[0]);
}
#endif

casadi_int main_J_vv_func(casadi_int argc, char* argv[]) {
  casadi_int j;
  casadi_real* a;
  const casadi_real* r;
  casadi_int flag;
  casadi_int *iw = 0;
  casadi_real w[701];
  const casadi_real* arg[5];
  casadi_real* res[1];
  arg[0] = w+0;
  arg[1] = w+40;
  arg[2] = w+220;
  arg[3] = w+260;
  arg[4] = w+300;
  res[0] = w+480;
  a = w;
  for (j=0; j<480; ++j) if (scanf("%lg", a++)<=0) return 2;
  flag = J_vv_func(arg, res, iw, w+700, 0);
  if (flag) return flag;
  r = w+480;
  for (j=0; j<220; ++j) CASADI_PRINTF("%g ", *r++);
  CASADI_PRINTF("\n");
  return 0;
}


#ifdef MATLAB_MEX_FILE
void mexFunction(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  char buf[10];
  int buf_ok = argc > 0 && !mxGetString(*argv, buf, sizeof(buf));
  if (!buf_ok) {
    mex_J_vv_func(resc, resv, argc, argv);
    return;
  } else if (strcmp(buf, "J_vv_func")==0) {
    mex_J_vv_func(resc, resv, argc-1, argv+1);
    return;
  }
  mexErrMsgTxt("First input should be a command string. Possible values: 'J_vv_func'");
}
#endif
int main(int argc, char* argv[]) {
  if (argc<2) {
    /* name error */
  } else if (strcmp(argv[1], "J_vv_func")==0) {
    return main_J_vv_func(argc-2, argv+2);
  }
  fprintf(stderr, "First input should be a command string. Possible values: 'J_vv_func'\nNote: you may use function.generate_input to create a command string.\n");
  return 1;
}
#ifdef __cplusplus
} /* extern "C" */
#endif
