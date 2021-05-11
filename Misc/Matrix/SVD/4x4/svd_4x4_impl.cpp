//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:15:43
//

// Include Files
#include "svd_4x4_impl.h"
#include <MATLABCoderInit/rt_nonfinite.h>
#include <Matrix/SVD/sqrt.h>
#include "svd_4x4.h"
#include "xaxpy_4x4.h"
#include "xdotc_4x4.h"
#include "xnrm2_4x4.h"
#include "xrot_4x4.h"
#include "xrotg_4x4.h"
#include "xscal_4x4.h"
#include "xswap_4x4.h"
#include <math.h>
#include <string.h>

// Function Definitions

//
// Arguments    : const float A[16]
//                float U[16]
//                float s[4]
//                float V[16]
// Return Type  : void
//
void b_svd_4x4(const float A[16], float U[16], float s[4], float V[16])
{
    float     b_A[16];
    int       i;
    float     b_s[4];
    float     e[4];
    int       q;
    float     work[4];
    int       m;
    int       qq;
    boolean_T apply_transform;
    float     nrm;
    int       qp1jj;
    int       qs;
    float     snorm;
    float     rt;
    float     r;
    int       exitg1;
    boolean_T exitg2;
    float     f;
    float     scale;
    float     sqds;
    memcpy(&b_A[0], &A[0], sizeof(float) << 4);
    for (i = 0; i < 4; i++) {
        b_s[i]  = 0.0F;
        e[i]    = 0.0F;
        work[i] = 0.0F;
    }

    memset(&U[0], 0, sizeof(float) << 4);
    memset(&V[0], 0, sizeof(float) << 4);
    for (q = 0; q < 3; q++) {
        qq              = q + (q << 2);
        apply_transform = false;
        nrm             = xnrm2_4x4(4 - q, b_A, qq + 1);
        if (nrm > 0.0F) {
            apply_transform = true;
            if (b_A[qq] < 0.0F) {
                b_s[q] = -nrm;
            } else {
                b_s[q] = nrm;
            }

            if ((float)fabs((double)b_s[q]) >= 9.86076132E-32F) {
                nrm = 1.0F / b_s[q];
                i   = (qq - q) + 4;
                for (qp1jj = qq; qp1jj < i; qp1jj++) {
                    b_A[qp1jj] *= nrm;
                }
            } else {
                i = (qq - q) + 4;
                for (qp1jj = qq; qp1jj < i; qp1jj++) {
                    b_A[qp1jj] /= b_s[q];
                }
            }

            b_A[qq]++;
            b_s[q] = -b_s[q];
        } else {
            b_s[q] = 0.0F;
        }

        for (qs = q + 1; qs + 1 < 5; qs++) {
            i = q + (qs << 2);
            if (apply_transform) {
                xaxpy_4x4(4 - q, -(xdotc_4x4(4 - q, b_A, qq + 1, b_A, i + 1) / b_A[q + (q << 2)]), qq + 1, b_A, i + 1);
            }

            e[qs] = b_A[i];
        }

        for (qp1jj = q; qp1jj + 1 < 5; qp1jj++) {
            U[qp1jj + (q << 2)] = b_A[qp1jj + (q << 2)];
        }

        if (q + 1 <= 2) {
            nrm = b_xnrm2_4x4(3 - q, e, q + 2);
            if (nrm == 0.0F) {
                e[q] = 0.0F;
            } else {
                if (e[q + 1] < 0.0F) {
                    e[q] = -nrm;
                } else {
                    e[q] = nrm;
                }

                nrm = e[q];
                if ((float)fabs((double)e[q]) >= 9.86076132E-32F) {
                    nrm = 1.0F / e[q];
                    for (qp1jj = q + 1; qp1jj < 4; qp1jj++) {
                        e[qp1jj] *= nrm;
                    }
                } else {
                    for (qp1jj = q + 1; qp1jj < 4; qp1jj++) {
                        e[qp1jj] /= nrm;
                    }
                }

                e[q + 1]++;
                e[q] = -e[q];
                for (qp1jj = q + 1; qp1jj + 1 < 5; qp1jj++) {
                    work[qp1jj] = 0.0F;
                }

                for (qs = q + 1; qs + 1 < 5; qs++) {
                    b_xaxpy_4x4(3 - q, e[qs], b_A, (q + (qs << 2)) + 2, work, q + 2);
                }

                for (qs = q + 1; qs + 1 < 5; qs++) {
                    c_xaxpy_4x4(3 - q, -e[qs] / e[q + 1], work, q + 2, b_A, (q + (qs << 2)) + 2);
                }
            }

            for (qp1jj = q + 1; qp1jj + 1 < 5; qp1jj++) {
                V[qp1jj + (q << 2)] = e[qp1jj];
            }
        }
    }

    m      = 2;
    b_s[3] = b_A[15];
    e[2]   = b_A[14];
    e[3]   = 0.0F;
    for (qp1jj = 0; qp1jj < 4; qp1jj++) {
        U[12 + qp1jj] = 0.0F;
    }

    U[15] = 1.0F;
    for (q = 2; q >= 0; q--) {
        qq = q + (q << 2);
        if (b_s[q] != 0.0F) {
            for (qs = q + 1; qs + 1 < 5; qs++) {
                i = (q + (qs << 2)) + 1;
                xaxpy_4x4(4 - q, -(xdotc_4x4(4 - q, U, qq + 1, U, i) / U[qq]), qq + 1, U, i);
            }

            for (qp1jj = q; qp1jj + 1 < 5; qp1jj++) {
                U[qp1jj + (q << 2)] = -U[qp1jj + (q << 2)];
            }

            U[qq]++;
            for (qp1jj = 1; qp1jj <= q; qp1jj++) {
                U[(qp1jj + (q << 2)) - 1] = 0.0F;
            }
        } else {
            for (qp1jj = 0; qp1jj < 4; qp1jj++) {
                U[qp1jj + (q << 2)] = 0.0F;
            }

            U[qq] = 1.0F;
        }
    }

    for (q = 3; q >= 0; q--) {
        if ((q + 1 <= 2) && (e[q] != 0.0F)) {
            i = (q + (q << 2)) + 2;
            for (qs = q + 1; qs + 1 < 5; qs++) {
                qp1jj = (q + (qs << 2)) + 2;
                xaxpy_4x4(3 - q, -(xdotc_4x4(3 - q, V, i, V, qp1jj) / V[i - 1]), i, V, qp1jj);
            }
        }

        for (qp1jj = 0; qp1jj < 4; qp1jj++) {
            V[qp1jj + (q << 2)] = 0.0F;
        }

        V[q + (q << 2)] = 1.0F;
    }

    for (q = 0; q < 4; q++) {
        nrm = e[q];
        if (b_s[q] != 0.0F) {
            rt     = (float)fabs((double)b_s[q]);
            r      = b_s[q] / rt;
            b_s[q] = rt;
            if (q + 1 < 4) {
                nrm = e[q] / r;
            }

            xscal_4x4(r, U, 1 + (q << 2));
        }

        if ((q + 1 < 4) && (nrm != 0.0F)) {
            rt  = (float)fabs((double)nrm);
            r   = rt / nrm;
            nrm = rt;
            b_s[q + 1] *= r;
            xscal_4x4(r, V, 1 + ((q + 1) << 2));
        }

        e[q] = nrm;
    }

    qq    = 0;
    snorm = 0.0F;
    for (qp1jj = 0; qp1jj < 4; qp1jj++) {
        nrm = (float)fabs((double)b_s[qp1jj]);
        r   = (float)fabs((double)e[qp1jj]);
        if ((nrm > r) || rtIsNaNF(r)) {
        } else {
            nrm = r;
        }

        if (!((snorm > nrm) || rtIsNaNF(nrm))) {
            snorm = nrm;
        }
    }

    while ((m + 2 > 0) && (!(qq >= 75))) {
        qp1jj = m;
        do {
            exitg1 = 0;
            q      = qp1jj + 1;
            if (qp1jj + 1 == 0) {
                exitg1 = 1;
            } else {
                nrm = (float)fabs((double)e[qp1jj]);
                if ((nrm <= 1.1920929E-7F * ((float)fabs((double)b_s[qp1jj]) + (float)fabs((double)b_s[qp1jj + 1]))) ||
                    (nrm <= 9.86076132E-32F) || ((qq > 20) && (nrm <= 1.1920929E-7F * snorm))) {
                    e[qp1jj] = 0.0F;
                    exitg1   = 1;
                } else {
                    qp1jj--;
                }
            }
        } while (exitg1 == 0);

        if (qp1jj + 1 == m + 1) {
            i = 4;
        } else {
            qs     = m + 2;
            i      = m + 2;
            exitg2 = false;
            while ((!exitg2) && (i >= qp1jj + 1)) {
                qs = i;
                if (i == qp1jj + 1) {
                    exitg2 = true;
                } else {
                    nrm = 0.0F;
                    if (i < m + 2) {
                        nrm = (float)fabs((double)e[i - 1]);
                    }

                    if (i > qp1jj + 2) {
                        nrm += (float)fabs((double)e[i - 2]);
                    }

                    r = (float)fabs((double)b_s[i - 1]);
                    if ((r <= 1.1920929E-7F * nrm) || (r <= 9.86076132E-32F)) {
                        b_s[i - 1] = 0.0F;
                        exitg2     = true;
                    } else {
                        i--;
                    }
                }
            }

            if (qs == qp1jj + 1) {
                i = 3;
            } else if (qs == m + 2) {
                i = 1;
            } else {
                i = 2;
                q = qs;
            }
        }

        switch (i) {
            case 1:
                f    = e[m];
                e[m] = 0.0F;
                for (qp1jj = m; qp1jj + 1 >= q + 1; qp1jj--) {
                    xrotg_4x4(&b_s[qp1jj], &f, &nrm, &r);
                    if (qp1jj + 1 > q + 1) {
                        f = -r * e[qp1jj - 1];
                        e[qp1jj - 1] *= nrm;
                    }

                    xrot_4x4(V, 1 + (qp1jj << 2), 1 + ((m + 1) << 2), nrm, r);
                }
                break;

            case 2:
                f        = e[q - 1];
                e[q - 1] = 0.0F;
                for (qp1jj = q; qp1jj < m + 2; qp1jj++) {
                    xrotg_4x4(&b_s[qp1jj], &f, &nrm, &r);
                    f = -r * e[qp1jj];
                    e[qp1jj] *= nrm;
                    xrot_4x4(U, 1 + (qp1jj << 2), 1 + ((q - 1) << 2), nrm, r);
                }
                break;

            case 3:
                scale = (float)fabs((double)b_s[m + 1]);
                r     = (float)fabs((double)b_s[m]);
                if (!((scale > r) || rtIsNaNF(r))) {
                    scale = r;
                }

                r = (float)fabs((double)e[m]);
                if (!((scale > r) || rtIsNaNF(r))) {
                    scale = r;
                }

                r = (float)fabs((double)b_s[q]);
                if (!((scale > r) || rtIsNaNF(r))) {
                    scale = r;
                }

                r = (float)fabs((double)e[q]);
                if (!((scale > r) || rtIsNaNF(r))) {
                    scale = r;
                }

                f    = b_s[m + 1] / scale;
                nrm  = b_s[m] / scale;
                r    = e[m] / scale;
                sqds = b_s[q] / scale;
                rt   = ((nrm + f) * (nrm - f) + r * r) / 2.0F;
                nrm  = f * r;
                nrm *= nrm;
                if ((rt != 0.0F) || (nrm != 0.0F)) {
                    r = rt * rt + nrm;
                    b_sqrt(&r);
                    if (rt < 0.0F) {
                        r = -r;
                    }

                    r = nrm / (rt + r);
                } else {
                    r = 0.0F;
                }

                f  = (sqds + f) * (sqds - f) + r;
                rt = sqds * (e[q] / scale);
                for (qp1jj = q + 1; qp1jj <= m + 1; qp1jj++) {
                    xrotg_4x4(&f, &rt, &nrm, &r);
                    if (qp1jj > q + 1) {
                        e[qp1jj - 2] = f;
                    }

                    f            = nrm * b_s[qp1jj - 1] + r * e[qp1jj - 1];
                    e[qp1jj - 1] = nrm * e[qp1jj - 1] - r * b_s[qp1jj - 1];
                    rt           = r * b_s[qp1jj];
                    b_s[qp1jj] *= nrm;
                    xrot_4x4(V, 1 + ((qp1jj - 1) << 2), 1 + (qp1jj << 2), nrm, r);
                    b_s[qp1jj - 1] = f;
                    xrotg_4x4(&b_s[qp1jj - 1], &rt, &nrm, &r);
                    f          = nrm * e[qp1jj - 1] + r * b_s[qp1jj];
                    b_s[qp1jj] = -r * e[qp1jj - 1] + nrm * b_s[qp1jj];
                    rt         = r * e[qp1jj];
                    e[qp1jj] *= nrm;
                    xrot_4x4(U, 1 + ((qp1jj - 1) << 2), 1 + (qp1jj << 2), nrm, r);
                }

                e[m] = f;
                qq++;
                break;

            default:
                if (b_s[q] < 0.0F) {
                    b_s[q] = -b_s[q];
                    xscal_4x4(-1.0F, V, 1 + (q << 2));
                }

                i = q + 1;
                while ((q + 1 < 4) && (b_s[q] < b_s[i])) {
                    rt     = b_s[q];
                    b_s[q] = b_s[i];
                    b_s[i] = rt;
                    xswap_4x4(V, 1 + (q << 2), 1 + ((q + 1) << 2));
                    xswap_4x4(U, 1 + (q << 2), 1 + ((q + 1) << 2));
                    q = i;
                    i++;
                }

                qq = 0;
                m--;
                break;
        }
    }

    for (qp1jj = 0; qp1jj < 4; qp1jj++) {
        s[qp1jj] = b_s[qp1jj];
    }
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
