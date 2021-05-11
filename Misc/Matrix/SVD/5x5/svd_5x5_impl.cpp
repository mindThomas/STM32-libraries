//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:25:44
//

// Include Files
#include "svd_5x5_impl.h"
#include <MATLABCoderInit/rt_nonfinite.h>
#include <Matrix/SVD/sqrt.h>
#include "svd_5x5.h"
#include "xaxpy_5x5.h"
#include "xdotc_5x5.h"
#include "xnrm2_5x5.h"
#include "xrot_5x5.h"
#include "xrotg_5x5.h"
#include "xscal_5x5.h"
#include "xswap_5x5.h"
#include <math.h>
#include <string.h>

// Function Definitions

//
// Arguments    : const float A[25]
//                float U[25]
//                float s[5]
//                float V[25]
// Return Type  : void
//
void b_svd_5x5(const float A[25], float U[25], float s[5], float V[25])
{
    float     b_A[25];
    int       i;
    float     b_s[5];
    float     e[5];
    int       q;
    float     work[5];
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
    memcpy(&b_A[0], &A[0], 25U * sizeof(float));
    for (i = 0; i < 5; i++) {
        b_s[i]  = 0.0F;
        e[i]    = 0.0F;
        work[i] = 0.0F;
    }

    memset(&U[0], 0, 25U * sizeof(float));
    memset(&V[0], 0, 25U * sizeof(float));
    for (q = 0; q < 4; q++) {
        qq              = q + 5 * q;
        apply_transform = false;
        nrm             = xnrm2_5x5(5 - q, b_A, qq + 1);
        if (nrm > 0.0F) {
            apply_transform = true;
            if (b_A[qq] < 0.0F) {
                b_s[q] = -nrm;
            } else {
                b_s[q] = nrm;
            }

            if ((float)fabs((double)b_s[q]) >= 9.86076132E-32F) {
                nrm = 1.0F / b_s[q];
                i   = (qq - q) + 5;
                for (qp1jj = qq; qp1jj < i; qp1jj++) {
                    b_A[qp1jj] *= nrm;
                }
            } else {
                i = (qq - q) + 5;
                for (qp1jj = qq; qp1jj < i; qp1jj++) {
                    b_A[qp1jj] /= b_s[q];
                }
            }

            b_A[qq]++;
            b_s[q] = -b_s[q];
        } else {
            b_s[q] = 0.0F;
        }

        for (qs = q + 1; qs + 1 < 6; qs++) {
            i = q + 5 * qs;
            if (apply_transform) {
                xaxpy_5x5(5 - q, -(xdotc_5x5(5 - q, b_A, qq + 1, b_A, i + 1) / b_A[q + 5 * q]), qq + 1, b_A, i + 1);
            }

            e[qs] = b_A[i];
        }

        for (qp1jj = q; qp1jj + 1 < 6; qp1jj++) {
            U[qp1jj + 5 * q] = b_A[qp1jj + 5 * q];
        }

        if (q + 1 <= 3) {
            nrm = b_xnrm2_5x5(4 - q, e, q + 2);
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
                    for (qp1jj = q + 1; qp1jj < 5; qp1jj++) {
                        e[qp1jj] *= nrm;
                    }
                } else {
                    for (qp1jj = q + 1; qp1jj < 5; qp1jj++) {
                        e[qp1jj] /= nrm;
                    }
                }

                e[q + 1]++;
                e[q] = -e[q];
                for (qp1jj = q + 1; qp1jj + 1 < 6; qp1jj++) {
                    work[qp1jj] = 0.0F;
                }

                for (qs = q + 1; qs + 1 < 6; qs++) {
                    b_xaxpy_5x5(4 - q, e[qs], b_A, (q + 5 * qs) + 2, work, q + 2);
                }

                for (qs = q + 1; qs + 1 < 6; qs++) {
                    c_xaxpy_5x5(4 - q, -e[qs] / e[q + 1], work, q + 2, b_A, (q + 5 * qs) + 2);
                }
            }

            for (qp1jj = q + 1; qp1jj + 1 < 6; qp1jj++) {
                V[qp1jj + 5 * q] = e[qp1jj];
            }
        }
    }

    m      = 3;
    b_s[4] = b_A[24];
    e[3]   = b_A[23];
    e[4]   = 0.0F;
    for (qp1jj = 0; qp1jj < 5; qp1jj++) {
        U[20 + qp1jj] = 0.0F;
    }

    U[24] = 1.0F;
    for (q = 3; q >= 0; q--) {
        qq = q + 5 * q;
        if (b_s[q] != 0.0F) {
            for (qs = q + 1; qs + 1 < 6; qs++) {
                i = (q + 5 * qs) + 1;
                xaxpy_5x5(5 - q, -(xdotc_5x5(5 - q, U, qq + 1, U, i) / U[qq]), qq + 1, U, i);
            }

            for (qp1jj = q; qp1jj + 1 < 6; qp1jj++) {
                U[qp1jj + 5 * q] = -U[qp1jj + 5 * q];
            }

            U[qq]++;
            for (qp1jj = 1; qp1jj <= q; qp1jj++) {
                U[(qp1jj + 5 * q) - 1] = 0.0F;
            }
        } else {
            for (qp1jj = 0; qp1jj < 5; qp1jj++) {
                U[qp1jj + 5 * q] = 0.0F;
            }

            U[qq] = 1.0F;
        }
    }

    for (q = 4; q >= 0; q--) {
        if ((q + 1 <= 3) && (e[q] != 0.0F)) {
            i = (q + 5 * q) + 2;
            for (qs = q + 1; qs + 1 < 6; qs++) {
                qp1jj = (q + 5 * qs) + 2;
                xaxpy_5x5(4 - q, -(xdotc_5x5(4 - q, V, i, V, qp1jj) / V[i - 1]), i, V, qp1jj);
            }
        }

        for (qp1jj = 0; qp1jj < 5; qp1jj++) {
            V[qp1jj + 5 * q] = 0.0F;
        }

        V[q + 5 * q] = 1.0F;
    }

    for (q = 0; q < 5; q++) {
        nrm = e[q];
        if (b_s[q] != 0.0F) {
            rt     = (float)fabs((double)b_s[q]);
            r      = b_s[q] / rt;
            b_s[q] = rt;
            if (q + 1 < 5) {
                nrm = e[q] / r;
            }

            xscal_5x5(r, U, 1 + 5 * q);
        }

        if ((q + 1 < 5) && (nrm != 0.0F)) {
            rt  = (float)fabs((double)nrm);
            r   = rt / nrm;
            nrm = rt;
            b_s[q + 1] *= r;
            xscal_5x5(r, V, 1 + 5 * (q + 1));
        }

        e[q] = nrm;
    }

    qq    = 0;
    snorm = 0.0F;
    for (qp1jj = 0; qp1jj < 5; qp1jj++) {
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
                    xrotg_5x5(&b_s[qp1jj], &f, &nrm, &r);
                    if (qp1jj + 1 > q + 1) {
                        f = -r * e[qp1jj - 1];
                        e[qp1jj - 1] *= nrm;
                    }

                    xrot_5x5(V, 1 + 5 * qp1jj, 1 + 5 * (m + 1), nrm, r);
                }
                break;

            case 2:
                f        = e[q - 1];
                e[q - 1] = 0.0F;
                for (qp1jj = q; qp1jj < m + 2; qp1jj++) {
                    xrotg_5x5(&b_s[qp1jj], &f, &nrm, &r);
                    f = -r * e[qp1jj];
                    e[qp1jj] *= nrm;
                    xrot_5x5(U, 1 + 5 * qp1jj, 1 + 5 * (q - 1), nrm, r);
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
                    xrotg_5x5(&f, &rt, &nrm, &r);
                    if (qp1jj > q + 1) {
                        e[qp1jj - 2] = f;
                    }

                    f            = nrm * b_s[qp1jj - 1] + r * e[qp1jj - 1];
                    e[qp1jj - 1] = nrm * e[qp1jj - 1] - r * b_s[qp1jj - 1];
                    rt           = r * b_s[qp1jj];
                    b_s[qp1jj] *= nrm;
                    xrot_5x5(V, 1 + 5 * (qp1jj - 1), 1 + 5 * qp1jj, nrm, r);
                    b_s[qp1jj - 1] = f;
                    xrotg_5x5(&b_s[qp1jj - 1], &rt, &nrm, &r);
                    f          = nrm * e[qp1jj - 1] + r * b_s[qp1jj];
                    b_s[qp1jj] = -r * e[qp1jj - 1] + nrm * b_s[qp1jj];
                    rt         = r * e[qp1jj];
                    e[qp1jj] *= nrm;
                    xrot_5x5(U, 1 + 5 * (qp1jj - 1), 1 + 5 * qp1jj, nrm, r);
                }

                e[m] = f;
                qq++;
                break;

            default:
                if (b_s[q] < 0.0F) {
                    b_s[q] = -b_s[q];
                    xscal_5x5(-1.0F, V, 1 + 5 * q);
                }

                i = q + 1;
                while ((q + 1 < 5) && (b_s[q] < b_s[i])) {
                    rt     = b_s[q];
                    b_s[q] = b_s[i];
                    b_s[i] = rt;
                    xswap_5x5(V, 1 + 5 * q, 1 + 5 * (q + 1));
                    xswap_5x5(U, 1 + 5 * q, 1 + 5 * (q + 1));
                    q = i;
                    i++;
                }

                qq = 0;
                m--;
                break;
        }
    }

    for (qp1jj = 0; qp1jj < 5; qp1jj++) {
        s[qp1jj] = b_s[qp1jj];
    }
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
