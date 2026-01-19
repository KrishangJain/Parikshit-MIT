#ifndef EKF_HPP
#define EKF_HPP

#define _USE_MATRIX_PRINT
#include "../new_lib/matrixMath.hpp"
#include <cmath>

Matrix skew3(const Matrix &v)
{
    float_t vx = v(0, 0), vy = v(1, 0), vz = v(2, 0);
    Matrix S({{0, -vz, vy},
              {vz, 0, -vx},
              {-vy, vx, 0}});
    return S;
}

static const Matrix zero;

void Prediction(
    float_t qn0, float_t qn1, float_t qn2, float_t qn3,
    float_t wnx, float_t wny, float_t wnz,
    const Matrix &Wk1, const Matrix &Wk2,
    const Matrix &I, const Matrix &Tor,
    const Matrix &Pn, const Matrix &B_true,
    const Matrix &noise_b, const Matrix &noise_w,
    Matrix &Q1_bar, Matrix &W1_bar, Matrix &Pn1,
    Matrix &B_meas, Matrix &O_meas, Matrix &omega_s)
{
    float_t h = 0.01f, delT = 0.01f;

    Matrix Omega({{0, -wnx, -wny, -wnz},
                  {wnx, 0, wnz, -wny},
                  {wny, -wnz, 0, wnx},
                  {wnz, wny, -wnx, 0}});

    Matrix q({{qn0}, {qn1}, {qn2}, {qn3}});

    Matrix k1 = (Omega * q) * 0.5f;
    Matrix k2 = (Omega * (q + (k1 * (h * 0.5f)))) * 0.5f;
    Matrix k3 = (Omega * (q + (k2 * (h * 0.5f)))) * 0.5f;
    Matrix k4 = (Omega * (q + (k3 * h))) * 0.5f;

    Q1_bar = q + ((k1 + (k2 * 2.0f) + (k3 * 2.0f) + k4) * (h / 6.0f)) + Wk1;

    if ((powf(Q1_bar.sum(2), 0.5f)) == 0.0f)
    {
        Q1_bar = W1_bar = Pn1 = B_meas = O_meas = omega_s = zero;
        return;
    }
    Q1_bar /= (powf(Q1_bar.sum(2), 0.5f));

    omega_s = Matrix({{wnx}, {wny}, {wnz}});

    if (I.det() == 0.0f)
    {
        Q1_bar = W1_bar = Pn1 = B_meas = O_meas = omega_s = zero;
        return;
    }
    Matrix Iinv = I.inverse();

    Matrix l1 = Iinv * (Tor - (skew3(omega_s) * (I * omega_s)));
    Matrix l2 = Iinv * (Tor - (skew3(omega_s + (l1 * (h * 0.5f))) * (I * (omega_s + (l1 * (h * 0.5f))))));
    Matrix l3 = Iinv * (Tor - (skew3(omega_s + (l2 * (h * 0.5f))) * (I * (omega_s + (l2 * (h * 0.5f))))));
    Matrix l4 = Iinv * (Tor - (skew3(omega_s + (l3 * h)) * (I * (omega_s + (l3 * h)))));

    W1_bar = omega_s + ((l1 + (l2 * 2.0f) + (l3 * 2.0f) + l4) * (h / 6.0f)) + Wk2;

    Matrix fq({{0, -wnx * 0.5f, -wny * 0.5f, -wnz * 0.5f},
               {wnx * 0.5f, 0, wnz * 0.5f, -wny * 0.5f},
               {wny * 0.5f, -wnz * 0.5f, 0, wnx * 0.5f},
               {wnz * 0.5f, wny * 0.5f, -wnx * 0.5f, 0}});

    Matrix fw({{-qn1 * 0.5f, -qn2 * 0.5f, -qn3 * 0.5f},
               {qn0 * 0.5f, -qn3 * 0.5f, qn2 * 0.5f},
               {qn3 * 0.5f, qn0 * 0.5f, -qn1 * 0.5f},
               {-qn2 * 0.5f, qn1 * 0.5f, qn0 * 0.5f}});

    Matrix Iomega_v = I * omega_s;

    Matrix Iomega_skew({{0, -Iomega_v(2, 0), Iomega_v(1, 0)},
                        {Iomega_v(2, 0), 0, -Iomega_v(0, 0)},
                        {-Iomega_v(1, 0), Iomega_v(0, 0), 0}});

    Matrix omega_skew = skew3(omega_s);

    Matrix gw = Iinv * (Iomega_skew - (omega_skew * I));

    Matrix F({{fq(0, 0), fq(0, 1), fq(0, 2), fq(0, 3), fw(0, 0), fw(0, 1), fw(0, 2)},
              {fq(1, 0), fq(1, 1), fq(1, 2), fq(1, 3), fw(1, 0), fw(1, 1), fw(1, 2)},
              {fq(2, 0), fq(2, 1), fq(2, 2), fq(2, 3), fw(2, 0), fw(2, 1), fw(2, 2)},
              {fq(3, 0), fq(3, 1), fq(3, 2), fq(3, 3), fw(3, 0), fw(3, 1), fw(3, 2)},
              {0.0f, 0.0f, 0.0f, 0.0f, gw(0, 0), gw(0, 1), gw(0, 2)},
              {0.0f, 0.0f, 0.0f, 0.0f, gw(1, 0), gw(1, 1), gw(1, 2)},
              {0.0f, 0.0f, 0.0f, 0.0f, gw(2, 0), gw(2, 1), gw(2, 2)}});

    Matrix Q({{0.1f, 0, 0, 0, 0, 0, 0},
              {0, 0.1f, 0, 0, 0, 0, 0},
              {0, 0, 0.1f, 0, 0, 0, 0},
              {0, 0, 0, 0.1f, 0, 0, 0},
              {0, 0, 0, 0, 0.01f, 0, 0},
              {0, 0, 0, 0, 0, 0.01f, 0},
              {0, 0, 0, 0, 0, 0, 0.01f}});

    Pn1 = Pn + ((F * Pn + Pn * F.trans() + Q) * delT);

    float_t q0 = qn0, q1 = qn1, q2 = qn2, q3 = qn3;
    Matrix Rq({{(2 * q0 * q0 - 1 + 2 * q1 * q1), (2 * q1 * q2 + 2 * q0 * q3), (2 * q1 * q3 - 2 * q0 * q2)},
               {(2 * q1 * q2 - 2 * q0 * q3), (2 * q0 * q0 - 1 + 2 * q2 * q2), (2 * q2 * q3 + 2 * q0 * q1)},
               {(2 * q1 * q3 + 2 * q0 * q2), (2 * q2 * q3 - 2 * q0 * q1), (2 * q0 * q0 - 1 + 2 * q3 * q3)}});

    B_meas = (Rq.trans() * B_true) + noise_b;
    O_meas = omega_s + noise_w;
}

void Measurement(
    float_t qn0, float_t qn1, float_t qn2, float_t qn3,
    float_t wnx, float_t wny, float_t wnz,
    float_t Bx_ig, float_t By_ig, float_t Bz_ig,
    const Matrix &B_meas, const Matrix &w_meas,
    const Matrix &Q1_bar, const Matrix &W1_bar,
    const Matrix &Pn1, const Matrix &Vk1, const Matrix &Vk2,
    Matrix &Qn1, Matrix &Wn1, Matrix &Pn)
{
    Pn = Matrix(7, 7);

    float_t q0 = qn0, q1 = qn1, q2 = qn2, q3 = qn3;
    float_t Bx = (2 * (q0 * q0) - 1 + 2 * (q1 * q1)) * Bx_ig + (2 * q1 * q2 - 2 * q0 * q3) * By_ig + (2 * q1 * q3 + 2 * q0 * q2) * Bz_ig;
    float_t By = (2 * q1 * q2 + 2 * q0 * q3) * Bx_ig + (2 * (q0 * q0) - 1 + 2 * (q2 * q2)) * By_ig + (2 * q2 * q3 - 2 * q0 * q1) * Bz_ig;
    float_t Bz = (2 * q1 * q3 - 2 * q0 * q2) * Bx_ig + (2 * q2 * q3 + 2 * q0 * q1) * By_ig + (2 * (q0 * q0) - 1 + 2 * (q3 * q3)) * Bz_ig;

    Matrix B_pred({{Bx}, {By}, {Bz}});

    Matrix B_pred_w = B_pred + Vk1;

    Matrix omega({{wnx}, {wny}, {wnz}});
    omega = omega + Vk2;

    Matrix Bq({{2 * (2 * q0 * Bx - q3 * By + q2 * Bz), 2 * (2 * q1 * Bx + q2 * By + q3 * Bz), 2 * (q1 * By + q0 * Bz), 2 * (q1 * By - q0 * Bz)},
               {2 * (q3 * Bx + 2 * q0 * By - q1 * Bz), 2 * (q2 * Bx - q0 * Bz), 2 * (q1 * Bx + 2 * q2 * By + q3 * Bz), 2 * (q2 * Bx + q0 * Bz)},
               {2 * (-q2 * Bx + q1 * By + 2 * q0 * Bz), 2 * (q3 * Bx + q0 * By), 2 * (-q0 * Bx + q3 * By), 2 * (q1 * Bx + q2 * By + 2 * q3 * Bz)}});

    Matrix H({{Bq(0, 0), Bq(0, 1), Bq(0, 2), Bq(0, 3), 0.0f, 0.0f, 0.0f},
              {Bq(1, 0), Bq(1, 1), Bq(1, 2), Bq(1, 3), 0.0f, 0.0f, 0.0f},
              {Bq(2, 0), Bq(2, 1), Bq(2, 2), Bq(2, 3), 0.0f, 0.0f, 0.0f},
              {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
              {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
              {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}});

    Matrix R({{0.2f, 0, 0, 0, 0, 0},
              {0, 0.2f, 0, 0, 0, 0},
              {0, 0, 0.2f, 0, 0, 0},
              {0, 0, 0, 0.01f, 0, 0},
              {0, 0, 0, 0, 0.01f, 0},
              {0, 0, 0, 0, 0, 0.01f}});

    Matrix S = H * Pn1 * H.trans() + R;

    if (S.det() == 0.0f)
    {
        Qn1 = Wn1 = Pn = zero;
        return;
    }

    Matrix K = Pn1 * H.trans() * S.inverse();

    Matrix yk(6, 1);

    Matrix tempB = B_meas - B_pred_w;
    Matrix tempW = w_meas - omega;
    for (uint8_t i = 0; i < 3; ++i)
        yk(i, 0) = tempB(i, 0);
    for (uint8_t i = 0; i < 3; ++i)
        yk(3 + i, 0) = tempW(i, 0);

    Matrix prior_x({{Q1_bar(0, 0)}, {Q1_bar(1, 0)}, {Q1_bar(2, 0)}, {Q1_bar(3, 0)}, {W1_bar(0, 0)}, {W1_bar(1, 0)}, {W1_bar(2, 0)}});

    Matrix corrected_x = prior_x + (K * yk);

    Qn1 = Matrix({{corrected_x(0, 0)},
                  {corrected_x(1, 0)},
                  {corrected_x(2, 0)},
                  {corrected_x(3, 0)}});

    if ((powf(Qn1.sum(2), 0.5f)) == 0.0f)
    {
        Qn1 = Wn1 = Pn = zero;
        return;
    }
    Qn1 /= powf(Qn1.sum(2), 0.5f);

    Wn1 = Matrix({{corrected_x(4, 0)},
                  {corrected_x(5, 0)},
                  {corrected_x(6, 0)}});

    Matrix I7(7, 7, 1.0f);

    Pn = (I7 - (K * H)) * Pn1;
}

#endif