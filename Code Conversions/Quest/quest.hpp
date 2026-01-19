#ifndef QUEST_H
#define QUEST_H
#include <cmath>
#define _USE_MATRIX_PRINT
#include "../new_lib/matrixMath.hpp"
using namespace std;
constexpr float_t radToDeg = 57.29577951308232f;

Matrix quest(Matrix B1, Matrix B2, Matrix R1, Matrix R2, float_t w1, float_t w2);
void partialAltProfMatrix(float_t const weight, Matrix const &bVec, Matrix const &rVec, Matrix &resMat);

Matrix quest(Matrix B1, Matrix B2, Matrix R1, Matrix R2, float_t w1, float_t w2)
{
    // Variables
    float_t sigma, a, b, c, d;
    float_t lamn, lama;
    float_t alpha, beta, g, y, E;
    float_t R11, R21, R32, R33, R13;
    float_t phi, theta, psi;
    Matrix B(3, 3, 0.0f);
    Matrix S(3, 3, 0.0f);
    Matrix t(3, 1, 0.0f);
    Matrix X(3, 1, 0.0f);

    // Normalise
    vector<Matrix *> vecs = {&B1, &B2, &R1, &R2};
    for (Matrix *v : vecs)
    {
        if ((sqrt(v->sum(2))) == 0.0f)
            return Matrix(0, 0);
        *v /= sqrt(v->sum(2));
    }

    // Attitude profile matrix
    B += B1 * R1.trans() * w1;
    B += B2 * R2.trans() * w2;

    // Different elements
    t(0, 0) = B(1, 2) - B(2, 1);
    t(1, 0) = B(2, 0) - B(0, 2);
    t(2, 0) = B(0, 1) - B(1, 0);

    S = B + B.trans();
    sigma = B.trace();

    a = sigma * sigma - S.adj().trace();
    b = sigma * sigma + (t.trans() * t)(0, 0);
    c = S.det() + ((t.trans() * (S * t))(0, 0));
    d = (t.trans() * (S * S * t))(0, 0);

    // Newton-Raphson Method for lambda
    lama = w1 + w2;
    lamn = 0.0f;
    while (abs(lama - lamn) > 0.000001)
    {
        lamn = lama;
        if ((4 * lama * lama * lama - 2 * (a + b) * lama - c) == 0.0f)
            return Matrix(0, 0);
        lama = lama - (lama * lama * lama * lama - (a + b) * lama * lama - c * lama + (a * b + c * sigma - d)) / (4 * lama * lama * lama - 2 * (a + b) * lama - c);
    }

    // Equations for Qopt
    alpha = lama * lama - sigma * sigma + S.adj().trace();
    beta = lama - sigma;
    y = (lama + sigma) * alpha - S.det();
    Matrix I(3, 3, 1.0f);
    X = ((I * alpha) + (S * beta) + (S * S)) * t;
    g = X(0, 0) * X(0, 0) + X(1, 0) * X(1, 0) + X(2, 0) * X(2, 0);

    if ((sqrt(y * y + g)) == 0.0f)
        return Matrix(0, 0);
    E = 1.0f / sqrt(y * y + g);

    Matrix P({{E * X(0, 0)},
              {E * X(1, 0)},
              {E * X(2, 0)},
              {E * y}});

    float_t qx = P(0, 0);
    float_t qy = P(1, 0);
    float_t qz = P(2, 0);
    float_t qw = P(3, 0);

    R11 = 1 - 2 * (qy * qy + qz * qz);
    R21 = 2 * (qx * qy + qw * qz);
    R32 = 2 * (qy * qz + qw * qx);
    R33 = 1 - 2 * (qx * qx + qy * qy);
    R13 = 2 * (qx * qz - qw * qy);

    // Euler angles
    phi = atan2(R32, R33) * radToDeg;
    theta = atan(-R13) * radToDeg;
    psi = atan2(R21, R11) * radToDeg;

#ifdef _USE_MATRIX_PRINT
    cout << "phi: " << phi << "\n";
    cout << "theta: " << theta << "\n";
    cout << "psi: " << psi << "\n\n";
#undef _USE_MATRIX_PRINT
#endif

    return P;
}

#endif