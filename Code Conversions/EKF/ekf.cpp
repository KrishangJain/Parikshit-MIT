#define _USE_MATRIX_PRINT
#include "ekf.hpp"

int main()
{
    float_t qn0 = 0.98f, qn1 = 0.01f, qn2 = 0.02f, qn3 = 0.05f;
    float_t wnx = 0.001f, wny = -0.002f, wnz = 0.0005f;
    Matrix Wk1(4, 1, 0.0001f);
    Matrix Wk2(3, 1, 0.0001f);

    Matrix I({{0.02f, 0.0f, 0.0f},
              {0.0f, 0.022f, 0.0f},
              {0.0f, 0.0f, 0.018f}});

    Matrix Tor({{0.001f}, {0.0015f}, {0.0005f}});

    Matrix Pn(7, 7, 0.01f);

    Matrix B_true({{0.2f}, {0.01f}, {0.45f}});
    Matrix noise_b(3, 1, 0.001f);
    Matrix noise_w(3, 1, 0.0001f);

    Matrix Q1_bar(4, 1, 0.0f);
    Matrix W1_bar(3, 1, 0.0f);
    Matrix Pn1(7, 7, 0.0f);
    Matrix B_meas(3, 1, 0.0f);
    Matrix O_meas(3, 1, 0.0f);
    Matrix omega_s(3, 1, 0.0f);

    Prediction(qn0, qn1, qn2, qn3, wnx, wny, wnz, Wk1, Wk2, I, Tor, Pn, B_true, noise_b, noise_w, Q1_bar, W1_bar, Pn1, B_meas, O_meas, omega_s);

    std::cout << "After Prediction:\n";
    std::cout << "Q1_bar =\n"
              << Q1_bar << "\n";
    std::cout << "W1_bar =\n"
              << W1_bar << "\n";
    std::cout << "Pn1 =\n"
              << Pn1 << "\n";
    std::cout << "B_meas =\n"
              << B_meas << "\n";
    std::cout << "O_meas =\n"
              << O_meas << "\n";
    std::cout << "omega_s =\n"
              << omega_s << "\n";

    float_t Bx_ig = 0.2f, By_ig = 0.01f, Bz_ig = 0.45f;

    Matrix Vk1(3, 1, 0.0005f);
    Matrix Vk2(3, 1, 0.0001f);

    Matrix Qn1(4, 1, 0.0f);
    Matrix Wn1(3, 1, 0.0f);
    Matrix Pn_corrected(7, 7, 0.0f);

    Measurement(qn0, qn1, qn2, qn3, wnx, wny, wnz, Bx_ig, By_ig, Bz_ig, B_meas, O_meas, Q1_bar, W1_bar, Pn1, Vk1, Vk2, Qn1, Wn1, Pn_corrected);
    std::cout << "After Measurement:\n";
    std::cout << "Qn1 =\n"
              << Qn1 << "\n";
    std::cout << "Wn1 =\n"
              << Wn1 << "\n";
    std::cout << "Pn_corrected =\n"
              << Pn_corrected << "\n";

    printf("Q1_bar:\n");
    for (uint8_t i = 0; i < 4; ++i)
        printf("%.4f\n", Q1_bar(i, 0));
    printf("W1_bar:\n");
    for (uint8_t i = 0; i < 3; ++i)
        printf("%.4f\n", W1_bar(i, 0));
    printf("Pn1:\n");
    for (uint8_t i = 0; i < 7; ++i)
    {
        for (uint8_t j = 0; j < 7; ++j)
            printf("%.4f ", Pn1(i, j));
        printf("\n");
    }
    printf("B_meas:\n");
    for (uint8_t i = 0; i < 3; ++i)
        printf("%.4f\n", B_meas(i, 0));
    printf("O_meas :\n");
    for (uint8_t i = 0; i < 3; ++i)
        printf("%.4f\n", O_meas(i, 0));
    printf("omega_s:\n");
    for (uint8_t i = 0; i < 3; ++i)
        printf("%.4f\n", omega_s(i, 0));
    printf("Qn1:\n");
    for (uint8_t i = 0; i < 4; ++i)
        printf("%.4f\n", Qn1(i, 0));
    printf("Wn1:\n");
    for (uint8_t i = 0; i < 3; ++i)
        printf("%.4f\n", Wn1(i, 0));
    printf("Pn_corrected:\n");
    for (uint8_t i = 0; i < 7; ++i)
    {
        for (uint8_t j = 0; j < 7; ++j)
            printf("%.4f ", Pn_corrected(i, j));
        printf("\n");
    }
    return 0;
}
