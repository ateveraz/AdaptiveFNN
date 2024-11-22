//
// Created by ateveraz on 19/11/24.
//

#ifndef FOURIERNN_H
#define FOURIERNN_H

#include <iostream>
#include <Eigen/Dense>

class FourierNN {
public:
    FourierNN();
    ~FourierNN();

    void setFourierNN(const float W0_, const float W1_, const float omega_, const float threshold_);
    Eigen::Vector3f computeFourierNN(Eigen::Vector3f& errorSM, float t, float delta_t);
    void ResetWeights();

    Eigen::Matrix<float, 3, 7> getWeights();

private:
    Eigen::Matrix<float, 3, 7> Weights;             // Declaración de la matriz Weights
    Eigen::Matrix<float, 7, 1> InputBase;           // Regressor
    Eigen::Vector3f Approximation;                  // Product of C*Regressor
    Eigen::Matrix3f monitor(const Eigen::Vector3f& errorSM);
    void getInputs(float t);
    Eigen::Matrix<float, 3, 7> updateWeights(const Eigen::Matrix3f& Psi, const Eigen::Vector3f& nu);

    float W0, W1, threshold, omega;
};

#endif //FOURIERNN_H