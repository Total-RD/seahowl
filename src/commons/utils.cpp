#include "seahowl/commons/utils.h"

#include <string>
#include <iostream>

std::vector<seahowl::DiscretizationPoint> seahowl::get_indice_and_positions(
    const std::vector<double>& discretization_fractions,
    const std::vector<double>& reference_fractions) {
    // check for potential errors
    if (discretization_fractions.size() == 0) {
        throw std::runtime_error("Cannot get discretization with empty array.");
    } else if (reference_fractions.size() < 2) {
        throw std::runtime_error("Cannot get discretization with reference array with less than 2 elements.");
    }

    std::vector<DiscretizationPoint> points;
    for (int ii = 0; ii < discretization_fractions.size(); ii++) {
        double fraction = discretization_fractions[ii];
        // check bounds
        if (fraction < 0.0 || fraction > 1.0) {
            throw std::runtime_error("Discretization fraction must be between 0 and 1 but was " +
                                     std::to_string(fraction) + ".");
        }
        if (fraction == 0) {
            DiscretizationPoint point{};
            point.index = 0;
            point.eta = -1;
            points.push_back(point);
        } else {
            auto idx = std::upper_bound(reference_fractions.begin(), reference_fractions.end(), fraction) -
                       reference_fractions.begin();
            // decrease index for getting lower bound
            idx -= 1;
            if (idx == reference_fractions.size() - 1) {
                idx -= 1;
            }
            double fraction_lower = reference_fractions[idx];
            double fraction_upper = reference_fractions[idx + 1];
            double fraction_range = fraction_upper - fraction_lower;
            double eta = 2.0 * (fraction - fraction_lower) / fraction_range - 1.0;
            DiscretizationPoint point{};
            point.index = static_cast<decltype(point.index)>(idx);
            point.eta = eta;
            points.push_back(point);
        }
    }
    return points;
}

double seahowl::bilinear_interpolation(const Eigen::MatrixXd& dataMatrix,
                                       const Eigen::VectorXd& x_list,
                                       const Eigen::VectorXd& y_list,
                                       double y,
                                       double x) {
    // // //test the interp2D
    // // Eigen::Vector2d x_list; x_list(0) = 2.0; x_list(1) = 3.0;
    // // Eigen::Vector2d y_list; y_list(0) = 2.0; y_list(1) = 3.0;
    // // double x = 2.5;
    // // double y = 2.5;
    // // Eigen::Matrix2d dataMatrix;
    // // dataMatrix << 0.0 , 5.0 , 0.0 , 5.0;

    // Find the four surrounding data points
    int x0, y0 = -99;
    for (unsigned ii = 0; ii < x_list.size() - 1; ii++) {
        if (x >= x_list[ii] && x <= x_list[ii + 1])
            x0 = ii;
    }
    for (unsigned ii = 0; ii < y_list.size() - 1; ii++) {
        if (y >= y_list[ii] && y <= y_list[ii + 1])
            y0 = ii;
    }

    if (y0 == -99 || x0 == -99) {
        std::cout << "x = " << x << ",  y = " << y << " , " << std::endl;
        std::cout << "x list = " << x_list.transpose() << ",  \n"
                  << "y list = " << y_list.transpose() << " , " << std::endl;
        throw std::runtime_error("x or y not found in the coefficients list.");
    };

    double x_frac = (x - x_list[x0]) / (std::fabs(x_list[x0] - x_list[x0 + 1]));
    double y_frac = (y - y_list[y0]) / (std::fabs(y_list[y0] - y_list[y0 + 1]));

    // std::cout << "x_frac: "<< x_frac << std::endl;
    // std::cout << "y_frac: "<< y_frac << std::endl;

    std::cout << "indx: " << x0 << std::endl;
    std::cout << "indy: " << y0 << std::endl;

    double q11 = dataMatrix(y0, x0);
    double q21 = dataMatrix(y0, x0 + 1);
    double q12 = dataMatrix(y0 + 1, x0);
    double q22 = dataMatrix(y0 + 1, x0 + 1);

    double fP = (1 - x_frac) * (1 - y_frac) * q11 + x_frac * (1 - y_frac) * q21 + (1 - x_frac) * y_frac * q12 +
                x_frac * y_frac * q22;

    return fP;

    // float x2 = x_list[x0+1];
    // float y2 = y_list[y0+1];
    // float x1 = x_list[x0];
    // float y1 = x_list[y0];

    // float x2x1, y2y1, x2x, y2y, yy1, xx1;
    // x2x1 = x2 - x1;
    // y2y1 = y2 - y1;
    // x2x = x2 - x;
    // y2y = y2 - y;
    // yy1 = y - y1;
    // xx1 = x - x1;
    // float fP = 1.0 / (x2x1 * y2y1) * (
    //     q11 * x2x * y2y +
    //     q21 * xx1 * y2y +
    //     q12 * x2x * yy1 +
    //     q22 * xx1 * yy1
    // );
    // return fP;

    // int x0 = -1, y0 = -1;

    // // Find the closest grid points
    // for (int i = 0; i < x_list.size(); i++) {
    //     if (x >= x_list(i)) {
    //         x0 = i;
    //     } else {
    //         break;
    //     }
    // }

    // for (int i = 0; i < y_list.size(); i++) {
    //     if (y >= y_list(i)) {
    //         y0 = i;
    //     } else {
    //         break;
    //     }
    // }

    // if (x0 == -1 || y0 == -1 || x0 >= x_list.size() - 1 || y0 >= y_list.size() - 1) {
    //     throw std::runtime_error("x or y out of range.");
    // }

    // // Calculate fractional coordinates
    // double dx = x - x_list(x0);
    // double dy = y - y_list(y0);

    // // Define the bicubic kernel
    // Eigen::Matrix4d B;
    // B << 1.0, 0.0, 0.0, 0.0,
    //      0.0, 0.0, 1.0, 0.0,
    //      -3.0, 3.0, -2.0, -1.0,
    //      2.0, -2.0, 1.0, 1.0;

    // Eigen::Vector4d X(dx * dx * dx, dx * dx, dx, 1.0);
    // Eigen::Vector4d Y(dy * dy * dy, dy * dy, dy, 1.0);

    // // Perform bicubic interpolation
    // Eigen::MatrixXd Q(4, 4);
    // if (y0>0 and x0>0){
    //     for (int i = 0; i < 4; i++) {
    //         for (int j = 0; j < 4; j++) {
    //             Q(i, j) = dataMatrix(y0-1 + i, x0-1 + j);
    //         }
    //     }
    // }
    // else{
    //     for (int i = 0; i < 4; i++) {
    //         for (int j = 0; j < 4; j++) {
    //             Q(i, j) = dataMatrix(y0 + i, x0 + j);
    //         }
    //     }
    // }

    // Eigen::VectorXd PX = X.transpose() * B * Q * B.transpose();
    // double result = PX.transpose() * Y;

    // return result;
}
