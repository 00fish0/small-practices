#include <iostream>
#include <cmath>
#include <iomanip>

const double k = 0.5;
const double m = 2;
const double g = 9.8;

double f1(double v1, double v2)
{
    return -(k / m) * sqrt(v1 * v1 + v2 * v2) * v1;
}
double f2(double v1, double v2)
{
    return -(k / m) * sqrt(v1 * v1 + v2 * v2) * v2 - g;
}

void RK4(double t0, double &v1, double &v2, double h)
{
    double k11 = h * f1(v1, v2);
    double k12 = h * f2(v1, v2);

    double k21 = h * f1(v1 + 1.0 / 2 * k11, v2 + 1.0 / 2 * k12);
    double k22 = h * f2(v1 + 1.0 / 2 * k11, v2 + 1.0 / 2 * k12);

    double k31 = h * f1(v1 + 1.0 / 2 * k21, v2 + 1.0 / 2 * k22);
    double k32 = h * f2(v1 + 1.0 / 2 * k21, v2 + 1.0 / 2 * k22);

    double k41 = h * f1(v1 + k31, v2 + k32);
    double k42 = h * f2(v1 + k31, v2 + k32);

    double v1out = v1 + 1.0 / 6 * (k11 + 2 * k21 + 2 * k31 + k41);
    double v2out = v2 + 1.0 / 6 * (k12 + 2 * k22 + 2 * k32 + k42);
    v1 = v1out, v2 = v2out;
}

int n;
double v_x[100001], v_y[100001];
double tmax;
const double t0 = 0;
const double h = 1e-4;

void IntegralAndPrint()
{
    double x = 0, y = 0, t=0;
    for (int i = 1; i <= n; i++)
    {
        double dx = (v_x[i] + v_x[i + 1]) * h / 2.0;
        double dy = (v_y[i] + v_y[i + 1]) * h / 2.0;
        x += dx;
        y += dy;
        t += h;
        if(i%100==0)
            std::cout << std::setw(12) << t << std::setw(12) << x << std::setw(12) << y << std::endl;
    }
}

int main()
{
    double v0, theta;
    std::cout << "Please input v0, theta, max_time\n";
    std::cin >> v0 >> theta;

    std::cin >> tmax;
    n = (tmax - t0) / h;

    double v1 = v0 * cos(theta), v2 = v0 * sin(theta);
    v_x[0] = v1, v_y[0] = v2;

    for (int i = 1; i < n; i++)
    {
        RK4(t0, v1, v2, h);
        v_x[i] = v1;
        v_y[i] = v2;
    }
    std::cout << std::setw(12) << 't' << std::setw(12) << 'x' << std::setw(12) << 'y' << std::endl;
    IntegralAndPrint();
}