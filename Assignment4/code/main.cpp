#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    if (control_points.size() == 1) {
        return control_points.front();
    }

    std::vector<cv::Point2f> new_control_points;
    for (int i = 0; i < control_points.size() - 1; ++i) {
        new_control_points.push_back(control_points[i + 1] * (1 - t) + control_points[i] * t);
    }

    return recursive_bezier(new_control_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        cv::Point2f point = recursive_bezier(control_points, t);
        float x = point.x, y = point.y;

        int x00 = x, y00 = y;
        int x01 = x, y01 = y + 1;
        int x10 = x + 1, y10 = y;
        int x11 = x + 1, y11 = y + 1;


        float r1 = sqrt(pow(x - x00, 2) + pow(y - y00, 2));
        float r2 = sqrt(pow(x - x01, 2) + pow(y - y01, 2));
        float r3 = sqrt(pow(x - x10, 2) + pow(y - y10, 2));
        float r4 = sqrt(pow(x - x11, 2) + pow(y - y11, 2));

        window.at<cv::Vec3b>(y00, x00)[1] = 255 * (sqrt(2) - r1) / sqrt(2);
        window.at<cv::Vec3b>(y01, x01)[1] = 255 * (sqrt(2) - r2) / sqrt(2);
        window.at<cv::Vec3b>(y10, x10)[1] = 255 * (sqrt(2) - r3) / sqrt(2);
        window.at<cv::Vec3b>(y11, x11)[1] = 255 * (sqrt(2) - r4) / sqrt(2);
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
//            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
