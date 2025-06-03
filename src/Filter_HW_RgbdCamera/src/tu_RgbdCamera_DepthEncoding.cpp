#include <vision/CDepthImageProcessing.h>

int main()
{
    cv::Mat in(100, 100, CV_32FC1);
    cv::randu(in, 10.F, 100.F);

    auto val_in = in.at<float>(50, 50);
    std::cout << val_in << std::endl;

    cv::Mat encoded = CDepthImageProcessing::depth2image(in);

    auto val_encoded = encoded.at<cv::Vec3b>(50, 50);
    std::cout << val_encoded << std::endl;

    cv::Mat decoded = CDepthImageProcessing::image2depth(encoded);

    auto val_decoded = decoded.at<float>(50, 50);
    std::cout << val_decoded << std::endl;

    auto norm = cv::norm(decoded - in);
    std::cout << "Norm: " << norm << std::endl;

    return 0;
}
