#include "camera_models/Camera.h"
#include "camera_models/ScaramuzzaCamera.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace camodocal
{

    Camera::Parameters::Parameters(ModelType modelType)
        : m_modelType(modelType), m_imageWidth(0), m_imageHeight(0)
    {
        switch (modelType)
        {
        case KANNALA_BRANDT:
            m_nIntrinsics = 8;
            break;
        case PINHOLE:
            m_nIntrinsics = 8;
            break;
        case SCARAMUZZA:
            m_nIntrinsics = SCARAMUZZA_CAMERA_NUM_PARAMS;
            break;
        case MEI:
        default:
            m_nIntrinsics = 9;
        }
    }

    Camera::Parameters::Parameters(ModelType modelType,
                                   const std::string &cameraName,
                                   int w, int h)
        : m_modelType(modelType), m_cameraName(cameraName), m_imageWidth(w), m_imageHeight(h)
    {
        switch (modelType)
        {
        case KANNALA_BRANDT:
            m_nIntrinsics = 8;
            break;
        case PINHOLE:
            m_nIntrinsics = 8;
            break;
        case SCARAMUZZA:
            m_nIntrinsics = SCARAMUZZA_CAMERA_NUM_PARAMS;
            break;
        case MEI:
        default:
            m_nIntrinsics = 9;
        }
    }

    Camera::ModelType &
    Camera::Parameters::modelType(void)
    {
        return m_modelType;
    }

    std::string &
    Camera::Parameters::cameraName(void)
    {
        return m_cameraName;
    }

    int &
    Camera::Parameters::imageWidth(void)
    {
        return m_imageWidth;
    }

    int &
    Camera::Parameters::imageHeight(void)
    {
        return m_imageHeight;
    }

    Camera::ModelType
    Camera::Parameters::modelType(void) const
    {
        return m_modelType;
    }

    const std::string &
    Camera::Parameters::cameraName(void) const
    {
        return m_cameraName;
    }

    int
    Camera::Parameters::imageWidth(void) const
    {
        return m_imageWidth;
    }

    int
    Camera::Parameters::imageHeight(void) const
    {
        return m_imageHeight;
    }

    int
    Camera::Parameters::nIntrinsics(void) const
    {
        return m_nIntrinsics;
    }

    cv::Mat &
    Camera::mask(void)
    {
        return m_mask;
    }

    const cv::Mat &
    Camera::mask(void) const
    {
        return m_mask;
    }

    void
    Camera::projectPoints(const std::vector<cv::Point3f> &objectPoints,
                          const cv::Mat &rvec,
                          const cv::Mat &tvec,
                          std::vector<cv::Point2f> &imagePoints) const
    {
        // project 3D object points to the image plane
        imagePoints.reserve(objectPoints.size());

        // double
        cv::Mat R0;
        cv::Rodrigues(rvec, R0);

        Eigen::MatrixXd R(3, 3);
        R << R0.at<double>(0, 0), R0.at<double>(0, 1), R0.at<double>(0, 2),
            R0.at<double>(1, 0), R0.at<double>(1, 1), R0.at<double>(1, 2),
            R0.at<double>(2, 0), R0.at<double>(2, 1), R0.at<double>(2, 2);

        Eigen::Vector3d t;
        t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

        for (size_t i = 0; i < objectPoints.size(); ++i)
        {
            const cv::Point3f &objectPoint = objectPoints.at(i);

            // Rotate and translate
            Eigen::Vector3d P;
            P << objectPoint.x, objectPoint.y, objectPoint.z;

            P = R * P + t;

            Eigen::Vector2d p;
            spaceToPlane(P, p);

            imagePoints.push_back(cv::Point2f(p(0), p(1)));
        }
    }

}
