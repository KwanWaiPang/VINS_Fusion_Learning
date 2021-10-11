#ifndef CAMERA_H
#define CAMERA_H

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>

namespace camodocal
{

class Camera
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum ModelType
    {//相机模型的种类
        KANNALA_BRANDT,//鱼眼相机
        MEI,
        PINHOLE,//普通相机
        PINHOLE_FULL,
        SCARAMUZZA
    };
//相机下的参数
    class Parameters
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Parameters( ModelType modelType );

        Parameters( ModelType modelType, const std::string& cameraName, int w, int h );

        ModelType& modelType( void );
        std::string& cameraName( void );
        int& imageWidth( void );
        int& imageHeight( void );

        ModelType modelType( void ) const;
        const std::string& cameraName( void ) const;
        int imageWidth( void ) const;
        int imageHeight( void ) const;

        int nIntrinsics( void ) const;

        virtual bool readFromYamlFile( const std::string& filename )      = 0;
        virtual void writeToYamlFile( const std::string& filename ) const = 0;

        protected:
        ModelType m_modelType;//种类
        int m_nIntrinsics;//内参
        std::string m_cameraName;
        int m_imageWidth;
        int m_imageHeight;
    };

    virtual ModelType modelType( void ) const           = 0;
    virtual const std::string& cameraName( void ) const = 0;
    virtual int imageWidth( void ) const                = 0;
    virtual int imageHeight( void ) const               = 0;

    virtual cv::Mat& mask( void );
    virtual const cv::Mat& mask( void ) const;

//估计内参
    virtual void estimateIntrinsics( const cv::Size& boardSize,
                                     const std::vector< std::vector< cv::Point3f > >& objectPoints,
                                     const std::vector< std::vector< cv::Point2f > >& imagePoints )
    = 0;
    //估计外参
    virtual void estimateExtrinsics( const std::vector< cv::Point3f >& objectPoints,
                                     const std::vector< cv::Point2f >& imagePoints,
                                     cv::Mat& rvec,
                                     cv::Mat& tvec ) const;

    // Lift points from the image plane to the sphere
    virtual void liftSphere( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const = 0;
    //%output P

//下面两个虚函数至关重要。不同的相机模型对于这两个函数的实现不一样
    // Lift points from the image plane to the projective space （从2Dimage plane到3D投影空间）
    virtual void liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P ) const = 0;
    //%output P

    // Projects 3D points to the image plane (Pi function)  从3D点到2D的图像平面。
    virtual void spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p ) const = 0;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    // virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    //                          Eigen::Matrix<double,2,3>& J) const = 0;
    //%output p
    //%output J

    virtual void undistToPlane( const Eigen::Vector2d& p_u, Eigen::Vector2d& p ) const = 0;
    //%output p

    // virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0)
    // const = 0;
    virtual cv::Mat initUndistortRectifyMap( cv::Mat& map1,
                                             cv::Mat& map2,
                                             float fx           = -1.0f,
                                             float fy           = -1.0f,
                                             cv::Size imageSize = cv::Size( 0, 0 ),
                                             float cx           = -1.0f,
                                             float cy           = -1.0f,
                                             cv::Mat rmat = cv::Mat::eye( 3, 3, CV_32F ) ) const = 0;

    virtual int parameterCount( void ) const = 0;

    virtual void readParameters( const std::vector< double >& parameters )  = 0;
    virtual void writeParameters( std::vector< double >& parameters ) const = 0;

    virtual void writeParametersToYamlFile( const std::string& filename ) const = 0;

    virtual std::string parametersToString( void ) const = 0;

    /**
     * \brief Calculates the reprojection distance between points
     *
     * \param P1 first 3D point coordinates
     * \param P2 second 3D point coordinates
     * \return euclidean distance in the plane
     */
    double reprojectionDist( const Eigen::Vector3d& P1, const Eigen::Vector3d& P2 ) const;

    double reprojectionError( const std::vector< std::vector< cv::Point3f > >& objectPoints,
                              const std::vector< std::vector< cv::Point2f > >& imagePoints,
                              const std::vector< cv::Mat >& rvecs,
                              const std::vector< cv::Mat >& tvecs,
                              cv::OutputArray perViewErrors = cv::noArray( ) ) const;

    double reprojectionError( const Eigen::Vector3d& P,
                              const Eigen::Quaterniond& camera_q,
                              const Eigen::Vector3d& camera_t,
                              const Eigen::Vector2d& observed_p ) const;

    void projectPoints( const std::vector< cv::Point3f >& objectPoints,
                        const cv::Mat& rvec,
                        const cv::Mat& tvec,
                        std::vector< cv::Point2f >& imagePoints ) const;

    protected:
    cv::Mat m_mask;
};

typedef boost::shared_ptr< Camera > CameraPtr;
typedef boost::shared_ptr< const Camera > CameraConstPtr;
}

#endif
