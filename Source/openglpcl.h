#ifndef OpenGLPCL_H
#define OpenGLPCL_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QScopedPointer>
#include <QException>
#include <QMutex>
#include <QtConcurrent/QtConcurrent>
#include <DepthVistaSDK/DepthVista.h>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QElapsedTimer>
#include <QCursor>
#include <QMatrix4x4>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "pcl/io/ply_io.h"


using namespace std;
using namespace cv;

typedef enum {
    UYVY_PCL = 1,
    Y16_PCL = 2,
    RGB_PCL = 3,
}PixelformatPCL;
struct color_point_t
{
    float xyz[3];
    uint8_t rgb[3];
};
#define COLORMAP_LIMIT 21

#define FOCUS_POINT_VGA_X               385.714
#define FOCUS_POINT_VGA_Y               385.714
#define PRINCIPLE_AXIS_VGA_X            320
#define PRINCIPLE_AXIS_VGA_Y            240

#define FOCUS_POINT_HD_X                385.714 * 1.67
#define FOCUS_POINT_HD_Y                385.714 * 1.67
#define PRINCIPLE_AXIS_HD_X             640
#define PRINCIPLE_AXIS_HD_Y             360


class OpenGLPCL : public QOpenGLWidget, public QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit OpenGLPCL(QWidget* parent = nullptr,PixelformatPCL pixformat = UYVY_PCL);
    ~OpenGLPCL();
    QMutex							renderMutex; // mutex to use in rendering
    bool							gotFrame;
    double							alpha, beta;
    uint16_t						colorMap;
    QQuaternion						local_qrot;
    QVector3D						local_pos;
    QVector3D						local_rot;
    PixelformatPCL                  pixelFormat;
    bool							savePLYfile = false;
    QString							plyFileName;
    uint16_t						depth_min_val_ply;
    uint16_t						depth_max_val_ply;
    uint16_t                        depth_min_val;
    uint16_t                        depth_max_val;
	uint32_t						currentWidth;
	uint32_t						currentHeight;
    bool                            savePLYSpecificRange = false;
    void setSavePLYfile(QString file_name, bool save_specific_range, uint16_t depth_min_val, uint16_t depth_max_val);
    void InitDrawBuffer(unsigned bsize);
    void DisplayVideoFrame(unsigned char *depth_data, unsigned char* rgb_data, int frameWidth, int frameHeight);
    void StopFrame();
    void getColorMapProp(uint16_t depthmin, uint16_t depthMax, uint16_t colormap);

    cv::Mat							DepthImg;
    cv::Mat							PrevDepthFrame;
    cv::Mat							Depthcolormap;
    cv::Mat							rawDepthFrame;
    QMatrix4x4						ViewMatrix;
    QMatrix4x4						ProjectionMatrix;
    QMatrix4x4						cameraMatrix;
    // Initial position : on +Z
    QVector3D						position;
    QPoint							lastMousePos;
    float							horizontalAngle;
    float							verticalAngle;
    // Initial Field of View
    float							initialFoV;
    //Initial camera position
    bool							initialPos;

    QOpenGLVertexArrayObject		vao;
    QOpenGLBuffer					vbo,vbo1;
    float							mouseSpeed = 0.5f;
    QWheelEvent						*wheelevent;
    void computeMatricesFromInputs();
    void setInitialPos();
	void dataModeChanged(uint8_t dataMode);
    void paintGL() override;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent* event) override;
signals:
    Q_SIGNAL void maximized(QWidget* widget);
    Q_SIGNAL void renderframe();
    Q_SIGNAL void ply_file_save();
public slots:
    Q_SLOT void updateFrame();

private slots:
    Q_SLOT void end_ply_saving_thread();
private:
    struct OpenGLPCLImpl;
    QScopedPointer<OpenGLPCLImpl>   impl;
    int                             mPositionLoc;
    int                             mTexCoordLoc;
    GLint                           samplerLoc;
    GLuint                          TextureId;
    GLuint                          modelID,viewID,projectionID;
    void shaderRGB();
    void renderRGB();
    unsigned char                   *srcBuffer;
    uint8_t                         *pfmb;
    QFuture<void>                   savePLYThread;
    void                            saveplyFile();
    bool                            save_cloud_ready = false;
    bool                            ply_thread_running = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr save_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

};

/***********************************************************************/

class OpenGlExceptionPCL: public QException
{
public:
     void raise() const { throw *this; }
     OpenGlExceptionPCL *clone() const { return new OpenGlExceptionPCL(*this); }
};

#endif // OpenGLPCL_H
