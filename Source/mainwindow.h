#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QScreen>
#include <QSlider>
#include <QComboBox>
#include <QLabel>
#include "cameraproperties.h"
#include <DepthVistaSDK/DepthVista.h>
#include <QString>
#include <QSurfaceFormat>
#include <QtConcurrent/QtConcurrent>
#include <QThread>
#include <QAction>
#include "opengldisplay.h"
#include "openglpcl.h"
#include <utility>
#include <vector>
#include <chrono>
#include <QMouseEvent>
#include <opencv2/rgbd/depth.hpp>


#define PRE_RAW_WIDTH      		640
#define PRE_RAW_HEIGHT			240
#define PRE_DEPTH_IR_WIDTH 		640
#define PRE_DEPTH_IR_HEIGHT		480
#define PRE_RGB_VGA_WIDTH		640
#define PRE_RGB_VGA_HEIGHT		480
#define PRE_RGB_HD_WIDTH		1280
#define PRE_RGB_HD_HEIGHT		720
// Preview RGB Full HD Image Resolution 1920x1080
#define PRE_RGB_FULL_HD_WIDTH	1920
#define PRE_RGB_FULL_HD_HEIGHT	1080

// Preview RGB Full HD Image Resolution 1920x1080
#define PRE_RGB_ORIGINAL_WIDTH	1920
#define PRE_RGB_ORIGINAL_HEIGHT 1200

typedef enum 
{
    All = 0,
    ToF_cam = 1,
    ColorCam_uvc = 2,
    ColorCam_extension = 3,
    Default = 4
}CamProp;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void callback(int a);
    void notification_callback(int a);
signals:
    Q_SIGNAL void deviceEnumerated(QStringList devices);

    Q_SIGNAL void dataModeQueried(DataMode cDataMode);
    Q_SIGNAL void depthRangeQueried(uint16_t cDepthRange);
    Q_SIGNAL void tofExposureQueried(uint16_t cExpValue);
    Q_SIGNAL void tofCoringQueried(uint16_t cCoringValue);
    Q_SIGNAL void campropQueried(std::vector< std::pair <int,int> > camera_prop);
    Q_SIGNAL void depthModeChanged(std::vector< std::pair <int,int> > camera_prop);
    Q_SIGNAL void uvcpropQueried(std::vector< UVCProp > uvc_prop);
    Q_SIGNAL void windowResized();

    Q_SIGNAL void setMousePointer(int x, int y, int ctrlID);
    Q_SIGNAL void updateData(int avgDepth, int stdDepth, int avgIR, int stdIR);
    Q_SIGNAL void savingFramesComplete();
    Q_SIGNAL void tofCamModeSelected(bool ir_mode);
    Q_SIGNAL void dualCamModeSelected(bool vgaMode);
    Q_SIGNAL void rgbCamModeSelected();
    Q_SIGNAL void deviceRemoved();
    Q_SIGNAL void firmwareVersionRead(uint8_t gMajorVersion, uint8_t gMinorVersion1, uint16_t gMinorVersion2, uint16_t gMinorVersion3);
    Q_SIGNAL void uniqueIDRead(uint64_t uniqueID);


protected slots:
    Q_SLOT void enumerateDevices();
    Q_SLOT void onInputSelected(int index);
    Q_SLOT void onslidemenu();
    Q_SLOT void onSetCamProperty(int ctrlID, int value);
    Q_SLOT void onSetFilterType(int ctrlID, bool selected);
    Q_SLOT void onSetCamProperty(int ctrlID, int value, int value1, int value2);
    Q_SLOT void onSetMousePointer(int x, int y, int ctrlID);
    Q_SLOT void ongetDepthIRdata();
    Q_SLOT void onSavePLYClicked(QString saveDir);
    Q_SLOT void onwidgetMaximized(QWidget* widget);
    Q_SLOT void onApplyAvgChangesClicked(int avg_x, int avg_y);
    Q_SLOT void onStartSavingFrames(bool depth, bool ir, bool rgb, bool rawDepth, bool PYL, QString save_frame_dir);
    Q_SLOT void savingPLYFramesOver();
    Q_SLOT void onPCLDisplay_checkBox_stateChange(bool state);
    Q_SLOT void onRGBDMapping_checkBox_stateChange(int state);
    Q_SLOT void onUndistort_checkBox_stateChange(int state);
private:
    int									oldindex = 0;
    DataMode							data_mode = ModeUnknown;
    uint16_t							depthRange = 0;
    Ui::MainWindow						*ui;
    DeviceInfo							*gDevicesList;
    Cameraproperties					*side_menu;

    OpenGLDisplay*						yuvdisplay = NULL;
    OpenGLDisplay*						irdisplay = NULL;
    OpenGLDisplay*						depthdisplay = NULL;
    OpenGLPCL*							pclDisplay = NULL;
    bool								pcl_display_flag = true;
    bool								pcl_display_flag_1 = false;
    bool								rgbDMapping_flag = false;
    bool								rgbDMapping_flag_thread = false;
	bool								depthIntrinsicData = false;
	bool								rgbIntrinsicData = false;
	bool								extrinsicData = false;
	bool								undistortDepth = false;
    int									depth_range;
	int									count = 0;

    QLabel*								fps_label;


    ToFFrame							ToFIrFrame;
    ToFFrame							ToFDepthColorMapFrame;
    ToFFrame							ToFDepthRawFrame;
    ToFFrame							ToFRGBFrame;
    uint8_t								queryCamPropFlag = 1 << CamProp::All;
    std::vector<UVCProp>				uvc_prop;
    uint16_t							Depth_min = 0, Depth_max = 4095;

    cv::Mat								DepthImg_Y16 = cv::Mat(PRE_DEPTH_IR_HEIGHT, PRE_DEPTH_IR_WIDTH, CV_16UC1);
    cv::Mat								Depthcolormap = cv::Mat(480, 640, CV_8UC3);
    cv::Mat								Depthcolormap_cpy = cv::Mat(480, 640, CV_8UC3);
    cv::Mat								IRImg_cpy = cv::Mat(480,640, CV_16UC1);
    cv::Mat								UYVY_frame;
    cv::Mat								UYVY_frame_cpy;
    cv::Mat								rgbFrame;
	QMutex								renderMutex; // mutex to use in rendering
	uint64_t							skippedFrames = 0;
	bool								first_occurance = true;


    bool								stopFlag = false;
    bool								sidebarstatechanged = false;
    bool								widgetMaximized = false;
    int									auto_roi_win_size = 0;
    bool								savePLY = false;
    bool								saveDepthFrame = false;
    bool								saveIRFrame = false;
    bool 								saveRGBFrame = false;
    bool 								saveRawFrame = false;
    bool 								saveRawFrame_start = false;
    bool 								saveDepthFrame_start = false;
    bool 								saveIRFrame_start = false;
    bool 								saveRGBFrame_start = false;
    bool 								savePLY_start = false;
    bool 								savingPLY = false;
    bool 								saveCorrespondingDepth = true;
    bool 								saveCorrespondingIR = true;
    bool 								saveCorrespondingRGB = true;
    bool 								saveCorrespondingRaw = true;
    bool 								startedSavingFrames = false;
    int									savingRaw = 0;
    uint16_t							saveDepthFrameCount =0, saveIRFrameCount = 0, saveRGBFrameCount = 0, saveRawFrameCount = 0, savePLYFrameCount=0;
    QString								saveDepthDir;
    QString 							saveRGBDir;
    QString 							saveIRDir;
    QString 							save3dDir;
    QString 							saveRAWDir;
    QString 							savePLYfile_name;

    cv::Mat								M1, M2, D1, D2;
	cv::Mat								transform_matrix;

    int									avgRegionFlag = 0;
    std::vector<MousePtr>				savePos;

    bool								save_ply_flag = false;
    bool								saved_ply_flag = true;
    bool								spc_ply_flag = false;
    bool								depth_mouse_clicked = false;
    bool								calibParamObtained = false;
    uint16_t							depth_min_val = 0, depth_max_val = 0;

    cv::FileStorage						readDepthIntrinsic;
    cv::FileStorage						readRGBIntrinsic;
    cv::FileStorage						readExtrinsic;
    cv::Mat								rotation_matrix;
    cv::Mat								translation_matrix;
    cv::Mat								PrevDepthFrame;
    cv::Mat								PrevDepthFrame1;
    cv::Mat								PrevDepthFrame2;
    cv::Mat								Depthcolormap1;
    cv::Mat								Depthcolormap2;
    cv::Mat								Depthcolormap3;
    cv::Mat								depth_intrinsic;
    cv::Mat								depth_intrinsic_new;
    cv::Mat								depthUndistort = cv::Mat(PRE_DEPTH_IR_HEIGHT, PRE_DEPTH_IR_WIDTH, CV_16UC1);
    cv::Mat								readDepth = cv::Mat(PRE_DEPTH_IR_HEIGHT, PRE_DEPTH_IR_WIDTH, CV_16UC1);
    cv::Mat								readDepth1 = cv::Mat(720, 1280, CV_16UC1);
    cv::Mat								readDepth2 = cv::Mat(720, 1280, CV_16UC1);
    cv::Mat								depth_distortion;
    cv::Mat								rgb_intrinsic;
    cv::Mat								rgbHD_intrinsic;
    cv::Mat								rgb_distortion;
    cv::Mat								registeredDepth;
    cv::Mat								registeredDepthHD;
    cv::Mat								writeRegisteredDepthHD;
    double								alpha, beta;
    double								x_comp = 0, y_comp = 0;

    QFuture<void> previewRenderThread;
    QFuture<void> savePLYThread;
    QFuture<void> fpsDisplayThread;

    int									windowheight;
    int									windowwidth;
    double								widgetheight,widgetwidth;
    double								vga_aspect_ratio = 640.00/480.00;
    double								hd_aspect_ratio = 1280.00/720.00;
    double								full_hd_aspect_ratio = 1920.00/1080.00;
    double								rgb_1200p_aspect_ratio = 1920.00/1200.00;

    void getFrame();
    void createDisplayComponents();
    void modifyUi(DataMode dataMode);
    void modifySize(DataMode dataMode);
    void startStream();
    void stopStream();
    void querycameraproperties(uint8_t cam);
	void framesPerSecondDisplay();
protected:
    bool eventFilter(QObject *obj, QEvent *event);
    void resizeEvent(QResizeEvent *event);
};
#endif // MAINWINDOW_H
