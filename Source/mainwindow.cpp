#include "mainwindow.h"
#include "./ui_mainwindow.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    if(Initialize()<0){
        pError();
    }
    this->setWindowTitle("DepthVista");
    this->setWindowIcon(QIcon(":/images/DepthVistaIcon.png"));
    connect(ui->slide_menu_button,SIGNAL(clicked()),this,SLOT(onslidemenu()));
    connect(this,SIGNAL(setMousePointer(int,int,int)),this,SLOT(onSetMousePointer(int,int,int)));
    createDisplayComponents();
    onslidemenu();
}

double MainWindow::GetIMUIntervalTime(IMUCONFIG_TypeDef	lIMUConfig)
{

    double lIMUIntervalTime = 10;

    if (lIMUConfig.IMU_MODE == IMU_ACC_GYRO_ENABLE)
    {
        switch (lIMUConfig.IMU_ODR_CONFIG)
        {
        case IMU_ODR_12_5HZ:
            lIMUIntervalTime = 1000.00 / 12.5;
            break;

        case IMU_ODR_26HZ:
            lIMUIntervalTime = 1000.00 / 26.0;
            break;

        case IMU_ODR_52HZ:
            lIMUIntervalTime = 1000.00 / 52.0;
            break;

        case IMU_ODR_104HZ:
            lIMUIntervalTime = 1000.00 / 104.0;
            break;

        case IMU_ODR_208HZ:
            lIMUIntervalTime = 1000.00 / 208.0;
            break;

        case IMU_ODR_416HZ:
            lIMUIntervalTime = 1000.00 / 416.00;
            break;

        case IMU_ODR_833HZ:
            lIMUIntervalTime = 1000.00 / 833.00;
            break;

        case IMU_ODR_1666HZ:
            lIMUIntervalTime = 1000.00 / 1666.00;
            break;

        case IMU_ODR_3332HZ:
            lIMUIntervalTime = 1000.00 / 3332.00;
            break;

        case IMU_ODR_6664HZ:
            lIMUIntervalTime = 1000.00 / 6664.00;
            break;
        }
    }
    return lIMUIntervalTime;
}

double MainWindow::squared(double x)
{
    return x * x;
}

void MainWindow::getInclination(double g_x, double g_y, double g_z, double a_x, double a_y, double a_z)
{
    int w = 0;
    double tmpf = 0.0;
    int signRzGyro;
    static bool firstSample = true;
    double wGyro = 10.0;
    double norm;

    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;

    double RwAcc[3] = { a_x, a_y, a_z };
    double RwGyro[3] = { g_x, g_y, g_z };
    double Awz[2];
    double Gyro[3];

    if (firstSample)
    {
        for (w = 0; w <= 2; w++)
        {
            RwEst[w] = RwAcc[w];
        }
    }
    else
    {
        if (fabs(RwEst[2]) < 0.1)
        {
            for (w = 0; w <= 2; w++)
            {
                Gyro[w] = RwEst[w];
            }
        }
        else {
            //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
            for (w = 0; w <= 1; w++)
            {
                tmpf = RwGyro[w];                        //get current gyro rate in deg/s
                tmpf *= glIMU_Interval / 1000.0f;                     //get angle change in deg
                Awz[w] = atan2(RwEst[w], RwEst[2]) * RAD2DEG;   //get angle and convert to degrees
                Awz[w] += tmpf;             //get updated angle according to gyro movement
            }

            //estimate sign of RzGyro by looking in what qudrant the angle Axz is,
            //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
            signRzGyro = (cos(Awz[0] * DEG2RAD) >= 0) ? 1 : -1;

            //reverse calculation of Gyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
            Gyro[0] = sin(Awz[0] * DEG2RAD);
            Gyro[0] /= sqrt(1 + squared(cos(Awz[0] * DEG2RAD)) * squared(tan(Awz[1] * DEG2RAD)));
            Gyro[1] = sin(Awz[1] * DEG2RAD);
            Gyro[1] /= sqrt(1 + squared(cos(Awz[1] * DEG2RAD)) * squared(tan(Awz[0] * DEG2RAD)));
            Gyro[2] = signRzGyro * sqrt(1 - squared(Gyro[0]) - squared(Gyro[1]));
        }

        //combine Accelerometer and gyro readings
        for (w = 0; w <= 2; w++)
            RwEst[w] = (RwAcc[w] + wGyro * Gyro[w]) / (1 + wGyro);

        //Normalizing the estimates
        norm = sqrt(RwEst[0] * RwEst[0] + RwEst[1] * RwEst[1] + RwEst[2] * RwEst[2]);
        RwEst[0] /= norm;
        RwEst[1] /= norm;
        RwEst[2] /= norm;
    }

    firstSample = false;

    //Computing the angles
    angleX = RwEst[0] * HALF_PI * RAD2DEG;
    angleY = RwEst[1] * HALF_PI * RAD2DEG;
    angleZ = RwEst[2] * HALF_PI * RAD2DEG;

    emit imuValues(angleX, angleY, angleZ);
}

void MainWindow::createDisplayComponents()
{
    QScreen* screen = QGuiApplication::primaryScreen();
    QRect mm = screen->availableGeometry();
    int screen_width_mm = mm.width();
    int screen_height_mm = mm.height();
    int screen_width = this->width();
    int screen_height = this->height();
    int width_percent_30 = (int)screen_width_mm * 0.25;
    int height_percent_90 = (int)screen_height_mm * 0.9;
    side_menu = new Cameraproperties(this, oldindex);
    side_menu->setMinimumWidth(width_percent_30);
    side_menu->setMaximumHeight(height_percent_90);
    QWidget* w = new QWidget;
    QHBoxLayout* _hlayout = new QHBoxLayout();
    fps_label = new QLabel();
    QLabel* label1 = new QLabel("DepthVista Version : 1.0.0.11");
    label1->setAlignment(Qt::AlignRight);
    _hlayout->addWidget(fps_label);
    _hlayout->addWidget(label1);
    w->setLayout(_hlayout);

    ui->statusbar->addPermanentWidget(w, 2);
    ui->slide_menu_button->setMinimumSize(QSize(screen_width * 0.021, screen_height * 0.078));
    ui->slide_menu_button->setMaximumSize(QSize(screen_width * 0.021, screen_height * 0.078));
    ui->slide_menu_button->setIconSize(QSize(screen_width * 0.021, screen_height * 0.078));

    yuvdisplay = new OpenGLDisplay(this,Pixelformat::UYVY);
    yuvdisplay->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
    yuvdisplay->InitDrawBuffer((PRE_RGB_VGA_WIDTH*PRE_RGB_VGA_HEIGHT*2));
    ui->horizontalLayout_3->insertWidget(1,yuvdisplay);
    yuvdisplay->setVisible(false);

    depthdisplay = new OpenGLDisplay(this,Pixelformat::RGB);
    depthdisplay->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
    depthdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*3));
    ui->horizontalLayout_4->insertWidget(1,depthdisplay);
    depthdisplay->setVisible(false);

    irdisplay = new OpenGLDisplay(this,Pixelformat::Y16);
    irdisplay->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
    irdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*2));
    ui->horizontalLayout_3->insertWidget(2,irdisplay);
    irdisplay->setVisible(false);

    pclDisplay = new OpenGLPCL(this, PixelformatPCL::RGB_PCL);
    pclDisplay->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    pclDisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH * PRE_DEPTH_IR_HEIGHT * 3));
    ui->horizontalLayout_4->insertWidget(2, pclDisplay);
    pclDisplay->setVisible(false);
	//	Added by Abishek on 28/02/2023
	//	To pass the depth lens intrinsic parameters to create 3d file.
	pclDisplay->get3DIntrinsic(LENS_FOCUS_POINT_VGA_X, LENS_FOCUS_POINT_VGA_Y,
		LENS_PRINCIPLE_AXIS_VGA_X, LENS_PRINCIPLE_AXIS_VGA_Y);

}
void MainWindow::onslidemenu()
{

    if(sidebarstatechanged){

        side_menu->setVisible(false);
        ui->sidemenu->insertSpacerItem(0,new QSpacerItem(0,10,
                                                     QSizePolicy::Expanding,
                                                     QSizePolicy::Expanding));
        ui->slide_menu_button->setIcon(QIcon(":/images/open_tab.png"));
        sidebarstatechanged = false;
    }else{

        delete ui->sidemenu->takeAt(0);
        ui->sidemenu->insertWidget(0,side_menu);
        side_menu->setVisible(true);
        ui->slide_menu_button->setIcon(QIcon(":/images/close_tab.png"));
        sidebarstatechanged = true;
    }
}

void MainWindow::enumerateDevices()
{
    uint32_t count=0;
    QStringList devices;
    DeviceHandle enumeratedHandle;
	int updatedStreamingDeviceInd = -1;
    enumeratedHandleList.clear();
		devices.append("----Select Camera Device----");
    if(GetDeviceCount(&count)<0){
        pError("GetDeviceCount");
    }

    if(count>0){
        gDevicesList = new DeviceInfo[count];
        if(GetDeviceListInfo(count,gDevicesList)<0){
            pError("GetDeviceListInfo");
        }
        for(int i=0;i<count;i++){
            std::string devicename = gDevicesList[i].deviceName;
            std::string serialNo = devicename + " (" + gDevicesList[i].serialNo + ")";
            strcpy(enumeratedHandle.serialNo, gDevicesList[i].serialNo);
			if (std::strcmp(gDevicesList[i].serialNo, openDeviceHandle.serialNo) == 0)
			{
				updatedStreamingDeviceInd = i + 1;
			}
            devices.append(QString(serialNo.c_str()));
            if(enumeratedHandleList.size() > 0)
            {
                for(int itr = 0; itr < enumeratedHandleList.size() ; itr++)
                {
                    if (std::strcmp(gDevicesList[i].serialNo, enumeratedHandleList[itr].serialNo))
                    {
                        enumeratedHandleList.push_back(enumeratedHandle);
                    }
                }
            }
            else
            {
                enumeratedHandleList.push_back(enumeratedHandle);
            }
        }
		if (updatedStreamingDeviceInd == -1)
		{
			updatedStreamingDeviceInd = 0;
		}
        oldindex = updatedStreamingDeviceInd;
	}
	else
	{
		updatedStreamingDeviceInd = 0;
	}
	emit deviceEnumerated(devices, updatedStreamingDeviceInd);
    ui->scrollArea->setFocus();
}

void MainWindow::querycameraproperties(uint8_t cam)
{
    int gvalue = -1;
    uint8_t gvalue1 = 0,gvalue2 = 0,gvalue3 = 0;
    uint16_t gvalue4 = 0;
    uint32_t gvalue5 = 0;
    std::vector< std::pair <int,int> > camera_prop;
    uvc_prop.clear();

    if ((cam & (1 << CamProp::All)) || (cam & (1 << CamProp::ColorCam_uvc)))
    {
        for (int id = TOF_UVC_CID_BRIGHTNESS; id <= TOF_UVC_CID_EXPSOURE_ABS; id++)
        {
            UVCProp gProp;
            if (GetUVCControl(devHandle, id, &gProp) > 0) {
				PrintLog(LOG_ESSENTIAL_DEBUG, "control values:");
                    PrintLog(LOG_ESSENTIAL_DEBUG, "id         : " +  std::to_string(gProp.id));
                    PrintLog(LOG_ESSENTIAL_DEBUG, "cur        : " + std::to_string(gProp.cur));
                    PrintLog(LOG_ESSENTIAL_DEBUG, "min        : " + std::to_string(gProp.min));
                    PrintLog(LOG_ESSENTIAL_DEBUG, "max        : " + std::to_string(gProp.max));
                    PrintLog(LOG_ESSENTIAL_DEBUG, "step       : " + std::to_string(gProp.step));
                    PrintLog(LOG_ESSENTIAL_DEBUG, "default    : " + std::to_string(gProp.default_val) + "\n");
                uvc_prop.push_back(gProp);
            }
        }
        emit uvcpropQueried(uvc_prop);
    }
    if ((cam & (1 << CamProp::All)) || (cam & (1 << CamProp::ToF_cam)))
    {
        if (GetDataMode(devHandle, &data_mode) > 0) {
            camera_prop.push_back(std::make_pair(TOF_UVC_EXT_CID_DATA_MODE, data_mode));
            modifyUi(data_mode);
            if (data_mode <= DataMode::IR_Mode)
            {
                if(data_mode == DataMode::IR_Mode)
                    emit tofCamModeSelected(false);
                else
                    emit tofCamModeSelected(true);
            }
            else if (data_mode == DataMode::Depth_IR_RGB_VGA_Mode)
            {
                pclDisplay->dataModeChanged(Depth_IR_RGB_VGA_Mode);
                emit dualCamModeSelected(true);
            }
            else if (data_mode == DataMode::Depth_IR_RGB_HD_Mode)
            {
                pclDisplay->dataModeChanged(Depth_IR_RGB_HD_Mode);
                emit dualCamModeSelected(false);
            }
            else if (data_mode >= DataMode::RGB_VGA_Mode)
            {
				uint8_t frameRateCtrl = 10;
				uint8_t ROIMode, winSize;
				uint32_t expoComp = 10;
                if (GetFrameRateCtrl(devHandle, &frameRateCtrl) > 0) {
					PrintLog(LOG_ESSENTIAL_DEBUG, "Frame rate Ctrl get value is : " + std::to_string((uint16_t)frameRateCtrl));
				}
                if (GetExposureCompensation(devHandle, &expoComp) > 0) {
					PrintLog(LOG_ESSENTIAL_DEBUG, "GetExposureCompensation value is : " + std::to_string(expoComp));
				}
                if (GetAutoExposureROI(devHandle, &ROIMode, &winSize) > 0) {
					PrintLog(LOG_ESSENTIAL_DEBUG, "GetAutoExposureROI : " + std::to_string(ROIMode) + " , " + std::to_string(winSize ));
					if (ROIMode == TOF_APP_VAL_MANUAL_ROI) {

						yuvdisplay->setMouseTracking(true);
						yuvdisplay->installEventFilter(this);
					}
					else {
						yuvdisplay->setMouseTracking(false);
						yuvdisplay->removeEventFilter(this);
					}
				}
				emit rgbCamModeSelected(frameRateCtrl, expoComp, ROIMode, winSize);
            }
        }
        if (GetDepthRange(devHandle, &depthRange) > 0) {
            camera_prop.push_back(std::make_pair(TOF_UVC_EXT_CID_DEPTH_RANGE, depthRange));
            QSettings settings(QString("config.ini"), QSettings::IniFormat);

            if (depthRange == 0)
            {
                depth_max_val = TOF_APP_VAL_NEAR_SPC_DEPTH_MAX;
                depth_min_val = TOF_APP_VAL_NEAR_SPC_DEPTH_MIN;
                Depth_min = settings.value("Near_mode/Lower_limit").toInt();
                Depth_max = settings.value("Near_mode/Upper_limit").toInt();
                depth_range = (Depth_max - Depth_min) * 0.20;
				alpha = (255 / ((double)(Depth_max + depth_range - Depth_min)));
				beta = ((double)alpha) * Depth_min;
                UpdateColorMap(devHandle, Depth_min, Depth_max + depth_range, 4);
                camera_prop.push_back(std::make_pair(TOF_APP_MIN_DEPTH, Depth_min));
                camera_prop.push_back(std::make_pair(TOF_APP_MAX_DEPTH, Depth_max));
                if (pcl_display_flag)
                    pclDisplay->getColorMapProp(Depth_min, Depth_max + depth_range, 4, depthRange);
            }
            else
            {
                depth_max_val = TOF_APP_VAL_FAR_SPC_DEPTH_MAX;
                depth_min_val = TOF_APP_VAL_FAR_SPC_DEPTH_MIN;
                Depth_min = settings.value("Far_mode/Lower_limit").toInt();
                Depth_max = settings.value("Far_mode/Upper_limit").toInt();
                depth_range = (Depth_max - Depth_min) * 0.05;
                camera_prop.push_back(std::make_pair(TOF_APP_MIN_DEPTH, Depth_min));
                camera_prop.push_back(std::make_pair(TOF_APP_MAX_DEPTH, Depth_max));
                UpdateColorMap(devHandle, Depth_min, Depth_max + depth_range, 4);
                if (pcl_display_flag)
                    pclDisplay->getColorMapProp(Depth_min, Depth_max + depth_range, 4, depthRange);
            }
            if (pcl_display_flag)
                pclDisplay->getColorMapProp(Depth_min, Depth_max + depth_range, 4, depthRange);
        }
        if (GetTOFCoring(devHandle, &gvalue4) > 0) {
            camera_prop.push_back(std::make_pair(TOF_UVC_EXT_CID_CORING, gvalue4));
            gvalue4 = 0;
        }
        if (GetTOFIRGain(devHandle, &gvalue4) > 0) {
            camera_prop.push_back(std::make_pair(TOF_UVC_EXT_CID_IR_GAIN, gvalue4));
            gvalue4 = 0;
        }

	}
    if ((cam & (1 << CamProp::All)) || (cam & (1 << CamProp::ColorCam_extension)))
    {
        if (GetAntiFlickerDetection(devHandle, &gvalue1) > 0) {
            camera_prop.push_back(std::make_pair(TOF_HID_CID_ANTI_FLICKER_DETECTION, gvalue1));
            gvalue1 = 0;
        }
        if (GetSpecialEffect(devHandle, &gvalue1) > 0) {
            switch (gvalue1) {
            case TOF_APP_VAL_SPC_EFFECT_NORMAL:
                gvalue1 = 0;
                break;
            case TOF_APP_VAL_SPC_EFFECT_BW:
                gvalue1 = 1;
                break;
            case TOF_APP_VAL_SPC_EFFECT_GRAY:
                gvalue1 = 2;
                break;
            case TOF_APP_VAL_SPC_EFFECT_NEGATIVE:
                gvalue1 = 3;
                break;
            case TOF_APP_VAL_SPC_EFFECT_SKETCH:
                gvalue1 = 4;
                break;
            }
            camera_prop.push_back(std::make_pair(TOF_HID_CID_SPECIAL_EFFECT, gvalue1));
            gvalue1 = 0;
        }
        if (GetDenoise(devHandle, &gvalue1) > 0) {
            camera_prop.push_back(std::make_pair(TOF_HID_CID_DENOISE, gvalue1));
            gvalue1 = 0;
        }
        if (GetAutoExposureROI(devHandle, &gvalue1, &gvalue2) > 0) {
			PrintLog(LOG_ESSENTIAL_DEBUG, "GetAutoExposureROI : " + std::to_string(gvalue1) + " , " + std::to_string(gvalue2));
            camera_prop.push_back(std::make_pair(TOF_HID_CID_AUTO_EXP_ROI, gvalue1));
            camera_prop.push_back(std::make_pair(TOF_HID_CID_AUTO_EXP_WIN_SIZE, gvalue2));
            if (gvalue1 == TOF_APP_VAL_MANUAL_ROI) {

                yuvdisplay->setMouseTracking(true);
                yuvdisplay->installEventFilter(this);
            }
            else {
                yuvdisplay->setMouseTracking(false);
                yuvdisplay->removeEventFilter(this);
            }
            gvalue1 = 0;
            gvalue2 = 0;
        }
        if (GetOrientation(devHandle, &gvalue1) > 0) {
            camera_prop.push_back(std::make_pair(TOF_HID_CID_ORIENTATION, gvalue1));
            gvalue1 = 0;
        }
        if (GetFaceDetection(devHandle, &gvalue1, &gvalue2, &gvalue3) > 0) {
            camera_prop.push_back(std::make_pair(TOF_HID_CID_FACE_DETECTION, gvalue1));
            camera_prop.push_back(std::make_pair(TOF_HID_CID_FACE_STATUS, gvalue2));
            camera_prop.push_back(std::make_pair(TOF_HID_CID_FACE_RECT, gvalue3));
            gvalue1 = 0;
            gvalue2 = 0;
            gvalue3 = 0;
        }
        if (GetSmileDetection(devHandle, &gvalue1, &gvalue2) > 0) {
            camera_prop.push_back(std::make_pair(TOF_HID_CID_SMILE_STATUS, gvalue2));
            camera_prop.push_back(std::make_pair(TOF_HID_CID_SMILE_DETECTION, gvalue1));
            gvalue1 = 0;
            gvalue2 = 0;
        }
        if (GetIMUEmbeddedData(devHandle, &gvalue1))
		{
			camera_prop.push_back(std::make_pair(TOF_HID_CID_IMU_DATA, gvalue1));
			gvalue1 = 0;
		}
        if (GetExposureCompensation(devHandle, &gvalue5) > 0) {
            camera_prop.push_back(std::make_pair(TOF_HID_CID_EXPOSURE_COMP, gvalue5));
            gvalue5 = 0;
        }
        if (GetFrameRateCtrl(devHandle, &gvalue1) > 0) {
            camera_prop.push_back(std::make_pair(TOF_HID_CID_FPS_CTRL, gvalue1));
            gvalue1 = 0;
        }
    }
    emit campropQueried(camera_prop);
}

void MainWindow::onSetFilterType(int ctrlID, bool selected)
{
    QSettings settings(QString("config.ini"), QSettings::IniFormat);
    uint16_t num_of_pre_frames;
    num_of_pre_frames = settings.value("temporal_history/num_of_pre_frames").toInt();
    switch (ctrlID)
    {
    case TOF_APP_ENABLE_SPATIAL_FILTER:
    case TOF_APP_ENABLE_TEMPORAL_FILTER:
    case TOF_APP_ENABLE_EDGE_DETECTION:
        SetFilterType(devHandle, ctrlID - TOF_APP_ENABLE_SPATIAL_FILTER, selected);
        break;
    default:
        break;
    }
}
void MainWindow::onSetCamProperty(int ctrlID, int value)
{
    uint16_t gvalue4 = 0;
    int depth_range;
    std::vector< std::pair <int, int> > camera_prop;
    DataMode set_mode;
    uint8_t gMajorVersion = 0, gMinorVersion1 = 0;
    uint16_t gMinorVersion2 = 0, gMinorVersion3 = 0;
    uint64_t uniqueID;

    switch (ctrlID) {
    case TOF_UVC_CID_BRIGHTNESS:
    case TOF_UVC_CID_CONTRAST:
    case TOF_UVC_CID_SATURATION:
    case TOF_UVC_CID_WB_AUTO:
    case TOF_UVC_CID_GAMMA:
    case TOF_UVC_CID_GAIN:
    case TOF_UVC_CID_PWR_LINE_FREQ:
    case TOF_UVC_CID_WB_TEMP:
    case TOF_UVC_CID_SHARPNESS:
    case TOF_UVC_CID_EXPOSURE_AUTO:
    case TOF_UVC_CID_EXPSOURE_ABS:
        if(SetUVCControl(devHandle, ctrlID,value)<0){
            pError("SetUVCControl");
        }
        else
        {
            uvc_prop[ctrlID].cur = value;
        }
        break;
    case TOF_UVC_EXT_CID_DATA_MODE:
        stopStream();
        set_mode = (DataMode)((value == 0 )? value: value = value + 1);

        if(SetDataMode(devHandle, set_mode)<0){
            pError("SetDataMode");
            startStream();
        }else{
			PrintLog(LOG_ESSENTIAL_DEBUG, "SetDataMode success\n");
            data_mode = set_mode;
            if (data_mode <= DataMode::IR_Mode)
            {
                if (data_mode == DataMode::IR_Mode)
                    emit tofCamModeSelected(false);
                else
                    emit tofCamModeSelected(true);
            }
            else if (data_mode == DataMode::Depth_IR_RGB_VGA_Mode)
            {
				pclDisplay->dataModeChanged(Depth_IR_RGB_VGA_Mode);
                emit dualCamModeSelected(true);
            }
            else if (data_mode == DataMode::Depth_IR_RGB_HD_Mode)
            {
				pclDisplay->dataModeChanged(Depth_IR_RGB_HD_Mode);
                emit dualCamModeSelected(false);
            }
            else if (data_mode >= DataMode::RGB_VGA_Mode)
            {
				queryCamPropFlag = (1 << CamProp::ColorCam_uvc) | (1 << CamProp::Default);
				querycameraproperties(queryCamPropFlag);
				uint8_t frameRateCtrl = 10;
				uint8_t ROIMode, winSize;
				uint32_t expoComp = 10;
                if (GetFrameRateCtrl(devHandle, &frameRateCtrl) > 0) {
					PrintLog(LOG_ESSENTIAL_DEBUG, "Frame rate Ctrl get value is : " + std::to_string((uint16_t)frameRateCtrl));
				}
                if (GetExposureCompensation(devHandle, &expoComp) > 0) {
					PrintLog(LOG_ESSENTIAL_DEBUG, "GetExposureCompensation value is : " + std::to_string(expoComp));
				}
                if (GetAutoExposureROI(devHandle, &ROIMode, &winSize) > 0) {
					PrintLog(LOG_ESSENTIAL_DEBUG, "GetAutoExposureROI : " + std::to_string(ROIMode) + " , " + std::to_string(winSize));
					if (ROIMode == TOF_APP_VAL_MANUAL_ROI) {

						yuvdisplay->setMouseTracking(true);
						yuvdisplay->installEventFilter(this);
					}
					else {
						yuvdisplay->setMouseTracking(false);
						yuvdisplay->removeEventFilter(this);
					}
				}
                emit rgbCamModeSelected(frameRateCtrl, expoComp, ROIMode, winSize);
            }
            fps_label->setText("Current FPS:" + QString::number(0));
            if (data_mode == DataMode::IR_Mode)
            {
                widgetMaximized = false;
                onwidgetMaximized(irdisplay);
            }
			else if(data_mode >= DataMode::RGB_VGA_Mode )
            {
                widgetMaximized = false;
                onwidgetMaximized(yuvdisplay);
            }
            else
                modifyUi(data_mode);
			if (depthIntrinsicData && rgbIntrinsicData && extrinsicData)
			{
				//	Added by Abishek on 28/02/2023
				//	To pass the rgb intrinsic parameters to create 3d file after transforming the depth data to RGB plane.
				if (data_mode == DataMode::Depth_IR_RGB_VGA_Mode)
				{
					pclDisplay->get3DIntrinsic(rgb_intrinsic.at<double>(0, 0), rgb_intrinsic.at<double>(1, 1),
						rgb_intrinsic.at<double>(0, 2), rgb_intrinsic.at<double>(1, 2));
				}
				else if (data_mode == DataMode::Depth_IR_RGB_HD_Mode)
				{
					pclDisplay->get3DIntrinsic(rgbHD_intrinsic.at<double>(0, 0), rgbHD_intrinsic.at<double>(1, 1),
						rgbHD_intrinsic.at<double>(0, 2), rgbHD_intrinsic.at<double>(1, 2));
				}

			}

            startStream();
        }
		break;
    case TOF_UVC_EXT_CID_DEPTH_RANGE:
        stopStream();
        if(SetDepthRange(devHandle, value)<0){
            pError("SetDepthRange");
            startStream();
        }else{
            depthRange = value;
            camera_prop.push_back(std::make_pair(TOF_UVC_EXT_CID_DEPTH_RANGE, depthRange));

            QSettings settings(QString("config.ini"), QSettings::IniFormat);
            if(value == 0)
            {
                Depth_min = settings.value("Near_mode/Lower_limit").toInt();
                Depth_max = settings.value("Near_mode/Upper_limit").toInt();
                 //5% of the range is added to the max value so that the depth at the depth_max can be viewed
                depth_range = (Depth_max - Depth_min) * 0.20;
                UpdateColorMap(devHandle, Depth_min, Depth_max + depth_range , 4);
                if (pcl_display_flag)
                    pclDisplay->getColorMapProp(Depth_min, Depth_max + depth_range, 4, depthRange);
                camera_prop.push_back(std::make_pair(TOF_APP_MIN_DEPTH,Depth_min));
                camera_prop.push_back(std::make_pair(TOF_APP_MAX_DEPTH,Depth_max));
            }else
            {
                Depth_min = settings.value("Far_mode/Lower_limit").toInt();
                Depth_max = settings.value("Far_mode/Upper_limit").toInt();
                depth_range = (Depth_max - Depth_min) * 0.05;
                camera_prop.push_back(std::make_pair(TOF_APP_MIN_DEPTH,Depth_min ));
                camera_prop.push_back(std::make_pair(TOF_APP_MAX_DEPTH,Depth_max ));
                UpdateColorMap(devHandle, Depth_min, Depth_max + depth_range , 4);
                if (pcl_display_flag)
                    pclDisplay->getColorMapProp(Depth_min, Depth_max + depth_range, 4, depthRange);

            }
            fps_label->setText("Current FPS:" + QString::number(0));

            if (GetTOFCoring(devHandle, &gvalue4) > 0) {
                camera_prop.push_back(std::make_pair(TOF_UVC_EXT_CID_CORING, gvalue4));
                gvalue4 = 0;
            }
            if (GetTOFIRGain(devHandle, &gvalue4) > 0) {
                camera_prop.push_back(std::make_pair(TOF_UVC_EXT_CID_IR_GAIN, gvalue4));
                gvalue4 = 0;
            }

            emit depthModeChanged(camera_prop);
            modifyUi(data_mode);
            startStream();
        }
        break;

    case TOF_UVC_EXT_CID_CORING:
        if (value <= 65535)
        {
            if(SetTOFCoring(devHandle, value)<0){
                pError("SetTOFCoring");
            }
        }
        break;
    case TOF_UVC_EXT_CID_IR_GAIN:
        if(SetTOFIRGain(devHandle, value)<0){
            pError("SetTOFIRGain");
        }
        break;
    case TOF_HID_CID_ANTI_FLICKER_DETECTION:
        if(SetAntiFlickerDetection(devHandle, value)<0){
            pError("SetAntiFlickerDetection");
        }
        break;

    case TOF_HID_CID_SPECIAL_EFFECT:
        switch(value){
        case 0:
            value= TOF_APP_VAL_SPC_EFFECT_NORMAL;
            break;
        case 1:
            value= TOF_APP_VAL_SPC_EFFECT_BW;
            break;
        case 2:
            value= TOF_APP_VAL_SPC_EFFECT_GRAY;
            break;
        case 3:
            value= TOF_APP_VAL_SPC_EFFECT_NEGATIVE;
            break;
        case 4:
            value= TOF_APP_VAL_SPC_EFFECT_SKETCH;
            break;
        }

        if(SetSpecialEffect(devHandle, value)<0){
            pError("SetSpecialEffect");
        }
        break;
    case TOF_HID_CID_DENOISE:
        if(SetDenoise(devHandle, value)<0){
            pError("SetDenoise");
        }
        break;
    case TOF_HID_CID_ORIENTATION:
        if(SetOrientation(devHandle, value)<0){
            pError("SetOrientation");
        }

        break;
    case TOF_HID_CID_EXPOSURE_COMP:
        if(SetExposureCompensation(devHandle, value)<0){
            pError("SetExposureCompensation");
        }
        break;
    case TOF_HID_CID_FPS_CTRL:
        if(SetFrameRateCtrl(devHandle, value)<0){
            pError("SetFrameRateCtrl");
        }
        break;
    case TOF_APP_CID_DEPTH_REGION:
        if(SetAvgRegion(devHandle, (AvgRegion)value)<0){
            pError("SetAvgRegion");
        }

        avgRegionFlag ^= (1<<value);
        if(value==AvgRegion::CustomPtr){
            avgRegionFlag&=~(1<<AvgRegion::Center);
        }else{
            avgRegionFlag&=~(1<<AvgRegion::CustomPtr);
        }
        if(data_mode!=IR_Mode && data_mode<=Depth_IR_RGB_HD_Mode){
            if(avgRegionFlag &(1<<AvgRegion::CustomPtr)){
                depthdisplay->setMouseTracking(true);
                depthdisplay->installEventFilter(this);
            }else{
                depthdisplay->setMouseTracking(false);
                depthdisplay->removeEventFilter(this);
            }
        }
        break;
    case TOF_APP_CID_CURSOR_COLOR:
        if(SetCursorColor(devHandle, value)<0){
            pError("SetCursorColor");
        }
        break;
	case TOF_HID_CID_IMU_DATA:
        if (SetIMUEmbeddedData(devHandle, (uint8_t)value) < 0) {
			pError("SetIMUEmbeddedData");
		}
		break;
    case TOF_APP_CID_DEPTH_PLANARIZE:
        if(SetPlanarization(devHandle, value)<0){
            pError("SetPlanarization");
        }
        break;
    case TOF_APP_CID_DEPTH_X:
        x_comp = value/100.0;
        break;
    case TOF_APP_CID_DEPTH_Y:
        y_comp = value/100.0;
        break;
    case TOF_UVC_CID_SET_DEFAULT:
        for (int ctrl_ID = TOF_UVC_CID_EXPSOURE_ABS; ctrl_ID >= TOF_UVC_CID_BRIGHTNESS ; ctrl_ID--)
        {

            if ((ctrl_ID == TOF_UVC_CID_WB_TEMP || ctrl_ID == TOF_UVC_CID_WB_AUTO) && uvc_prop[TOF_UVC_CID_WB_AUTO].cur)
                continue;

            if ((ctrl_ID == TOF_UVC_CID_EXPSOURE_ABS || ctrl_ID == TOF_UVC_CID_EXPOSURE_AUTO) && !uvc_prop[TOF_UVC_CID_EXPOSURE_AUTO].cur)
                continue;

            if ((data_mode == DataMode::Depth_IR_RGB_VGA_Mode || data_mode == DataMode::Depth_IR_RGB_HD_Mode) && ctrl_ID == TOF_UVC_CID_EXPSOURE_ABS)
            {
                continue;
            }

            if(ctrl_ID == TOF_UVC_CID_EXPOSURE_AUTO)
            {
                if (SetUVCControl(devHandle, ctrl_ID, 0) < 0) {
                    pError("SetUVCControl");
                }
            }
            else
            {
                if (SetUVCControl(devHandle, ctrl_ID, uvc_prop[ctrl_ID].default_val) < 0) {
                    pError("SetUVCControl");
                }
            }


        }
        queryCamPropFlag = (1 << CamProp::ColorCam_uvc) | (1 << CamProp::Default);
        querycameraproperties(queryCamPropFlag);
        queryCamPropFlag = 1 << CamProp::All;
        break;

    case TOF_HID_CID_SET_DEFAULT:
        if (SetDefault(devHandle) < 0) {
            pError("SetCursorColor");
        }
        else
        {
            queryCamPropFlag = (1 << CamProp::ColorCam_extension) | (1 << CamProp::Default);
            querycameraproperties(queryCamPropFlag);
			queryCamPropFlag = 1 << CamProp::All;
        }
        break;
    case TOF_APP_GET_FIRMWARE_VERSION:
        ReadFirmwareVersion(devHandle, &gMajorVersion, &gMinorVersion1, &gMinorVersion2, &gMinorVersion3);
         emit firmwareVersionRead(gMajorVersion, gMinorVersion1, gMinorVersion2, gMinorVersion3);
         break;
    case TOF_APP_GET_UNIQUE_ID:
		uniqueID = 0;
        GetUniqueID(devHandle, &uniqueID);
        emit uniqueIDRead(uniqueID);
        break;
    case TOF_APP_AVERAGE_IR_DISPLAY:
        SetAvgIRDisplay(devHandle, value);
        break;
       default:
        break;
    }
}

void MainWindow::onSetCamProperty(int ctrlID, int value, int value1, int value2)
{

    switch(ctrlID){

    case TOF_HID_CID_AUTO_EXP_ROI:
        auto_roi_win_size = value1;
        if(SetAutoExposureROI(devHandle, value,yuvdisplay->size().width(),yuvdisplay->size().height(),0,0,auto_roi_win_size)<0)
        {
            pError("SetAutoExposureROI");
        }
        if(value==2){
            yuvdisplay->setMouseTracking(true);
            yuvdisplay->installEventFilter(this);
        }else{
            yuvdisplay->setMouseTracking(false);
            yuvdisplay->removeEventFilter(this);
        }
        break;
    case TOF_HID_CID_FACE_DETECTION:
        if(SetFaceDetection(devHandle, value,value1,value2)<0){
            pError("SetFaceDetection");
        }
        break;
    case TOF_HID_CID_SMILE_DETECTION:
        if(SetSmileDetection(devHandle, value,value1)<0){
            pError("SetSmileDetection");
        }
        break;
    case TOF_APP_CID_SPC_DEPTH_RANGE:
        if(value){
            spc_ply_flag = true;
            depth_min_val = value1;
            depth_max_val = value2;
        }else{
            spc_ply_flag = false;
            if(depthRange == 0) {
                depth_max_val = TOF_APP_VAL_NEAR_SPC_DEPTH_MAX;
                depth_min_val = TOF_APP_VAL_NEAR_SPC_DEPTH_MIN;
            }
            else {
                depth_max_val = TOF_APP_VAL_FAR_SPC_DEPTH_MAX;
                depth_min_val = TOF_APP_VAL_FAR_SPC_DEPTH_MIN;
            }
        }
        break;
    }
}
void MainWindow::modifySize(DataMode dataMode)
{
    QScreen* screen = QGuiApplication::primaryScreen();
    QRect mm = screen->availableGeometry();
    int screen_width_mm = mm.width();
    int screen_height_mm = mm.height();

    int screen_width = this->width();
    int screen_height = this->height();
    windowheight = (int)screen_height_mm * 0.9;
    windowwidth = (int)screen_width_mm * 0.75;

    switch(dataMode){
        case Depth_IR_Mode:
            widgetheight = windowheight/2.00;
            widgetwidth = widgetheight*vga_aspect_ratio;
            depthdisplay->setMinimumSize(widgetwidth,widgetheight);
            irdisplay->setMinimumSize(widgetwidth,widgetheight);
            if (pcl_display_flag)
                pclDisplay->setMinimumSize(widgetwidth,widgetheight);
            break;
        case Depth_Mode:
            widgetheight = windowheight / 2.00;
            widgetwidth = widgetheight * vga_aspect_ratio;
            depthdisplay->setMinimumSize(widgetwidth,widgetheight);
            if (pcl_display_flag)
                pclDisplay->setMinimumSize(widgetwidth,widgetheight);
            break;
        case IR_Mode:
            irdisplay->setMinimumSize(PRE_DEPTH_IR_WIDTH,PRE_DEPTH_IR_HEIGHT);
            break;
        case Depth_IR_RGB_VGA_Mode:
            widgetheight = windowheight/2.00;
            widgetwidth = widgetheight*vga_aspect_ratio;
			depthdisplay->setMinimumSize(widgetwidth, widgetheight);

            irdisplay->setMinimumSize(widgetwidth,widgetheight);
            if (pcl_display_flag)
                pclDisplay->setMinimumSize(widgetwidth,widgetheight);
            yuvdisplay->setMinimumSize(widgetwidth,widgetheight);
            break;
        case Depth_IR_RGB_HD_Mode:
            widgetheight = windowheight/2.00;
            widgetwidth = widgetheight*vga_aspect_ratio;

            depthdisplay->setMinimumSize(widgetwidth,widgetheight);
            if (pcl_display_flag)
                pclDisplay->setMinimumSize(widgetwidth,widgetheight);
            irdisplay->setMinimumSize(widgetwidth,widgetheight);
			if (rgbDMapping_flag)
			{
				widgetheight = widgetwidth / hd_aspect_ratio;
				depthdisplay->setMinimumSize(widgetwidth, widgetheight);
			}
			else
			{
				depthdisplay->setMinimumSize(widgetwidth, widgetheight);
			}
            widgetheight = widgetwidth/hd_aspect_ratio;
            yuvdisplay->setMinimumSize(widgetwidth,widgetheight);
            break;
        case RGB_VGA_Mode:
            yuvdisplay->setMinimumSize(PRE_RGB_VGA_WIDTH,PRE_RGB_VGA_HEIGHT);
            break;
        case RGB_HD_Mode:
            widgetwidth = (windowwidth * 0.8);
            widgetheight = widgetwidth/hd_aspect_ratio;
            yuvdisplay->setMinimumSize(widgetwidth,widgetheight);
            break;
        case RGB_Full_HD_Mode:
            widgetwidth = (windowwidth * 0.8);
            widgetheight = widgetwidth/full_hd_aspect_ratio;
            yuvdisplay->setMinimumSize(widgetwidth,widgetheight);
            break;
        case RGB_1200p_Mode:
            widgetwidth = (windowwidth * 0.8);
            widgetheight = widgetwidth/rgb_1200p_aspect_ratio;
            yuvdisplay->setMinimumSize(widgetwidth,widgetheight);
            break;

    }
}

void MainWindow::modifyUi(DataMode dataMode)
{
    modifySize(dataMode);
    switch(dataMode){
    case Depth_IR_Mode:
        yuvdisplay->setVisible(false);
        irdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*2));
        irdisplay->setVisible(true);
        depthdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*3));
        depthdisplay->setVisible(true);
        if (pcl_display_flag)
        {
            pclDisplay->setInitialPos();
            pclDisplay->setVisible(true);
        }
        break;
    case Depth_Mode:
        yuvdisplay->setVisible(false);
        irdisplay->setVisible(false);
        depthdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*3));
        depthdisplay->setVisible(true);
        if (pcl_display_flag)
        {
            pclDisplay->setInitialPos();
            pclDisplay->setVisible(true);
        }
        break;
    case IR_Mode:
        yuvdisplay->setVisible(false);
        depthdisplay->setVisible(false);
        if (pcl_display_flag)
            pclDisplay->setVisible(false);
        irdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*2));
        irdisplay->setVisible(true);
        break;
    case Depth_IR_RGB_VGA_Mode:
        irdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*2));
        irdisplay->setVisible(true);
        depthdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*3));
        depthdisplay->setVisible(true);
        yuvdisplay->InitDrawBuffer(PRE_RGB_VGA_WIDTH*PRE_RGB_VGA_HEIGHT*2);
        yuvdisplay->setVisible(true);
        if (pcl_display_flag)
        {
            pclDisplay->setInitialPos();
			pclDisplay->InitDrawBuffer((PRE_RGB_VGA_WIDTH*PRE_RGB_VGA_HEIGHT * 3));
            pclDisplay->setVisible(true);
        }
        break;
    case Depth_IR_RGB_HD_Mode:
        irdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT*2));
        irdisplay->setVisible(true);
		if (rgbDMapping_flag )
		{
			depthdisplay->InitDrawBuffer((PRE_RGB_HD_WIDTH*PRE_RGB_HD_HEIGHT * 3));
			depthdisplay->setVisible(true);
		}
		else
		{
			PrintLog(LOG_ESSENTIAL_DEBUG, "rgbMapping_flag is false\n");
			depthdisplay->InitDrawBuffer((PRE_DEPTH_IR_WIDTH*PRE_DEPTH_IR_HEIGHT * 3));
			depthdisplay->setVisible(true);
		}
        yuvdisplay->InitDrawBuffer(PRE_RGB_HD_WIDTH*PRE_RGB_HD_HEIGHT*2);
        yuvdisplay->setVisible(true);
        if (pcl_display_flag)
        {
            pclDisplay->setInitialPos();
			pclDisplay->InitDrawBuffer((PRE_RGB_HD_WIDTH*PRE_RGB_HD_HEIGHT * 3));
            pclDisplay->setVisible(true);
        }
        break;
    case RGB_VGA_Mode:
        depthdisplay->setVisible(false);
        irdisplay->setVisible(false);
        if (pcl_display_flag)
            pclDisplay->setVisible(false);
        yuvdisplay->InitDrawBuffer(PRE_RGB_VGA_WIDTH*PRE_RGB_VGA_HEIGHT*2);
        yuvdisplay->setVisible(true);
        break;
    case RGB_HD_Mode:
        depthdisplay->setVisible(false);
        irdisplay->setVisible(false);
        if (pcl_display_flag)
            pclDisplay->setVisible(false);
        yuvdisplay->InitDrawBuffer(PRE_RGB_HD_WIDTH * PRE_RGB_HD_HEIGHT * 2);
        yuvdisplay->setVisible(true);
        break;
    case RGB_Full_HD_Mode:
        depthdisplay->setVisible(false);
        irdisplay->setVisible(false);
        if (pcl_display_flag)
            pclDisplay->setVisible(false);
        yuvdisplay->InitDrawBuffer(PRE_RGB_FULL_HD_WIDTH * PRE_RGB_FULL_HD_HEIGHT * 2);
        yuvdisplay->setVisible(true);
        break;
    case RGB_1200p_Mode:
        depthdisplay->setVisible(false);
        irdisplay->setVisible(false);
        if (pcl_display_flag)
            pclDisplay->setVisible(false);
        yuvdisplay->InitDrawBuffer(PRE_RGB_ORIGINAL_WIDTH * PRE_RGB_ORIGINAL_HEIGHT * 2);
        yuvdisplay->setVisible(true);
        break;
    default:
        yuvdisplay->setVisible(false);
        depthdisplay->setVisible(false);
        irdisplay->setVisible(false);
        if (pcl_display_flag)
            pclDisplay->setVisible(false);
        break;

    }
    if (pcl_display_flag_1 == false)
    {
        pclDisplay->setVisible(false);
    }

}

void MainWindow::onInputSelected(int index)
{
	deviceOpened = true;
    showIMUFlag = false;
    if (index != oldindex)
	{
        stopStream();
        for(int itr = 0 ; itr < enumeratedHandleList.size() ; itr++)
        {
            CloseDevice(enumeratedHandleList[itr]);
        }
        emit tofSettingsDefault();
        pclDisplay->get3DIntrinsic(LENS_FOCUS_POINT_VGA_X, LENS_FOCUS_POINT_VGA_Y,
            LENS_PRINCIPLE_AXIS_VGA_X, LENS_PRINCIPLE_AXIS_VGA_Y);
        if (OpenDevice(gDevicesList[index - 1], &devHandle) < 0) {
          emit deviceRemoved();
			pError("OpenDevice:");
			return;
		}
		strcpy(openDeviceHandle.serialNo, devHandle.serialNo);
        RegisterFrameCallback(devHandle, std::bind(&MainWindow::callback, this, std::placeholders::_1, std::placeholders::_2));
        RegisterNotificationCallback(devHandle, std::bind(&MainWindow::notification_callback, this, std::placeholders::_1, std::placeholders::_2));
		stopStream();
		queryCamPropFlag = 1 << CamProp::All;
		querycameraproperties(queryCamPropFlag);
		modifyUi(data_mode);
		startStream();
		oldindex = index;
        Result readCalibRes = GetDeviceCalibrationParams(devHandle, &devCalibrationData);
        if(readCalibRes == Result::Ok)
        {
            deviceCalibData = true;
        }
		lIMUConfig.IMU_MODE = IMU_ACC_GYRO_ENABLE;
		lIMUConfig.ACC_AXIS_CONFIG = IMU_ACC_X_Y_Z_ENABLE;
		lIMUConfig.IMU_ODR_CONFIG = IMU_ODR_104HZ;
		lIMUConfig.ACC_SENSITIVITY_CONFIG = IMU_ACC_SENS_2G;
		lIMUConfig.GYRO_AXIS_CONFIG = IMU_GYRO_X_Y_Z_ENABLE;
		lIMUConfig.GYRO_SENSITIVITY_CONFIG = IMU_GYRO_SENS_250DPS;


#ifdef __linux__
        usleep(500 * 1000);
#elif _WIN32
        Sleep(500);
#endif
        uint8_t uStatus = SetIMUConfig(devHandle, lIMUConfig);
        if (!uStatus)
        {
            std::cout << "Set IMU Config Failed\r\n";
        }
        else
        {
            uStatus = GetIMUConfig(devHandle, &lIMUConfig);
            if (!uStatus)
            {
                std::cout << "Get IMU Config Failed\r\n";
            }
            glIMU_Interval = GetIMUIntervalTime(lIMUConfig); //Only gyro and acc enabled are used
        }
        lIMUInput.imu_update_mode = IMU_CONT_UPDT_EN;
        lIMUInput.imu_num_of_values = IMU_AXES_VALUES_MIN;
        uStatus = ControlIMUCapture(devHandle, &lIMUInput);
        if (uStatus == false)
        {
            std::cout << "Control IMU Capture  Failed\r\n";
        }
        else
        {
            glIMUInput = lIMUInput;
        }
        if(lIMUOutput == NULL)
        {
            lIMUOutput = (IMUDATAOUTPUT_TypeDef*)malloc(IMU_AXES_VALUES_MAX * sizeof(IMUDATAOUTPUT_TypeDef));
            if (lIMUOutput == NULL)
            {
                std::cout << "Memory Allocation for output failed\n";
            }
        }
    }
}

void MainWindow::getFrame()
{
	if (!stopFlag) {

        if (GetNextFrame(devHandle) == Result::Ok) {

			saveDepthFrame_start = saveDepthFrame;
            saveIRFrame_start = saveIRFrame;
            saveRGBFrame_start = saveRGBFrame;
            savePLY_start = savePLY;
            saveRawFrame_start = saveRawFrame;

            if (startedSavingFrames && (saveDepthFrameCount == 0 && saveIRFrameCount == 0 && saveRGBFrameCount == 0 && saveRawFrameCount == 0 && savePLYFrameCount == 0 && saved_ply_flag && plyImageSaveStatus))
            {
                emit savingFramesComplete();
                startedSavingFrames = false;
			}
            if (startedSavingFrames && (saveDepthFrameCount == 0 && saveIRFrameCount == 0 && saveRGBFrameCount == 0 && saveRawFrameCount == 0 && savePLYFrameCount == 0 && saved_ply_flag && !plyImageSaveStatus))
            {
                emit othersavedoneplyfail();
                plyImageSaveStatus = true;
                startedSavingFrames = false;
            }

            QString TimeStamp = QDateTime::currentDateTime().toString("yyyy_MM_dd_hh_mm_ss_zzz");

            if (data_mode >= Depth_IR_RGB_VGA_Mode) {
				if (yuvdisplay->isVisible() || saveRGBFrame_start) {
                    if (GetToFFrame(devHandle, FrameType::RGBFrame, &ToFRGBFrame) > 0) {
                        memcpy(UYVY_frame.data, ToFRGBFrame.frame_data, ToFRGBFrame.size);
                        if (yuvdisplay->isVisible())
                        {
							yuvdisplay->DisplayVideoFrame((unsigned char*)ToFRGBFrame.frame_data, ToFRGBFrame.width, ToFRGBFrame.height);
						}

						if (saveRGBFrame_start && saveCorrespondingRGB )
                        {
							cv::cvtColor(UYVY_frame, UYVY_frame_cpy, cv::COLOR_YUV2BGR_UYVY);
                            if (saveRGBFrameCount > 0)
                            {
								QString RGBFileName;
                                RGBFileName = saveRGBDir;
                                QTextStream(&RGBFileName) << "/DepthVista_RGB" << "_" << TimeStamp << ".bmp";
                                QByteArray name_temp = RGBFileName.toLocal8Bit();
                                std::string str = std::string(name_temp);
                                /* convert string to char**/
                                char* file_name = (char*)str.c_str();
                                cv::imwrite(file_name, UYVY_frame_cpy);
                                saveRGBFrameCount--;
                                if (savePLY)
									saveCorrespondingRGB = false;
							}
                            else
                            {
								saveRGBFrame = false;
                                saveRGBFrame_start = false;
							}
                        }
					}
                }
            }

            if (data_mode != IR_Mode && data_mode <= Depth_IR_RGB_HD_Mode)
            {
                if (GetToFFrame(devHandle, FrameType::DepthColorMap, &ToFDepthColorMapFrame) > 0)
				{
					Depthcolormap.data = (uchar*)ToFDepthColorMapFrame.frame_data;
                    if (depthdisplay->isVisible() || saveDepthFrame_start) {

								depthdisplay->DisplayVideoFrame((unsigned char*)Depthcolormap.data, Depthcolormap.cols, Depthcolormap.rows);

                        if (saveDepthFrame_start)
                        {
							Depthcolormap_cpy = Depthcolormap.clone();
                            if (saveDepthFrameCount > 0)
                            {
								QString DepthFileName;
                                DepthFileName = saveDepthDir;
                                struct tm* tm;
                                time_t t = time(0);
                                tm = localtime(&t);
                                QTime millisecond;
                                QTextStream(&DepthFileName) << "/DepthVista_Depth" << "_" << TimeStamp << ".bmp";
                                QByteArray name_temp = DepthFileName.toLocal8Bit();
                                std::string str = std::string(name_temp);
                                /* convert string to char**/
                                char* file_name = (char*)str.c_str();
                                cv::imwrite(file_name, Depthcolormap_cpy);
								saveDepthFrameCount--;
                            }
                            else
                            {
								saveDepthFrame = false;
                                saveDepthFrame_start = false;
                            }
                        }
                    }
                }

                if ((pcl_display_flag && pclDisplay->isVisible()) || savePLY_start || saveRawFrame_start) {
                    if (GetToFFrame(devHandle, FrameType::DepthRawFrame, &ToFDepthRawFrame) > 0) {

                            memcpy(DepthImg_Y16.data, ToFDepthRawFrame.frame_data, ToFDepthRawFrame.size);
                        if (rgbDMapping_flag && (data_mode == Depth_IR_RGB_VGA_Mode || data_mode == Depth_IR_RGB_HD_Mode))
                        {
                            if (GetToFFrame(devHandle, FrameType::RGBFrame, &ToFRGBFrame) > 0)
                            {
                                memcpy(UYVY_frame.data, ToFRGBFrame.frame_data, ToFRGBFrame.size);
                            }

							if (data_mode == Depth_IR_RGB_VGA_Mode)
							{
								cv::cvtColor(UYVY_frame, rgbFrame, cv::COLOR_YUV2BGR_UYVY);
								pclDisplay->DisplayVideoFrame((unsigned char*)ToFDepthRawFrame.frame_data, (unsigned char*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows);

							}
							else if (data_mode == Depth_IR_RGB_HD_Mode)
							{
								try
								{

									cv::cvtColor(UYVY_frame, rgbFrame, cv::COLOR_YUV2BGR_UYVY);
									pclDisplay->DisplayVideoFrame((unsigned char*)DepthImg_Y16.data, (unsigned char*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows);

								}
								catch(...)
								{
									PrintLog(LOG_ESSENTIAL_DEBUG, "registerDepthFailed");
								}
							}
                        }
						else
						{
								pclDisplay->DisplayVideoFrame((unsigned char*)DepthImg_Y16.data, NULL, ToFDepthRawFrame.width, ToFDepthRawFrame.height);
                        }
                        if (saveRawFrame_start && saveCorrespondingRaw)
                        {
                            if (saveRawFrameCount > 0)
                            {
                                QString saveRawFileName;
                                saveRawFileName = saveRAWDir;
                                QTextStream(&saveRawFileName) << "/DepthVista_Raw" << "_" << TimeStamp << ".raw";
                                std::string str = saveRawFileName.toStdString();
                                /* convert string to char**/
                                const char* file_name = str.c_str();
                                FILE* fp;
                                fp = fopen(file_name, "wb+");
                                fwrite(DepthImg_Y16.data, DepthImg_Y16.total() * DepthImg_Y16.channels() * DepthImg_Y16.elemSize1(), 1, fp);
                                fclose(fp);

                                saveRawFrameCount--;
                                if (savePLY)
                                    saveCorrespondingRaw = false;
                            }
                            else
                            {
                                saveRawFrame = false;
                                saveRawFrame_start = false;
                            }
                        }
						if (savePLY_start)
                        {
							savingRaw++;
                            QString savePLYFileName;
                            if (savePLYFrameCount > 0)
                            {
								savePLYFileName = save3dDir;
                                QTextStream(&savePLYFileName) << "/DepthVista_PLY" << "_" << TimeStamp << ".ply";
                                savePLYfile_name = savePLYFileName;
                                pclDisplay->setSavePLYfile(savePLYfile_name, spc_ply_flag, depth_min_val, depth_max_val);
                                saved_ply_flag = false;
                                savePLYFrameCount--;
                                save_ply_flag = true;
                                savingPLY = true;
								if (saveDepthFrame)
									saveCorrespondingDepth = true;
								if (saveIRFrame)
                                    saveCorrespondingIR = true;
								if (saveRGBFrame)
									saveCorrespondingRGB = true;
								if (saveRawFrame)
									saveCorrespondingRaw = true;
                            }
                            else
                            {
								savePLY = false;
                                savePLY_start = false;
							}
						}
                    }
				}
			}

			if (data_mode != Depth_Mode && data_mode <= Depth_IR_RGB_HD_Mode)
			{
				if (irdisplay->isVisible() || saveIRFrame_start) {
                    if (GetToFFrame(devHandle, FrameType::IRPreviewFrame, &ToFIrFrame) > 0) {
						if(irdisplay->isVisible())
							irdisplay->DisplayVideoFrame((unsigned char*)ToFIrFrame.frame_data, ToFIrFrame.width, ToFIrFrame.height);
                    if (saveIRFrame_start /*&& saveCorrespondingIR*/ )
                    {
                        memcpy(IRImg_cpy.data, ToFIrFrame.frame_data, ToFIrFrame.size);
                        if (saveIRFrameCount > 0)
                        {
							QString IRFileName;
                            IRFileName = saveIRDir;
                            struct tm* tm;
                            time_t t = time(0);
                            tm = localtime(&t);
                            QTime millisecond;
                            QTextStream(&IRFileName) << "/DepthVista_IR" << "_"<< TimeStamp << ".png";
                            QByteArray name_temp = IRFileName.toLocal8Bit();
                            std::string str = std::string(name_temp);
                            /* convert string to char**/
                            char* file_name = (char*)str.c_str();
                            cv::imwrite(file_name, IRImg_cpy);
                            saveIRFrameCount--;
						}
                        else
                        {
							savingRaw = 0;
                            saveIRFrame = false;
                            saveIRFrame_start = false;
						}
                    }
					}
                }
			}
        }
    }

}

void MainWindow::framesPerSecondDisplay()
{
	while (1)
	{
		if (!stopFlag)
        {
#ifdef __linux__
        usleep(10000);
#elif _WIN32
        Sleep(10);
#endif
            fps_label->setText("Current FPS:" + QString::number(GetFramesPerSecond(devHandle)));
            if(showIMUFlag)
            {
                if (GetIMUValue(devHandle, lIMUOutput))
                {
                    getInclination(lIMUOutput->gyroX, lIMUOutput->gyroY, lIMUOutput->gyroZ,
                        lIMUOutput->accX, lIMUOutput->accY, lIMUOutput->accZ);
                }
            }
        }
		else
			break;
	}
}
void MainWindow::callback(int a, int deviceIndex)
{
    if(!previewRenderThread.isRunning()){
        previewRenderThread = QtConcurrent::run(this,&MainWindow::getFrame);
    }

	if(!fpsDisplayThread.isRunning()){
		fpsDisplayThread = QtConcurrent::run(this,&MainWindow::framesPerSecondDisplay);
    }
}

void MainWindow::notification_callback(int a, int deviceIndex)
{
	if (deviceIndex == oldindex - 1)
	{
		if (a == 0 || a == 2)
		{
			CloseDevice(devHandle);
		}
		stopStream();

		emit tofSettingsDefault();
		enumerateDevices();
		emit deviceRemoved();
		fps_label->setText("Current FPS:0");

		calibParamObtained = false;
		depthIntrinsicData = false;
		rgbIntrinsicData = false;
		extrinsicData = false;

		yuvdisplay->setVisible(false);
		depthdisplay->setVisible(false);
		irdisplay->setVisible(false);
		if (pcl_display_flag) {
			pclDisplay->setVisible(false);
		}


		oldindex--;
		if (a == 2)
		{
#ifdef __linux__
        usleep(1000 * 5000);
#elif _WIN32
        Sleep(5000);
#endif
			emit framesStopped(oldindex + 1);
		}
		char nulArray[15] = "NODEVICEFOUND";
		strcpy(openDeviceHandle.serialNo, nulArray);
	}


}


void MainWindow::startStream()
{
    if (data_mode == Depth_IR_RGB_HD_Mode || data_mode == RGB_HD_Mode)
    {
        UYVY_frame.release();
        UYVY_frame = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_8UC2);
		if (data_mode == Depth_IR_RGB_HD_Mode)
		{
			if (rgbDMapping_flag )
			{
				Depthcolormap.release();
				Depthcolormap = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_8UC3);
				DepthImg_Y16.release();
				DepthImg_Y16 = cv::Mat(PRE_RGB_HD_HEIGHT, PRE_RGB_HD_WIDTH, CV_16UC1);
			}
			else
			{
				Depthcolormap.release();
				Depthcolormap = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC3);
				DepthImg_Y16.release();
				DepthImg_Y16 = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_16UC1);
			}
		}
    }
    else if (data_mode == RGB_Full_HD_Mode)
    {
        UYVY_frame.release();
        UYVY_frame = cv::Mat(PRE_RGB_FULL_HD_HEIGHT, PRE_RGB_FULL_HD_WIDTH, CV_8UC2);
    }
    else if (data_mode == RGB_1200p_Mode)
    {
        UYVY_frame.release();
        UYVY_frame = cv::Mat(PRE_RGB_ORIGINAL_HEIGHT, PRE_RGB_ORIGINAL_WIDTH, CV_8UC2);
    }
    else if (data_mode <= Depth_IR_RGB_VGA_Mode || data_mode == RGB_VGA_Mode)
    {
        UYVY_frame.release();
        UYVY_frame = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC2);
		Depthcolormap.release();
		Depthcolormap = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_8UC3);
		DepthImg_Y16.release();
		DepthImg_Y16 = cv::Mat(PRE_RGB_VGA_HEIGHT, PRE_RGB_VGA_WIDTH, CV_16UC1);
    }
    stopFlag = false;
}

void MainWindow::stopStream()
{
    stopFlag = true;
    count = 0;

    if(previewRenderThread.isRunning()){
        previewRenderThread.waitForFinished();
    }
    if(savePLYThread.isRunning()){
        savePLYThread.waitForFinished();
    }
	if (!fpsDisplayThread.isRunning()) {
		fpsDisplayThread.waitForFinished();
	}
    yuvdisplay->StopFrame();
    depthdisplay->StopFrame();
    irdisplay->StopFrame();

}
MainWindow::~MainWindow()
{
    CloseDevice(devHandle);
    stopStream();
    if(yuvdisplay!=NULL){
        delete yuvdisplay;
    }

    if(irdisplay!=NULL){
        delete irdisplay;
    }

    if(depthdisplay!=NULL){
        delete depthdisplay;
    }
    if (pcl_display_flag)
    {
        if (pclDisplay != NULL) {
            delete pclDisplay;
        }
    }
    if(lIMUOutput)
    {
        free(lIMUOutput);
        lIMUOutput = NULL;
    }

    delete ui;
}
bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if(watched==yuvdisplay && event->type() == QEvent::MouseButtonRelease){
         QMouseEvent* mEvent = (QMouseEvent*)event;
         emit setMousePointer(mEvent->x(),mEvent->y(),TOF_HID_CID_AUTO_EXP_ROI);
    }
    else if(watched == depthdisplay && event->type() == QEvent::MouseMove){
        QMouseEvent* mEvent = (QMouseEvent*)event;
        emit setMousePointer(mEvent->x(),mEvent->y(),TOF_APP_CID_DEPTH_REGION);
    }
    else if(watched == depthdisplay && event->type() == QEvent::MouseButtonRelease)
    {
        depth_mouse_clicked = !depth_mouse_clicked;
    }else if(pcl_display_flag && watched == pclDisplay && event->type() == QEvent::MouseButtonDblClick){
        onwidgetMaximized((QWidget*)pclDisplay);
    }
    return false;
}
void MainWindow::onwidgetMaximized(QWidget* widget)
{
    int heightMaxi=0, widthMaxi=0;

    if(!widgetMaximized){

        widgetMaximized = true;
        modifyUi((DataMode)-1);
        heightMaxi = windowheight * (1 - 0.13);
        widthMaxi = windowwidth * (1 - 0.15);
        if(widget==yuvdisplay && (data_mode == RGB_HD_Mode ||data_mode == Depth_IR_RGB_HD_Mode )){
            heightMaxi = widthMaxi /hd_aspect_ratio;
            widget->setMinimumSize(widthMaxi, heightMaxi);
        }
        else if (widget == yuvdisplay && (data_mode == RGB_Full_HD_Mode))
        {
            heightMaxi = widthMaxi / full_hd_aspect_ratio;
            widget->setMinimumSize(widthMaxi, heightMaxi);
        }
        else if (widget == yuvdisplay && (data_mode == RGB_1200p_Mode))
        {
            heightMaxi = widthMaxi / rgb_1200p_aspect_ratio;
            widget->setMinimumSize(widthMaxi, heightMaxi);
        }
		else if (widget == depthdisplay && (data_mode == Depth_IR_RGB_HD_Mode && rgbDMapping_flag))
		{
			heightMaxi = widthMaxi / full_hd_aspect_ratio;
			widget->setMinimumSize(widthMaxi, heightMaxi);
		}
        else
		{
            widthMaxi = heightMaxi *vga_aspect_ratio;
            widget->setMinimumSize(widthMaxi, heightMaxi);
        }
        widget->setVisible(true);
    }else{
        widgetMaximized = false;
        modifyUi(data_mode);
    }

}

void MainWindow::onSetMousePointer(int x, int y, int ctrlID)
{
    if(ctrlID == TOF_HID_CID_AUTO_EXP_ROI){
        if(SetAutoExposureROI(devHandle, TOF_APP_VAL_MANUAL_ROI,yuvdisplay->size().width(),yuvdisplay->size().height(),x,y,auto_roi_win_size))
        {
            pError("SetAutoExposureROI");
        }

    }else if(ctrlID == TOF_APP_CID_DEPTH_REGION){
        if(!depth_mouse_clicked){
			double outputXHigh;
			double outputYHigh;
			uint32_t frame_width;
			uint32_t frame_height;
			if (data_mode == Depth_IR_RGB_HD_Mode && rgbDMapping_flag)
			{
				frame_width = PRE_RGB_HD_WIDTH;
				frame_height = PRE_RGB_HD_HEIGHT;
				outputXHigh = PRE_RGB_HD_WIDTH;
				outputYHigh = PRE_RGB_HD_HEIGHT;
			}
			else
			{
				outputXHigh = PRE_DEPTH_IR_WIDTH;
				outputYHigh = PRE_DEPTH_IR_HEIGHT;
				frame_width = PRE_DEPTH_IR_WIDTH;
				frame_height = PRE_DEPTH_IR_HEIGHT;
			}
            double inputXHigh = depthdisplay->size().width()-1;
            double inputXCord = x;
            int outputXCord = (inputXCord / inputXHigh) * outputXHigh;
            double inputYHigh = depthdisplay->size().height()-1;
            double inputYCord = y;
            int outputYCord = (inputYCord / inputYHigh) * outputYHigh;
            DepthPtr pos;
            pos.X = outputXCord;
            pos.Y = outputYCord;

			if (!(pos.X < 0 || pos.Y < 0 || pos.X >= frame_width || pos.Y >= frame_height))
			{
                SetDepthPos(devHandle, pos);
			}
        }
    }
}
void MainWindow::ongetDepthIRdata()
{
    int avgDepth,stdDepth,avgIR,stdIR;
    if(GetDepthIRValues(devHandle, &avgDepth,&stdDepth,&avgIR,&stdIR)<0){
        pError("GetDepthIRValues");
        return;
    }
    emit updateData(avgDepth,stdDepth,avgIR,stdIR);
}

void MainWindow::onSavePLYClicked(QString saveDir)
{
    if(!save_ply_flag){
        savePLYfile_name =  saveDir;
        struct tm *tm;
        time_t t = time(0);
        tm = localtime(&t);
        QTextStream(&savePLYfile_name) << "/DepthVista" << "_" << tm->tm_mday << "_" << tm->tm_mon + 1 << "_" << tm->tm_year + 1900
            << "_" << tm->tm_hour << "_" << tm->tm_min << "_" << tm->tm_sec << ".ply";
    }
}

void MainWindow::onApplyAvgChangesClicked(int avg_x, int avg_y)
{
    UpdateAvgXandY(devHandle, avg_x, avg_y);
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    float scaling_factor;
    emit windowResized();

    QScreen* screen = QGuiApplication::primaryScreen();
    QRect mm = screen->availableGeometry();
    int screen_width = mm.width();
    int screen_height = mm.height();

    int width_percent_30 = (int)screen_width * 0.25;
    int height_percent_90 = (int)screen_height * 0.9;
    side_menu->setMinimumWidth(width_percent_30);
    side_menu->setMaximumWidth(width_percent_30);

    side_menu->setMaximumHeight(height_percent_90);
    if(!widgetMaximized)
		modifySize(data_mode);
    ui->slide_menu_button->setMinimumSize(QSize(screen_width * 0.021, screen_height * 0.078));
    ui->slide_menu_button->setMaximumSize(QSize(screen_width * 0.021, screen_height * 0.078));
    ui->slide_menu_button->setIconSize(QSize(screen_width * 0.021, screen_height * 0.078));
}

void MainWindow::savingPLYFramesOver(int plySaveStatus)
{
    plySaveStatus==0 ? plyImageSaveStatus=1 : plyImageSaveStatus=0;
    saved_ply_flag = true;
}

void MainWindow::onPCLDisplay_checkBox_stateChange(bool state)
{

    if (state == false)
    {
        pcl_display_flag_1 = false;
        modifyUi((DataMode)-1);
        modifyUi(data_mode);
    }
    else
    {
        modifyUi((DataMode)-1);
        modifyUi(data_mode);
        pclDisplay->setVisible(true);
        pcl_display_flag_1 = true;
    }
	widgetMaximized = false;
}

void MainWindow::onRGBDMapping_checkBox_stateChange(int state)
{
	stopStream();

	if (state)
	{
        if (deviceCalibData)
        {
//                Added by Abishek on 28/02/2023
//                To pass the rgb intrinsic parameters to create 3d file after transforming the depth data to RGB plane.
            if (data_mode == DataMode::Depth_IR_RGB_VGA_Mode)
            {
                pclDisplay->get3DIntrinsic(devCalibrationData.rgbCamVGAIntrinsic.fx, devCalibrationData.rgbCamVGAIntrinsic.fy,
                    devCalibrationData.rgbCamVGAIntrinsic.cx, devCalibrationData.rgbCamVGAIntrinsic.cy);
            }
            else if (data_mode == DataMode::Depth_IR_RGB_HD_Mode)
            {
                pclDisplay->get3DIntrinsic(devCalibrationData.rgbCamHDIntrinsic.fx, devCalibrationData.rgbCamHDIntrinsic.fy,
                    devCalibrationData.rgbCamHDIntrinsic.cx, devCalibrationData.rgbCamHDIntrinsic.cy);
            }

            Result res = SetRGBDMapping(devHandle, state);
			rgbDMapping_flag = state;
        }
	}
	else
	{
		//	Added by Abishek on 28/02/2023
		//	To pass the depth lens intrinsic parameters to create 3d file.
        if (deviceCalibData)
        {
            if (data_mode == DataMode::Depth_IR_RGB_VGA_Mode)
            {
                pclDisplay->get3DIntrinsic(devCalibrationData.depthCamVGAIntrinsic.fx, devCalibrationData.depthCamVGAIntrinsic.fy,
                    devCalibrationData.depthCamVGAIntrinsic.cx, devCalibrationData.depthCamVGAIntrinsic.cy);
            }
            else if (data_mode == DataMode::Depth_IR_RGB_HD_Mode)
            {
                pclDisplay->get3DIntrinsic(devCalibrationData.depthCamHDIntrinsic.fx, devCalibrationData.depthCamHDIntrinsic.fy,
                    devCalibrationData.depthCamHDIntrinsic.cx, devCalibrationData.depthCamHDIntrinsic.cy);
            }
            SetRGBDMapping(devHandle, state);
            rgbDMapping_flag = state;
        }
	}
	modifyUi(data_mode);
	startStream();
}


void MainWindow::onUndistort_checkBox_stateChange(int state)
{
    if (SetUnDistortion(devHandle, state) < 0)
	{
		perror("Calibration parameters Not Found");
	}

}

void MainWindow::onFlyingPixelToggled(int state)
{
    if (SetFlyingPixelFilter(devHandle, state) < 0)
	{
		perror("Calibration parameters Not Found");
	}

}

void MainWindow::onIMUValshowToggled(bool state)
{
    if(state)
    {
        showIMUFlag = true;
    }
    else
    {
        showIMUFlag = false;
    }
}

void MainWindow::onStartSavingFrames(bool depth, bool ir, bool rgb, bool rawDepth, bool PLY,  QString save_frame_dir)
{
    if(!QDir(save_frame_dir).exists()){
        //QMessageBox msgBox;
        //msgBox.setText("Path does not exist");
        //msgBox.setIcon(QMessageBox::Warning);
        //int result = msgBox.exec();
        //if(result == QMessageBox::Ok) {
        //    msgBox.close();
        //}
		QMessageBox *msgBox = new QMessageBox(this);
		msgBox->setWindowTitle("DepthVista");
		msgBox->setIcon(QMessageBox::Information);
		msgBox->setText("Path does not exist");
		msgBox->setStandardButtons(QMessageBox::Ok);
		msgBox->setWindowModality(Qt::NonModal);
		msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

		connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

		msgBox->show();
		emit saveFailedDueToInvalidPath();
    }
    else {
        if(depth || ir || rgb || rawDepth || PLY)
            startedSavingFrames = true;

        if (data_mode != IR_Mode && data_mode <= Depth_IR_RGB_HD_Mode) {

            if (savePLY = PLY)
                savePLYFrameCount = 1;
            if (saveRawFrame = rawDepth)
                saveRawFrameCount = 1;
            if (saveDepthFrame = depth)
                saveDepthFrameCount = 1;
        }
        if (data_mode >= Depth_IR_RGB_VGA_Mode) {
            if (saveRGBFrame = rgb)
                saveRGBFrameCount = 1;
        }
        if (data_mode != Depth_Mode && data_mode <= Depth_IR_RGB_HD_Mode) {
            if (saveIRFrame = ir)
                saveIRFrameCount = 1;
        }
        saveRGBDir = save_frame_dir;
        saveIRDir = save_frame_dir;
        saveDepthDir = save_frame_dir;
        save3dDir = save_frame_dir;
        saveRAWDir = save_frame_dir;
    }
}
