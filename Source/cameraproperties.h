#ifndef CAMERAPROPERTIES_H
#define CAMERAPROPERTIES_H

#include <QWidget>
#include<QScreen>
#include <QDebug>
#include <QTextStream>
#include <QComboBox>
#include <QLabel>
#include <QMessageBox>
#include <QAbstractItemView>
#include <DepthVistaSDK/DepthVista.h>
#include <utility>
#include <vector>
#include <QAbstractButton>
#include <QApplication>
#include <QClipboard>
#include <QStandardPaths>
#include <QFileDialog>

typedef enum {
    TOF_UVC_EXT_CID_DATA_MODE = 14,
    TOF_UVC_EXT_CID_DEPTH_RANGE,
    TOF_UVC_EXT_CID_CORING,
    TOF_UVC_EXT_CID_IR_GAIN,
    TOF_HID_CID_ANTI_FLICKER_DETECTION,
    TOF_HID_CID_SPECIAL_EFFECT,
    TOF_HID_CID_DENOISE,
    TOF_HID_CID_AUTO_EXP_ROI,
    TOF_HID_CID_AUTO_EXP_WIN_SIZE,
    TOF_HID_CID_ORIENTATION,
    TOF_HID_CID_FACE_DETECTION,
    TOF_HID_CID_FACE_STATUS,
    TOF_HID_CID_FACE_RECT,
    TOF_HID_CID_SMILE_DETECTION,
    TOF_HID_CID_SMILE_STATUS,
	TOF_HID_CID_IMU_DATA,
    TOF_HID_CID_EXPOSURE_COMP,
    TOF_HID_CID_FPS_CTRL,
    TOF_HID_CID_FLASH_MODE,
    TOF_HID_CID_SET_DEFAULT,
    TOF_APP_CID_DEPTH_UNIT,
    TOF_APP_CID_CURSOR_COLOR,
    TOF_APP_CID_DEPTH_REGION,
    TOF_APP_CID_CLIPBOARD_COPY,
    TOF_APP_CID_SPC_DEPTH_RANGE,
    TOF_APP_CID_RGB_OVERLAY,
    TOF_APP_CID_DEPTH_PLANARIZE,
    TOF_APP_CID_DEPTH_X,
    TOF_APP_CID_DEPTH_Y,
    TOF_APP_TEMP_DISPLAY,
    TOF_APP_AVERAGE_IR_DISPLAY,
    TOF_APP_ENABLE_SPATIAL_FILTER,
    TOF_APP_ENABLE_TEMPORAL_FILTER,
    TOF_APP_ENABLE_EDGE_DETECTION,
    TOF_APP_MIN_DEPTH,
    TOF_APP_MAX_DEPTH,
    TOF_APP_TOF_REGISTER,
    TOF_APP_TOF_REGISTER_VALUE,
    TOF_APP_GET_FIRMWARE_VERSION,
    TOF_APP_GET_UNIQUE_ID,
	TOF_UVC_CID_SET_DEFAULT
}CameraProp;

#define TOF_APP_VAL_FAR_SPC_DEPTH_MIN 1000
#define TOF_APP_VAL_FAR_SPC_DEPTH_MAX 6000
#define TOF_APP_VAL_NEAR_SPC_DEPTH_MIN 200
#define TOF_APP_VAL_NEAR_SPC_DEPTH_MAX 1200

#define TOF_APP_VAL_DISABLE_ROI 0
#define TOF_APP_VAL_FULL_ROI 1
#define TOF_APP_VAL_MANUAL_ROI 2

#define TOF_APP_VAL_SPC_EFFECT_NORMAL 1
#define TOF_APP_VAL_SPC_EFFECT_BW 4
#define TOF_APP_VAL_SPC_EFFECT_GRAY 7
#define TOF_APP_VAL_SPC_EFFECT_NEGATIVE 8
#define TOF_APP_VAL_SPC_EFFECT_SKETCH 16

#define TOF_APP_VAL_SCENE_NORMAL 0X01
#define TOF_APP_VAL_SCENE_DOC   0X0C

#define TOF_APP_VAL_FACE_DET_ENABLE 1
#define TOF_APP_VAL_FACE_DET_DISABLE 0

#define TOF_APP_VAL_SMILE_DET_ENABLE 1
#define TOF_APP_VAL_SMILE_DET_DISABLE 0

#define TOF_APP_VAL_IMU_ENABLE 1
#define TOF_APP_VAL_IMU_DISABLE 0

#define TOF_APP_VAL_DEPTH_UNIT_1MM 1
#define TOF_APP_VAL_DEPTH_UNIT_2MM 0

#define TOF_APP_VAL_AVG_DEPTH_CENTER 0
#define TOF_APP_VAL_AVG_DEPTH_MOUSE 1

#define TOF_APP_VAL_WHITE_CURSOR 0
#define TOF_APP_VAL_BLACK_CURSOR 1

#define TOF_APP_VAL_CLR_CLIPBOARD 0
#define TOF_APP_VAL_CPY_CLIPBOARD 1

#define TOF_APP_SPATIAL_MEDIAN_FILTER 0
#define TOF_APP_SPATIAL_GAUSSIAN_FILTER 1
#define TOF_APP_SPATIAL_BILATERAL_FILTER 2

namespace Ui {
class Cameraproperties;
}

class Cameraproperties : public QWidget
{
    Q_OBJECT

public:
    QStringList devicelist;
    explicit Cameraproperties(QWidget *parent = nullptr,int index = -1);
    ~Cameraproperties();
signals:
    Q_SIGNAL void enumerateDevices();
    Q_SIGNAL void inputselected(int index);
    Q_SIGNAL void setFilterType(int ctrlID, bool selected);
    Q_SIGNAL void setCamProperty(int ctrlID,int value);
    Q_SIGNAL void setCamProperty(int ctrlID,int value,int value1,int overlay);
    Q_SIGNAL void getDepthIRdata();
    Q_SIGNAL void savePLYClicked(QString saveDir);
    Q_SIGNAL void applyAvgChangesClicked(int avg_x, int avg_y);
    Q_SIGNAL void applyColorMapChangesClicked(int min, int max, int colorMap);
    Q_SIGNAL void startSavingFrames(bool depth, bool ir, bool rgb, bool rawDepth, bool PYL, QString save_img_dir);
public slots:
    Q_SLOT void onexitpressed();
    Q_SLOT void onDeviceEnumerated(QStringList devices);
    Q_SLOT void deviceChanged(int index);
    Q_SLOT void onCamPropQueried(std::vector< std::pair <int,int> > camera_prop);
    Q_SLOT void onDepthModeChanged(std::vector< std::pair <int,int> > camera_prop);
    Q_SLOT void onUvcPropQueried(std::vector< UVCProp > uvc_prop);
    Q_SLOT void onCamPropChanged(int ctrlID,int value, QLabel *label);
    Q_SLOT void onCamPropChanged(int ctrlID,int value);
    Q_SLOT void onFilterTypeSelected(int ctrlID);
    Q_SLOT void onGrpButtonchanged(int ctrlID,int id, bool checked);
    Q_SLOT void onUpdateData(int avgDepth,int stdDepth,int avgIR,int stdIR);
    Q_SLOT void onClipboardChanged(int id);
    Q_SLOT void onOpenFolder();
    Q_SLOT void savePLYPressed();
    Q_SLOT void applyAveChanges();
    Q_SLOT void changeFontSize();

    Q_SLOT void onFrameSaveClicked();
    Q_SLOT void enableAndDisableSaveButton();
    Q_SLOT void onSavingFramesComplete();
    Q_SLOT void onOthersavedoneplyfail();
    Q_SLOT void onTofCamModeSelected(bool ir_mode);
    Q_SLOT void onDualCamModeSelected(bool vgaMode);
    Q_SLOT void onRgbCamModeSelected();
    Q_SLOT void onDeviceRemoved();
    Q_SLOT void onFirmwareVersionRead(uint8_t gMajorVersion, uint8_t gMinorVersion1, uint16_t gMinorVersion2, uint16_t gMinorVersion3);
    Q_SLOT void onUniqueIDRead(uint64_t uniqueID);
	Q_SLOT void RGBDMapping_checkBox_stateChange(int state);
	Q_SLOT void RGBDMapping_enable(bool state);
    Q_SLOT void ontofSettingsDefault();

private:
    Ui::Cameraproperties *ui;
    QWidget *parent_widget;
    QClipboard *clipboard = QGuiApplication::clipboard();
    int oldindex = 0;
    QString save_img_dir, current_save_img_dir;
    void connectCamPropSlots();
    bool camPropSlotsConnected = false;
    bool threeDRendering = false;
    DataMode cDataMode;
    int dataModeCurrentIndex, depthRangeCurrentIndex;

protected:
    bool eventFilter(QObject *obj, QEvent *event);

private slots:
    void on_DatamodeApplyButton_clicked();
    void on_RangeApplyButton_clicked();
};

#endif // CAMERAPROPERTIES_H
