#include "cameraproperties.h"
#include "ui_cameraproperties.h"
Cameraproperties::Cameraproperties(QWidget *parent, int index) :
    ui(new Ui::Cameraproperties),
    parent_widget(parent),
    oldindex(index)
{
    ui->setupUi(this);
    ui->avg_x_pixel_spinBox->setMinimum(4);
    ui->avg_x_pixel_spinBox->setMaximum(200);
    ui->avg_y_pixel_spinBox->setMinimum(4);
    ui->avg_y_pixel_spinBox->setMaximum(200);
    ui->avg_x_pixel_spinBox->setValue(32);
    ui->avg_y_pixel_spinBox->setValue(32);
    ui->device_list_cmb->installEventFilter(this);
	ui->avg_depth_value->setText("0 mm");
	ui->std_depth_dev_value->setText("0 mm");

    QScreen* screen = QGuiApplication::primaryScreen();
    QRect mm = screen->availableGeometry();
    int screen_width = mm.width();
    int screen_height = mm.height();

    QFont tabWidgetFont;
    QFont device_list_cmb_Font;
    QFont input_device_label_Font;
    QFont depth_range_label_Font;

    tabWidgetFont = ui->tabWidget->font();
    device_list_cmb_Font = ui->device_list_cmb->font();
    input_device_label_Font = ui->input_device_label->font();
    depth_range_label_Font = ui->depth_range_label->font();

	msgBox = new QMessageBox(this);

    if (screen_width < 1650)
    {
        tabWidgetFont.setPointSize(9);
        device_list_cmb_Font.setPointSize(9);
        input_device_label_Font.setPointSize(9);
        depth_range_label_Font.setPointSize(9);
    }
    else if (screen_width >= 1650 && screen_width <= 2880)
    {
        tabWidgetFont.setPointSize(10);
        device_list_cmb_Font.setPointSize(10);
        input_device_label_Font.setPointSize(10);
        depth_range_label_Font.setPointSize(10);
    }
    else if (screen_width > 2880)
    {
        tabWidgetFont.setPointSize(11);
        device_list_cmb_Font.setPointSize(11);
        input_device_label_Font.setPointSize(11);
        depth_range_label_Font.setPointSize(11);
    }

    ui->FrameSavePushButton->setEnabled(false);
    ui->data_mode_label->setFont(tabWidgetFont);
    ui->depth_range_label->setFont(depth_range_label_Font);
    ui->device_list_cmb->setFont(device_list_cmb_Font);
    ui->input_device_label->setFont(input_device_label_Font);
    connect(parent_widget,SIGNAL(windowResized()),this,SLOT(changeFontSize()));
    connect(parent_widget,SIGNAL(framesStopped(int)),this,SLOT(onFramesStopped(int)));
    QSize scrollArea_2_size;
    scrollArea_2_size = ui->scrollArea_2->size();

    ui->gridLayout->setColumnMinimumWidth(0, scrollArea_2_size.width() * 0.3);
    ui->gridLayout->setColumnMinimumWidth(1, scrollArea_2_size.width() * 0.67);

    connect(this,SIGNAL(enumerateDevices()),parent_widget,SLOT(enumerateDevices()));
    connect(parent_widget,SIGNAL(deviceEnumerated(QStringList, int )),this, SLOT(onDeviceEnumerated(QStringList, int )));
    if(oldindex>0){
        ui->tabWidget->setEnabled(true);
    }else{
        ui->tabWidget->setEnabled(false);
    }
    connect(this,SIGNAL(inputselected(int)),parent_widget,SLOT(onInputSelected(int)));
    connect(parent_widget,SIGNAL(campropQueried(std::vector< std::pair <int,int> >)),this,SLOT(onCamPropQueried(std::vector< std::pair <int,int> >)));
    connect(parent_widget,SIGNAL(depthModeChanged(std::vector< std::pair <int,int> >)),this,SLOT(onDepthModeChanged(std::vector< std::pair <int,int> >)));
    connect(parent_widget,SIGNAL(uvcpropQueried(std::vector< UVCProp >)),this,SLOT(onUvcPropQueried(std::vector< UVCProp>)));
    connect(parent_widget,SIGNAL(updateData(int,int,int,int)),this,SLOT(onUpdateData(int,int,int,int)));
    connect(this,SIGNAL(getDepthIRdata()),parent_widget,SLOT(ongetDepthIRdata()));
    if(QSysInfo::productType() == "windows")
    {
        if(!QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista").exists()) {
            QDir().mkdir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista");
        }
        if(!QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista/Pictures").exists()) {
            QDir().mkdir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista/Pictures");
        }
        save_img_dir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista/Pictures";
    }
    else
    {
        if(!QDir("Pictures").exists()) {
            QDir().mkdir("Pictures");
        }
        if(QDir("Pictures").exists()) {
            QDir dir(QCoreApplication::applicationDirPath());
            save_img_dir = dir.absoluteFilePath("Pictures");
        }
    }
    ui->save_image_location->setText(save_img_dir);
    connect(ui->open_folder_btn,SIGNAL(clicked()),this,SLOT(onOpenFolder()));

    connect(ui->apply_avg_changes,SIGNAL(clicked()),this,SLOT(applyAveChanges()));
    connect(this,SIGNAL(applyAvgChangesClicked(int, int)),parent_widget,SLOT(onApplyAvgChangesClicked(int ,int)));
    connect(this,SIGNAL(savePLYClicked(QString)),parent_widget,SLOT(onSavePLYClicked(QString)));

    connect(ui->FrameSavePushButton, SIGNAL(clicked()), this, SLOT(onFrameSaveClicked()));
    connect(this, SIGNAL(startSavingFrames(bool, bool, bool, bool, bool, QString)), parent_widget, SLOT(onStartSavingFrames(bool, bool, bool, bool, bool, QString)));
    connect(parent_widget, SIGNAL(savingFramesComplete()), this, SLOT(onSavingFramesComplete()));
    connect(parent_widget, SIGNAL(othersavedoneplyfail()), this, SLOT(onOthersavedoneplyfail()));
    connect(parent_widget, SIGNAL(tofCamModeSelected(bool)), this, SLOT(onTofCamModeSelected(bool)));
    connect(parent_widget, SIGNAL(dualCamModeSelected(bool)), this, SLOT(onDualCamModeSelected(bool)));
    connect(parent_widget, SIGNAL(rgbCamModeSelected(uint8_t, uint32_t, uint8_t , uint8_t )), this, SLOT(onRgbCamModeSelected(uint8_t, uint32_t, uint8_t , uint8_t )));
    connect(parent_widget, SIGNAL(deviceRemoved()), this, SLOT(onDeviceRemoved()));
    connect(ui->threeDgroupBox, SIGNAL(toggled(bool)), parent_widget, SLOT(onPCLDisplay_checkBox_stateChange(bool)));
    connect(ui->rgbDCheckBox, SIGNAL(stateChanged(int)), parent_widget, SLOT(onRGBDMapping_checkBox_stateChange(int)));
    connect(ui->rgbDCheckBox, SIGNAL(stateChanged(int)), this, SLOT(RGBDMapping_checkBox_stateChange(int)));
    connect(ui->undistortChkBx, SIGNAL(stateChanged(int)), parent_widget, SLOT(onUndistort_checkBox_stateChange(int)));
	connect(ui->flying_pixel_chkb, SIGNAL(stateChanged(int)), parent_widget, SLOT(onFlyingPixelToggled(int)));
	connect(parent_widget, SIGNAL(firmwareVersionRead(uint8_t, uint8_t, uint16_t, uint16_t)), this, SLOT(onFirmwareVersionRead(uint8_t, uint8_t, uint16_t, uint16_t)));
    connect(parent_widget, SIGNAL(uniqueIDRead(uint64_t)), this, SLOT(onUniqueIDRead(uint64_t)));
    connect(parent_widget, SIGNAL(tofSettingsDefault()), this, SLOT(ontofSettingsDefault()));
    connect(parent_widget, SIGNAL(saveFailedDueToInvalidPath()), this, SLOT(enableAndDisableSaveButton()));
    connect(ui->DepthRawSaveCheckBox, SIGNAL(stateChanged(int)), this, SLOT(enableAndDisableSaveButton()));
    connect(ui->DepthSaveCheckBox, SIGNAL(stateChanged(int)), this, SLOT(enableAndDisableSaveButton()));
    connect(ui->IRSaveCheckBox, SIGNAL(stateChanged(int)), this, SLOT(enableAndDisableSaveButton()));
    connect(ui->PLYSaveCheckBox, SIGNAL(stateChanged(int)), this, SLOT(enableAndDisableSaveButton()));
    connect(ui->RGBSaveCheckBox, SIGNAL(stateChanged(int)), this, SLOT(enableAndDisableSaveButton()));
    connect(ui->imuValGroupBox, SIGNAL(toggled(bool)), parent_widget, SLOT(onIMUValshowToggled(bool)));
    connect(parent_widget, SIGNAL(imuValues(int, int, int)), this, SLOT(updateIMUValues(int, int, int)));
}

void Cameraproperties::ontofSettingsDefault()
{
    ui->avg_depth_value->setText("0 mm");
    ui->std_depth_dev_value->setText("0 mm");
    ui->avg_irvalue->setText("0");
    ui->std_ir_dev_value->setText("0");
    ui->tof_coring_spinBox->setValue(150);
    ui->tof_gain_slider->setValue(16);
    ui->data_mode_cmb->setCurrentIndex(3);
    ui->depth_range_cmb->setCurrentIndex(1);
    ui->rgbDCheckBox->setChecked(false);
    ui->depth_planarize_chkb->setChecked(false);
    ui->edge_chkBox->setChecked(false);
    ui->spatial_chkBox->setChecked(false);
    ui->temporal_chkBox->setChecked(false);
    ui->undistortChkBx->setChecked(false);
	ui->flying_pixel_chkb->setChecked(false);
    ui->avg_depth_ir_grpbox->setChecked(false);
    ui->IR_display_checkBox->setChecked(false);

    disconnect(this, SIGNAL(setFilterType(int, bool)), parent_widget, SLOT(onSetFilterType(int, bool)));
    disconnect(this, SIGNAL(setCamProperty(int, int)), parent_widget, SLOT(onSetCamProperty(int, int)));
    disconnect(this, SIGNAL(setCamProperty(int, int, int, int)), parent_widget, SLOT(onSetCamProperty(int, int, int, int)));

    ui->center_btn->setChecked(true);
    ui->mouse_ptr_btn->setChecked(false);

    connect(this,SIGNAL(setFilterType(int, bool)),parent_widget,SLOT(onSetFilterType(int, bool)));
    connect(this,SIGNAL(setCamProperty(int,int)),parent_widget,SLOT(onSetCamProperty(int,int)));
    connect(this,SIGNAL(setCamProperty(int,int,int,int)),parent_widget,SLOT(onSetCamProperty(int,int,int,int)));

    ui->cursor_black_btn->setChecked(false);
    ui->cursor_white_btn->setChecked(true);
    ui->DepthRawSaveCheckBox->setChecked(false);
    ui->DepthSaveCheckBox->setChecked(false);
    ui->IRSaveCheckBox->setChecked(false);
    ui->PLYSaveCheckBox->setChecked(false);
    ui->RGBSaveCheckBox->setChecked(false);
}

void Cameraproperties::updateIMUValues(int x, int y, int z)
{
    ui->x_value->setNum(x);
    ui->y_value->setNum(y);
    ui->z_value->setNum(z);
}

void Cameraproperties::onDeviceEnumerated(QStringList devices, int streamingDevIndexInList)
{
    devicelist = devices;
    disconnect(ui->device_list_cmb,SIGNAL(currentIndexChanged(int)), this, SLOT(deviceChanged(int)));
    ui->device_list_cmb->clear();
    ui->device_list_cmb->addItems(devices);
    ui->device_list_cmb->setCurrentIndex(streamingDevIndexInList);
	oldindex = streamingDevIndexInList;
    connect(ui->device_list_cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(deviceChanged(int)));
}

void Cameraproperties::changeFontSize()
{
    QScreen* screen = QGuiApplication::primaryScreen();
    QRect mm = screen->availableGeometry();
    int screen_width = mm.width();
    int screen_height = mm.height();
    QFont tabWidgetFont;
    QFont depth_range_label_Font;
    QFont device_list_cmb_Font;
    QFont input_device_label_Font;
    QFont scrollArea_Font;

    tabWidgetFont = ui->tabWidget->font();
    device_list_cmb_Font = ui->device_list_cmb->font();
    input_device_label_Font = ui->input_device_label->font();
    depth_range_label_Font = ui->depth_range_label->font();
    scrollArea_Font = ui->scrollArea->font();


    if (screen_width < 1650 )
    {
        tabWidgetFont.setPointSize(9);
        device_list_cmb_Font.setPointSize(9);
        input_device_label_Font.setPointSize(9);
        depth_range_label_Font.setPointSize(9);
        scrollArea_Font.setPointSize(9);
    }
    else if (screen_width >= 1650 && screen_width <= 2880)
    {
        tabWidgetFont.setPointSize(10);
        device_list_cmb_Font.setPointSize(10);
        input_device_label_Font.setPointSize(10);
        depth_range_label_Font.setPointSize(10);
        scrollArea_Font.setPointSize(10);

    }
    else if (screen_width > 2880)
    {
        tabWidgetFont.setPointSize(11);
        device_list_cmb_Font.setPointSize(11);
        input_device_label_Font.setPointSize(11);
        depth_range_label_Font.setPointSize(11);
        scrollArea_Font.setPointSize(11);
    }

    ui->tabWidget->setFont(tabWidgetFont);
    ui->data_mode_label->setFont(tabWidgetFont);
    ui->depth_range_label->setFont(depth_range_label_Font);
    ui->device_list_cmb->setFont(device_list_cmb_Font);
    ui->input_device_label->setFont(input_device_label_Font);
    ui->scrollArea->setFont(scrollArea_Font);
    QSize scrollArea_2_size;
    scrollArea_2_size = ui->scrollArea_2->size();

}
void Cameraproperties::onUvcPropQueried(std::vector< UVCProp > uvc_prop)
{
    disconnect(this, SIGNAL(setFilterType(int, bool)), parent_widget, SLOT(onSetFilterType(int, bool)));
    disconnect(this, SIGNAL(setCamProperty(int, int)), parent_widget, SLOT(onSetCamProperty(int, int)));
    disconnect(this, SIGNAL(setCamProperty(int, int, int, int)), parent_widget, SLOT(onSetCamProperty(int, int, int, int)));
    if(!camPropSlotsConnected)
        connectCamPropSlots();
    for (size_t i = 0; i < uvc_prop.size(); i++) {
        switch (uvc_prop[i].id) {
            case TOF_UVC_CID_BRIGHTNESS:
                ui->brightness_slider->setMinimum(uvc_prop[i].min);
                ui->brightness_slider->setMaximum(uvc_prop[i].max);
                ui->brightness_slider->setSingleStep(uvc_prop[i].step);
                ui->brightness_slider->setPageStep(uvc_prop[i].step);
                ui->brightness_slider->setValue(uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_CONTRAST:
                ui->contrast_slider->setMinimum(uvc_prop[i].min);
                ui->contrast_slider->setMaximum(uvc_prop[i].max);
                ui->contrast_slider->setSingleStep(uvc_prop[i].step);
                ui->contrast_slider->setPageStep(uvc_prop[i].step);
                ui->contrast_slider->setValue(uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_SATURATION:
                ui->saturation_slider->setMinimum(uvc_prop[i].min);
                ui->saturation_slider->setMaximum(uvc_prop[i].max);
                ui->saturation_slider->setSingleStep(uvc_prop[i].step);
                ui->saturation_slider->setPageStep(uvc_prop[i].step);
                ui->saturation_slider->setValue(uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_WB_AUTO:
                ui->wb_auto_chkb->setChecked((bool)uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_GAMMA:
                ui->gamma_slider->setMinimum(uvc_prop[i].min);
                ui->gamma_slider->setMaximum(uvc_prop[i].max);
                ui->gamma_slider->setSingleStep(uvc_prop[i].step);
                ui->gamma_slider->setPageStep(uvc_prop[i].step);
                ui->gamma_slider->setValue(uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_GAIN:
                ui->gain_slider->setMinimum(uvc_prop[i].min);
                ui->gain_slider->setMaximum(uvc_prop[i].max);
                ui->gain_slider->setSingleStep(uvc_prop[i].step);
                ui->gain_slider->setPageStep(uvc_prop[i].step);
                ui->gain_slider->setValue(uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_PWR_LINE_FREQ:
                ui->pwr_line_freq_cmb->setCurrentIndex(uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_WB_TEMP:
                ui->wb_slider->setEnabled(!ui->wb_auto_chkb->isChecked());
                ui->wb_slider->setMinimum(uvc_prop[i].min/50);
                ui->wb_slider->setMaximum(uvc_prop[i].max/50);
                ui->wb_slider->setSingleStep(uvc_prop[i].step/50);
                ui->wb_slider->setPageStep(uvc_prop[i].step/50);
                ui->wb_slider->setValue(uvc_prop[i].cur/50);
                break;
            case TOF_UVC_CID_SHARPNESS:
                ui->sharpness_slider->setMinimum(uvc_prop[i].min);
                ui->sharpness_slider->setMaximum(uvc_prop[i].max);
                ui->sharpness_slider->setSingleStep(uvc_prop[i].step);
                ui->sharpness_slider->setPageStep(uvc_prop[i].step);
                ui->sharpness_slider->setValue(uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_EXPOSURE_AUTO:
                ui->expo_auto_btn->setChecked(!(bool)uvc_prop[i].cur);
                break;
            case TOF_UVC_CID_EXPSOURE_ABS:
                ui->exposure_slider->setEnabled(!ui->expo_auto_btn->isChecked());
                ui->exposure_slider->setMinimum(uvc_prop[i].min);
                ui->exposure_slider->setMaximum(uvc_prop[i].max);
                ui->exposure_slider->setSingleStep(uvc_prop[i].step);
                ui->exposure_slider->setPageStep(uvc_prop[i].step);
                ui->exposure_slider->setValue(uvc_prop[i].cur);
                break;
        }
    }
    connect(this,SIGNAL(setFilterType(int, bool)),parent_widget,SLOT(onSetFilterType(int, bool)));
    connect(this,SIGNAL(setCamProperty(int,int)),parent_widget,SLOT(onSetCamProperty(int,int)));
    connect(this,SIGNAL(setCamProperty(int,int,int,int)),parent_widget,SLOT(onSetCamProperty(int,int,int,int)));
}
void Cameraproperties::onDepthModeChanged(std::vector< std::pair <int, int> > camera_prop)
{
    int value = 0, ctrl_id = 0;
    for (size_t i = 0; i < camera_prop.size(); i++) {
        ctrl_id = camera_prop[i].first;
        value = camera_prop[i].second;
        switch (ctrl_id) {

        case TOF_UVC_EXT_CID_DEPTH_RANGE:
            if (value) {
                ui->spc_depth_min_value->setMinimum(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
                ui->spc_depth_min_value->setMaximum(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
                ui->spc_depth_max_value->setMinimum(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
                ui->spc_depth_max_value->setMaximum(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
                ui->spc_depth_min_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
                ui->spc_depth_max_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
            }
            else {
                ui->spc_depth_min_value->setMinimum(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
                ui->spc_depth_min_value->setMaximum(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
                ui->spc_depth_max_value->setMinimum(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
                ui->spc_depth_max_value->setMaximum(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
                ui->spc_depth_min_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
                ui->spc_depth_max_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
            }
            break;
        case TOF_UVC_EXT_CID_CORING:

            ui->tof_coring_spinBox->setValue(value);
            break;
        case TOF_UVC_EXT_CID_IR_GAIN:
            ui->tof_gain_slider->setValue(value);
            ui->cur_tof_gain_value->setNum(value);
            break;
        case TOF_APP_MIN_DEPTH:
            ui->min_depth_val->setText(QStringLiteral("%1 mm").arg(value));
            break;
        case TOF_APP_MAX_DEPTH:
            ui->max_depth_val->setText(QStringLiteral("%1 mm").arg(value));
            break;
        }
    }
    ui->avg_depth_ir_btn_grp->setId(ui->center_btn, TOF_APP_VAL_AVG_DEPTH_CENTER);
    ui->avg_depth_ir_btn_grp->setId(ui->mouse_ptr_btn, TOF_APP_VAL_AVG_DEPTH_MOUSE);
    ui->cursor_color_btn_grp->setId(ui->cursor_white_btn, TOF_APP_VAL_WHITE_CURSOR);
    ui->cursor_color_btn_grp->setId(ui->cursor_black_btn, TOF_APP_VAL_BLACK_CURSOR);
    ui->clipboard_btn_grp->setId(ui->clear_clipboard_btn, TOF_APP_VAL_CLR_CLIPBOARD);
    ui->clipboard_btn_grp->setId(ui->copy_clipboard_btn, TOF_APP_VAL_CPY_CLIPBOARD);
}

void Cameraproperties::onCamPropQueried(std::vector< std::pair <int,int> > camera_prop)
{
    disconnect(this, SIGNAL(setFilterType(int, bool)), parent_widget, SLOT(onSetFilterType(int, bool)));
    disconnect(this, SIGNAL(setCamProperty(int, int)), parent_widget, SLOT(onSetCamProperty(int, int)));
    disconnect(this, SIGNAL(setCamProperty(int, int, int, int)), parent_widget, SLOT(onSetCamProperty(int, int, int, int)));
    int value = 0,ctrl_id = 0;
    for(size_t i = 0;i<camera_prop.size();i++){
        ctrl_id = camera_prop[i].first;
        value   = camera_prop[i].second;
        switch (ctrl_id) {

            case TOF_UVC_EXT_CID_DATA_MODE:
                cDataMode = (DataMode)value;
                value = (value == 0) ? value : value - 1;
                ui->data_mode_cmb->setCurrentIndex(value);
                dataModeCurrentIndex = ui->data_mode_cmb->currentIndex();
                if(cDataMode == DataMode::Depth_IR_Mode)
                {
                    ui->postProcessingGrpBox->setEnabled(true);
                    ui->tof_coring_label->setEnabled(true);
                    ui->tof_coring_spinBox->setEnabled(true);
                    ui->avg_depth_ir_grpbox->setEnabled(true);
                    ui->spc_depth_grp_box->setEnabled(true);
                    ui->spc_depth_grp_box->setChecked(true);
                    ui->tof_gain_label->setEnabled(true);
                    ui->tof_gain_slider->setEnabled(true);
                    ui->cur_tof_gain_value->setEnabled(true);
                    ui->depth_ir_data_grpbox->setEnabled(true);
                    ui->DepthRawSaveCheckBox->setEnabled(true);
                    ui->PLYSaveCheckBox->setEnabled(true);
                    ui->DepthSaveCheckBox->setEnabled(true);
                    ui->RGBSaveCheckBox->setEnabled(false);
                    ui->IRSaveCheckBox->setEnabled(true);
                }else if(cDataMode == DataMode::Depth_Mode)
                {
                    ui->postProcessingGrpBox->setEnabled(true);
                    ui->tof_coring_label->setEnabled(true);
                    ui->tof_coring_spinBox->setEnabled(true);
                    ui->avg_depth_ir_grpbox->setEnabled(true);
                    ui->spc_depth_grp_box->setEnabled(true);
                    ui->spc_depth_grp_box->setChecked(true);
                    ui->tof_gain_label->setEnabled(false);
                    ui->tof_gain_slider->setEnabled(false);
                    ui->cur_tof_gain_value->setEnabled(false);
                    ui->depth_ir_data_grpbox->setEnabled(true);
                    ui->DepthRawSaveCheckBox->setEnabled(true);
                    ui->PLYSaveCheckBox->setEnabled(true);
                    ui->DepthSaveCheckBox->setEnabled(true);
                    ui->RGBSaveCheckBox->setEnabled(false);
                    ui->IRSaveCheckBox->setEnabled(false);
                }else if (cDataMode == DataMode::IR_Mode) {
                    ui->postProcessingGrpBox->setEnabled(false);
                    ui->postProcessingGrpBox->setChecked(false);
                    ui->tof_coring_label->setEnabled(false);
                    ui->tof_coring_spinBox->setEnabled(false);
                    ui->avg_depth_ir_grpbox->setEnabled(false);
                    ui->spc_depth_grp_box->setEnabled(false);
                    ui->spc_depth_grp_box->setChecked(false);
                    ui->tof_gain_label->setEnabled(true);
                    ui->tof_gain_slider->setEnabled(true);
                    ui->cur_tof_gain_value->setEnabled(true);
                    ui->depth_ir_data_grpbox->setEnabled(false);
                    ui->IRSaveCheckBox->setEnabled(true);
                    ui->DepthRawSaveCheckBox->setEnabled(false);
                    ui->PLYSaveCheckBox->setEnabled(false);
                    ui->DepthSaveCheckBox->setEnabled(false);
                    ui->RGBSaveCheckBox->setEnabled(false);
                }
                else if(cDataMode == DataMode::Depth_IR_RGB_VGA_Mode || cDataMode == DataMode::Depth_IR_RGB_HD_Mode)
                {
                    ui->DepthRawSaveCheckBox->setEnabled(true);
                    ui->PLYSaveCheckBox->setEnabled(true);
                    ui->DepthSaveCheckBox->setEnabled(true);
                    ui->RGBSaveCheckBox->setEnabled(true);
                    ui->IRSaveCheckBox->setEnabled(true);
                }
                else if(cDataMode >= DataMode::RGB_VGA_Mode)
                {
                    ui->DepthRawSaveCheckBox->setEnabled(false);
                    ui->IRSaveCheckBox->setEnabled(false);
                    ui->PLYSaveCheckBox->setEnabled(false);
                    ui->DepthSaveCheckBox->setEnabled(false);
                    ui->RGBSaveCheckBox->setEnabled(true);
                }

                if(cDataMode == DataMode::RGB_Full_HD_Mode || cDataMode == DataMode::RGB_1200p_Mode)
                {
                    ui->cur_fps_ctrl_value->setMaximum(30);
                }
                if(cDataMode == DataMode::RGB_HD_Mode || cDataMode == DataMode::RGB_VGA_Mode)
                {
                    ui->cur_fps_ctrl_value->setMaximum(60);
                }
                break;

            case TOF_UVC_EXT_CID_DEPTH_RANGE:
                ui->depth_range_cmb->setCurrentIndex(value);
                if (value) {
                    ui->spc_depth_min_value->setMinimum(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
                    ui->spc_depth_min_value->setMaximum(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
                    ui->spc_depth_max_value->setMinimum(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
                    ui->spc_depth_max_value->setMaximum(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
                    ui->spc_depth_min_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
                    ui->spc_depth_max_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
                }
                else {
                    ui->spc_depth_min_value->setMinimum(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
                    ui->spc_depth_min_value->setMaximum(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
                    ui->spc_depth_max_value->setMinimum(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
                    ui->spc_depth_max_value->setMaximum(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
                    ui->spc_depth_min_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
                    ui->spc_depth_max_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
                }
                depthRangeCurrentIndex = ui->depth_range_cmb->currentIndex();
                break;
            case TOF_UVC_EXT_CID_CORING:
            ui->tof_coring_spinBox->setValue(value);
            break;
            case TOF_UVC_EXT_CID_IR_GAIN:
            ui->tof_gain_slider->setValue(value);
            ui->cur_tof_gain_value->setNum(value);
            break;
            case TOF_HID_CID_ANTI_FLICKER_DETECTION:
            ui->anti_flicker_cmb->setCurrentIndex(value);
            break;
            case TOF_HID_CID_SPECIAL_EFFECT:
            ui->special_effect_cmb->setCurrentIndex(value);
            break;
            case TOF_HID_CID_DENOISE:
            ui->denoise_slider->setValue(value);
            break;
            case TOF_HID_CID_AUTO_EXP_ROI:
            if(!value){
                ui->roi_disable_btn->setChecked(true);
				ui->win_size_label->setEnabled(false);

            }else if(value == TOF_APP_VAL_FULL_ROI){
                ui->full_roi_btn->setChecked(true);
				ui->win_size_label->setEnabled(false);
            }else if(value == TOF_APP_VAL_MANUAL_ROI){
				ui->win_size_label->setEnabled(true);
                ui->manual_roi_btn->setChecked(true);
            }
            ui->auto_roi_btn_grp->setId(ui->roi_disable_btn,TOF_APP_VAL_DISABLE_ROI);
            ui->auto_roi_btn_grp->setId(ui->full_roi_btn,TOF_APP_VAL_FULL_ROI);
            ui->auto_roi_btn_grp->setId(ui->manual_roi_btn,TOF_APP_VAL_MANUAL_ROI);
            break;
            case TOF_HID_CID_AUTO_EXP_WIN_SIZE:
            if(ui->manual_roi_btn->isChecked()){
                ui->win_size_cmb->setCurrentIndex(value-1);
            }else{
				ui->win_size_cmb->setEnabled(ui->manual_roi_btn->isChecked());
            }
            break;
            case TOF_HID_CID_ORIENTATION:
            ui->orientation_cmb->setCurrentIndex(value);
            break;
            case TOF_HID_CID_FACE_DETECTION:
            if(value){
                ui->face_enable_btn->setChecked(true);
                ui->face_status_chkb->setEnabled(true);
                ui->face_overlay_chkb->setEnabled(true);
            }else{
                ui->face_disable_btn->setChecked(true);
                ui->face_status_chkb->setEnabled(false);
                ui->face_overlay_chkb->setEnabled(false);
            }
            ui->face_detect_btn_grp->setId(ui->face_disable_btn,TOF_APP_VAL_FACE_DET_DISABLE);
            ui->face_detect_btn_grp->setId(ui->face_enable_btn,TOF_APP_VAL_FACE_DET_ENABLE);
            break;
            case TOF_HID_CID_FACE_STATUS:
            ui->face_status_chkb->setChecked((bool)value);
            break;
            case TOF_HID_CID_FACE_RECT:
            ui->face_overlay_chkb->setChecked((bool)value);
            break;
            case TOF_HID_CID_SMILE_DETECTION:
            if(value){
                ui->smile_enable_btn->setChecked(true);
                ui->smile_status_chkb->setEnabled(true);
            }else{
                ui->smile_disable_btn->setChecked(true);
                ui->smile_status_chkb->setEnabled(false);
            }
            ui->smile_detect_btn_grp->setId(ui->smile_disable_btn,TOF_APP_VAL_SMILE_DET_DISABLE);
            ui->smile_detect_btn_grp->setId(ui->smile_enable_btn,TOF_APP_VAL_SMILE_DET_ENABLE);
            break;
            case TOF_HID_CID_SMILE_STATUS:
            ui->smile_status_chkb->setChecked((bool)value);
            break;
			case TOF_HID_CID_IMU_DATA:
				if (value) {
					ui->IMUEnableRadioBtn->setChecked(true);
				}
				else {
					ui->IMUDisableRadioBtn->setChecked(true);
				}
				ui->IMU_btn_grp->setId(ui->IMUEnableRadioBtn, TOF_APP_VAL_IMU_ENABLE);
				ui->IMU_btn_grp->setId(ui->IMUDisableRadioBtn, TOF_APP_VAL_IMU_DISABLE);

				break;
            case TOF_HID_CID_EXPOSURE_COMP:
            ui->cur_expo_comp_value->setValue(value);
            break;
            case TOF_HID_CID_FPS_CTRL:
            ui->cur_fps_ctrl_value->setValue(value);
            break;
            case TOF_APP_MIN_DEPTH:
                ui->min_depth_val->setText(QStringLiteral("%1 mm").arg(value));
                break;
            case TOF_APP_MAX_DEPTH:
                ui->max_depth_val->setText(QStringLiteral("%1 mm").arg(value));
                break;
        }
    }
    ui->avg_depth_ir_btn_grp->setId(ui->center_btn,TOF_APP_VAL_AVG_DEPTH_CENTER);
    ui->avg_depth_ir_btn_grp->setId(ui->mouse_ptr_btn,TOF_APP_VAL_AVG_DEPTH_MOUSE);
    ui->cursor_color_btn_grp->setId(ui->cursor_white_btn,TOF_APP_VAL_WHITE_CURSOR);
    ui->cursor_color_btn_grp->setId(ui->cursor_black_btn,TOF_APP_VAL_BLACK_CURSOR);
    ui->clipboard_btn_grp->setId(ui->clear_clipboard_btn,TOF_APP_VAL_CLR_CLIPBOARD);
    ui->clipboard_btn_grp->setId(ui->copy_clipboard_btn,TOF_APP_VAL_CPY_CLIPBOARD);
    connect(this,SIGNAL(setFilterType(int, bool)),parent_widget,SLOT(onSetFilterType(int, bool)));
    connect(this,SIGNAL(setCamProperty(int,int)),parent_widget,SLOT(onSetCamProperty(int,int)));
    connect(this,SIGNAL(setCamProperty(int,int,int,int)),parent_widget,SLOT(onSetCamProperty(int,int,int,int)));
}

void Cameraproperties::connectCamPropSlots()
{
    connect(ui->set_default_btn,&QPushButton::clicked,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this, TOF_HID_CID_SET_DEFAULT,std::placeholders::_1));
    connect(ui->set_uvc_default_btn,&QPushButton::clicked,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this, TOF_UVC_CID_SET_DEFAULT,std::placeholders::_1));
    connect(ui->fimware_version_btn,&QPushButton::clicked,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this, TOF_APP_GET_FIRMWARE_VERSION,std::placeholders::_1));
    connect(ui->uniqueID_btn,&QPushButton::clicked,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this, TOF_APP_GET_UNIQUE_ID,std::placeholders::_1));
    connect(ui->brightness_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_BRIGHTNESS,std::placeholders::_1,ui->cur_brightness_value));
    connect(ui->contrast_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_CONTRAST,std::placeholders::_1,ui->cur_contrst_value));
    connect(ui->saturation_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_SATURATION,std::placeholders::_1,ui->cur_saturation_value));
    connect(ui->gamma_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_GAMMA,std::placeholders::_1,ui->cur_gamma_value));
    connect(ui->gain_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_GAIN,std::placeholders::_1,ui->cur_gain_value));
    connect(ui->sharpness_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_SHARPNESS,std::placeholders::_1,ui->cur_sharpness_value));
    connect(ui->wb_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_WB_TEMP,std::placeholders::_1,ui->cur_wb_value));
    connect(ui->exposure_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_EXPSOURE_ABS,std::placeholders::_1,ui->cur_expo_value));
    connect(ui->denoise_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_DENOISE,std::placeholders::_1,ui->cur_denoise_value));
    connect(ui->tof_gain_slider,&QAbstractSlider::valueChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int,QLabel*)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_EXT_CID_IR_GAIN,std::placeholders::_1,ui->cur_tof_gain_value));

    connect(ui->pwr_line_freq_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_PWR_LINE_FREQ,std::placeholders::_1));
    connect(ui->anti_flicker_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_ANTI_FLICKER_DETECTION,std::placeholders::_1));
    connect(ui->special_effect_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_SPECIAL_EFFECT,std::placeholders::_1));
    connect(ui->orientation_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_ORIENTATION,std::placeholders::_1));
    connect(ui->depth_planarize_chkb,&QCheckBox::stateChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_APP_CID_DEPTH_PLANARIZE,std::placeholders::_1));

    connect(ui->wb_auto_chkb,&QCheckBox::stateChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_WB_AUTO,std::placeholders::_1));
    connect(ui->expo_auto_btn,&QCheckBox::stateChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_UVC_CID_EXPOSURE_AUTO,std::placeholders::_1));
    connect(ui->IR_display_checkBox,&QCheckBox::stateChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this, TOF_APP_AVERAGE_IR_DISPLAY,std::placeholders::_1));

    connect(ui->tof_coring_spinBox,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this, TOF_UVC_EXT_CID_CORING,std::placeholders::_1));
    connect(ui->cur_expo_comp_value,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_EXPOSURE_COMP,std::placeholders::_1));
    connect(ui->cur_fps_ctrl_value,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_FPS_CTRL,std::placeholders::_1));


    connect(ui->temporal_chkBox, &QCheckBox::stateChanged, std::bind(static_cast<void(Cameraproperties::*)(int)>(&Cameraproperties::onFilterTypeSelected), this, TOF_APP_ENABLE_TEMPORAL_FILTER));
    connect(ui->spatial_chkBox, &QCheckBox::stateChanged, std::bind(static_cast<void(Cameraproperties::*)(int)>(&Cameraproperties::onFilterTypeSelected), this, TOF_APP_ENABLE_SPATIAL_FILTER));
    connect(ui->edge_chkBox, &QCheckBox::stateChanged, std::bind(static_cast<void(Cameraproperties::*)(int)>(&Cameraproperties::onFilterTypeSelected), this, TOF_APP_ENABLE_EDGE_DETECTION));

    connect(ui->auto_roi_btn_grp,static_cast<void (QButtonGroup::*)(int, bool)>(&QButtonGroup::buttonToggled),std::bind(&Cameraproperties::onGrpButtonchanged,this,TOF_HID_CID_AUTO_EXP_ROI,std::placeholders::_1,std::placeholders::_2));
    connect(ui->face_detect_btn_grp,static_cast<void (QButtonGroup::*)(int, bool)>(&QButtonGroup::buttonToggled),std::bind(&Cameraproperties::onGrpButtonchanged,this,TOF_HID_CID_FACE_DETECTION,std::placeholders::_1,std::placeholders::_2));
    connect(ui->smile_detect_btn_grp,static_cast<void (QButtonGroup::*)(int, bool)>(&QButtonGroup::buttonToggled),std::bind(&Cameraproperties::onGrpButtonchanged,this,TOF_HID_CID_SMILE_DETECTION,std::placeholders::_1,std::placeholders::_2));
    connect(ui->IMU_btn_grp,static_cast<void (QButtonGroup::*)(int, bool)>(&QButtonGroup::buttonToggled),std::bind(&Cameraproperties::onGrpButtonchanged,this, TOF_HID_CID_IMU_DATA,std::placeholders::_1,std::placeholders::_2));
    connect(ui->face_status_chkb,&QCheckBox::stateChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int, int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_FACE_DETECTION,TOF_APP_VAL_FACE_DET_ENABLE, 1));
    connect(ui->face_overlay_chkb,&QCheckBox::stateChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int, int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_FACE_DETECTION,TOF_APP_VAL_FACE_DET_ENABLE, 0));
    connect(ui->smile_status_chkb,&QCheckBox::stateChanged,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_SMILE_DETECTION,TOF_APP_VAL_SMILE_DET_ENABLE));
    connect(ui->win_size_cmb,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_HID_CID_AUTO_EXP_ROI,TOF_APP_VAL_MANUAL_ROI));

    connect(ui->avg_depth_ir_grpbox,&QGroupBox::toggled,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_APP_CID_DEPTH_REGION,std::placeholders::_1));
    connect(ui->avg_depth_ir_btn_grp,static_cast<void (QButtonGroup::*)(int, bool)>(&QButtonGroup::buttonToggled),std::bind(&Cameraproperties::onGrpButtonchanged,this,TOF_APP_CID_DEPTH_REGION,std::placeholders::_1,std::placeholders::_2));
    connect(ui->cursor_color_btn_grp,static_cast<void (QButtonGroup::*)(int, bool)>(&QButtonGroup::buttonToggled),std::bind(&Cameraproperties::onGrpButtonchanged,this,TOF_APP_CID_CURSOR_COLOR,std::placeholders::_1,std::placeholders::_2));

    connect(ui->clipboard_btn_grp,SIGNAL(buttonPressed(int)),this,SLOT(onClipboardChanged(int)));
    connect(ui->update_btn,SIGNAL(clicked()),this,SIGNAL(getDepthIRdata()));

    connect(ui->spc_depth_grp_box,&QGroupBox::toggled,std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_APP_CID_SPC_DEPTH_RANGE,std::placeholders::_1));
    connect(ui->spc_depth_min_value,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_APP_CID_SPC_DEPTH_RANGE,std::placeholders::_1));
    connect(ui->spc_depth_max_value,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),std::bind(static_cast<void(Cameraproperties::*)(int,int)>(&Cameraproperties::onCamPropChanged),this,TOF_APP_CID_SPC_DEPTH_RANGE,std::placeholders::_1));
    clipboard->clear();
    camPropSlotsConnected = true;

}

void Cameraproperties::onGrpButtonchanged(int ctrlID, int id, bool checked)
{
    if(checked){
        switch (ctrlID) {
        case TOF_HID_CID_AUTO_EXP_ROI:
            if(id == TOF_APP_VAL_MANUAL_ROI){
                ui->win_size_cmb->setEnabled(true);
                ui->win_size_label->setEnabled(true);
            }else{
                ui->win_size_cmb->setEnabled(false);
                ui->win_size_label->setEnabled(false);
            }

            emit setCamProperty(ctrlID,id,ui->win_size_cmb->currentIndex()+1,0);
            break;
        case TOF_HID_CID_FACE_DETECTION:
            if(id){
                ui->face_status_chkb->setEnabled(true);
                ui->face_overlay_chkb->setEnabled(true);
            }else{
                ui->face_status_chkb->setCheckState(Qt::Unchecked);
                ui->face_overlay_chkb->setCheckState(Qt::Unchecked);
                ui->face_status_chkb->setEnabled(false);
                ui->face_overlay_chkb->setEnabled(false);
            }
            emit setCamProperty(ctrlID,id,ui->face_status_chkb->isChecked(),ui->face_overlay_chkb->isChecked());
			break;
        case TOF_HID_CID_SMILE_DETECTION:
            if(id){
                ui->smile_status_chkb->setEnabled(true);
            }else{
                ui->smile_status_chkb->setEnabled(false);
            }
            emit setCamProperty(ctrlID,id,ui->smile_status_chkb->isChecked(),0);
            break;
		case TOF_HID_CID_IMU_DATA:
			emit setCamProperty(ctrlID, id);
			break;
        case TOF_APP_CID_SPC_DEPTH_RANGE:
            if(ui->spc_depth_min_value->value() >= ui->spc_depth_max_value->value()) {
                if(ui->depth_range_cmb->currentIndex() == 1) {
                    ui->spc_depth_min_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
                    ui->spc_depth_max_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
                }
                else {
                    ui->spc_depth_min_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
                    ui->spc_depth_max_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
                }
            }
            else {
                emit setCamProperty(ctrlID,id,ui->spc_depth_min_value->value(),ui->spc_depth_max_value->value());
            }
            break;
        case TOF_APP_CID_DEPTH_REGION:
            if (((cDataMode == Depth_IR_RGB_VGA_Mode || cDataMode == Depth_IR_RGB_HD_Mode) && (!ui->rgbDCheckBox->isChecked()) )|| cDataMode == Depth_IR_Mode)
			{


				ui->IR_display_checkBox->setEnabled(true);
			}
			emit setCamProperty(ctrlID, id);
			break;
        case TOF_APP_CID_CURSOR_COLOR:
        case TOF_APP_CID_DEPTH_UNIT:
            emit setCamProperty(ctrlID,id);
            break;
        }
    }
}
void Cameraproperties::onCamPropChanged(int ctrlID,int value, QLabel *label)
{
    uint16_t slider_value;
    if (ctrlID == TOF_UVC_CID_WB_TEMP || ctrlID == TOF_UVC_CID_WB_AUTO)
    {
        label->setNum(value * 50);
        emit setCamProperty(ctrlID, value * 50);
    }
    else
    {
        label->setNum(value);
        emit setCamProperty(ctrlID, value);
    }
}

void Cameraproperties::onFilterTypeSelected(int ctrlID)
{
    switch (ctrlID)
    {
    case TOF_APP_ENABLE_SPATIAL_FILTER:
        emit setFilterType(ctrlID , ui->spatial_chkBox->isChecked());
        break;
    case TOF_APP_ENABLE_TEMPORAL_FILTER:
        emit setFilterType(ctrlID , ui->temporal_chkBox->isChecked());
        break;
    case TOF_APP_ENABLE_EDGE_DETECTION:
        emit setFilterType(ctrlID , ui->edge_chkBox->isChecked());
    default:
        break;
    }
}
void Cameraproperties::onCamPropChanged(int ctrlID,int value)
{
    switch (ctrlID) {

    case TOF_APP_CID_SPC_DEPTH_RANGE:
        if(ui->spc_depth_min_value->value() >= ui->spc_depth_max_value->value()) {
            if(ui->depth_range_cmb->currentIndex() == 1) {
                ui->spc_depth_min_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
                ui->spc_depth_max_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
            }
            else {
                ui->spc_depth_min_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
                ui->spc_depth_max_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
            }
        }
        else {
            emit setCamProperty(ctrlID,value,ui->spc_depth_min_value->value(),ui->spc_depth_max_value->value());
        }
        return;

    case TOF_HID_CID_SMILE_DETECTION:
        if(value){
            ui->smile_status_chkb->setEnabled(true);
        }else{
            ui->smile_status_chkb->setEnabled(false);
        }
        emit setCamProperty(ctrlID,value,ui->smile_status_chkb->isChecked(),0);
        if (ui->smile_status_chkb->isChecked() && ui->tabWidget->currentIndex() == 1)
		{
            //QMessageBox msgBox;
            //msgBox.setText("DepthVista_Face_and_Smile_Detection_Application_Note.pdf");
            //int result = msgBox.exec();
            //if(result == QMessageBox::Ok) {
            //    msgBox.close();
            //}
			msgBox->setWindowTitle("DepthVista");
			msgBox->setIcon(QMessageBox::Information);
			msgBox->setText("DepthVista_Face_and_Smile_Detection_Application_Note.pdf");
			msgBox->setStandardButtons(QMessageBox::Ok);
			msgBox->setWindowModality(Qt::NonModal);
			msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

			connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

			msgBox->show();
		}
        return;
    case TOF_HID_CID_AUTO_EXP_ROI:
        if(value == TOF_APP_VAL_MANUAL_ROI){
            ui->win_size_cmb->setEnabled(true);
        }else{
            ui->win_size_cmb->setEnabled(false);
        }
        emit setCamProperty(ctrlID,value,ui->win_size_cmb->currentIndex()+1,0);
        return;

    case TOF_UVC_CID_WB_AUTO:
        ui->wb_slider->setEnabled(!value);
        ui->cur_wb_value->setEnabled(!value);
        break;
    case TOF_UVC_CID_EXPOSURE_AUTO:
        ui->exposure_slider->setEnabled(!value);
        ui->cur_expo_value->setEnabled(!value);
		ui->gain_label->setEnabled(!value);
		ui->gain_slider->setEnabled(!value);
		ui->cur_gain_value->setEnabled(!value);
        value = !value;
        break;
    case TOF_APP_AVERAGE_IR_DISPLAY:
        value = ui->IR_display_checkBox->isChecked();
        break;
    case TOF_APP_CID_DEPTH_REGION:
		if (value == 0)
		{
			ui->IR_display_checkBox->setEnabled(false);
		}
		else
		{
			if (((cDataMode == Depth_IR_RGB_VGA_Mode || cDataMode == Depth_IR_RGB_HD_Mode) && (!ui->rgbDCheckBox->isChecked())) || cDataMode == Depth_IR_Mode)
			{
				ui->IR_display_checkBox->setEnabled(true);
			}
		}
        value = ui->mouse_ptr_btn->isChecked();
        break;

    }
    emit setCamProperty(ctrlID,value);
}

void Cameraproperties::onCamPropChanged(int ctrlID,int value, int specificFeature)
{

    switch (ctrlID) {
    case TOF_HID_CID_FACE_DETECTION:
        if(value){
            ui->face_status_chkb->setEnabled(true);
            ui->face_overlay_chkb->setEnabled(true);
        }else{
            ui->face_status_chkb->setEnabled(false);
            ui->face_overlay_chkb->setEnabled(false);
        }
        emit setCamProperty(ctrlID,value,ui->face_status_chkb->isChecked(),ui->face_overlay_chkb->isChecked());
        if (specificFeature == FACE_STATUS_FEATURE && ui->face_status_chkb->isChecked() && ui->tabWidget->currentIndex() == 1)
        {
            //QMessageBox msgBox;
            //msgBox.setText("DepthVista_Face_and_Smile_Detection_Application_Note.pdf");
            //int result = msgBox.exec();
            //if(result == QMessageBox::Ok) {
            //    msgBox.close();
            //}
			msgBox->setWindowTitle("DepthVista");
			msgBox->setIcon(QMessageBox::Information);
			msgBox->setText("DepthVista_Face_and_Smile_Detection_Application_Note.pdf");
			msgBox->setStandardButtons(QMessageBox::Ok);
			msgBox->setWindowModality(Qt::NonModal);
			msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

			connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

			msgBox->show();
        }
        if (specificFeature == FACE_OVERLAY_FEATURE && ui->face_overlay_chkb->isChecked() && ui->tabWidget->currentIndex() == 1)
        {
            //QMessageBox msgBox;
            //msgBox.setText("DepthVista_Face_and_Smile_Detection_Application_Note.pdf");
            //int result = msgBox.exec();
            //if(result == QMessageBox::Ok) {
            //    msgBox.close();
            //}
			msgBox->setWindowTitle("DepthVista");
			msgBox->setIcon(QMessageBox::Information);
			msgBox->setText("DepthVista_Face_and_Smile_Detection_Application_Note.pdf");
			msgBox->setStandardButtons(QMessageBox::Ok);
			msgBox->setWindowModality(Qt::NonModal);
			msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

			connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

			msgBox->show();
        }
    default:
        break;
    }
}

void Cameraproperties::onClipboardChanged(int id)
{
    if(id){
        ui->update_btn->click();
        QString originalText = clipboard->text();
        QTextStream(&originalText) << ui->avg_depth_value->text() <<"\t"<<ui->std_depth_dev_value->text() <<"\t"
                                   << ui->avg_irvalue->text()<<"\t"<<ui->std_ir_dev_value->text()<<"\n";
        clipboard->setText(originalText);
    }else{
        clipboard->clear();
    }
}
void Cameraproperties::deviceChanged(int index){
    if(index!=oldindex){
        if(index == 0){
            ui->device_list_cmb->setCurrentIndex(oldindex);
        }
        if(index!=-1 && index!=0){
            ui->tabWidget->setEnabled(true);
            ui->spc_depth_grp_box->setChecked(false);
            ui->imuValGroupBox->setChecked(false);
            emit inputselected(index);
        }
    }
}

void Cameraproperties::onFramesStopped(int index)
{
	ui->tabWidget->setEnabled(true);
	emit inputselected(index);
}


void Cameraproperties::onUpdateData(int avgDepth,int stdDepth,int avgIR,int stdIR)
{
    ui->avg_depth_value->setText(QString::number(avgDepth) + QString(" mm"));
    ui->std_depth_dev_value->setText(QString::number(stdDepth) + QString(" mm"));
    ui->avg_irvalue->setNum(avgIR);
    ui->std_ir_dev_value->setNum(stdIR);
}
bool Cameraproperties::eventFilter(QObject *obj, QEvent *event)
{
    if(obj==ui->device_list_cmb){
        if(event->type()==QEvent::MouseButtonPress||event->type()==QEvent::KeyPress)
        {
            emit enumerateDevices();
        }
    }
    return false;
}
void Cameraproperties::onOpenFolder()
{
    current_save_img_dir = save_img_dir;
    save_img_dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),"/home",
                                                         QFileDialog::ShowDirsOnly
                                                         | QFileDialog::DontResolveSymlinks);
    if(save_img_dir == "")
    {
        save_img_dir = current_save_img_dir;
    }
    ui->save_image_location->setText(save_img_dir);
}
void Cameraproperties::savePLYPressed()
{
    emit savePLYClicked(save_img_dir);
}

void Cameraproperties::applyAveChanges()
{
    emit applyAvgChangesClicked(ui->avg_x_pixel_spinBox->value(), ui->avg_y_pixel_spinBox->value());
}

void Cameraproperties::onexitpressed()
{
    this->deleteLater();
}

void Cameraproperties::onFrameSaveClicked()
{
    if(QSysInfo::productType() == "windows")
    {
        if(!QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista").exists()) {
            QDir().mkdir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista");
        }
        if(!QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista/Pictures").exists()) {
            QDir().mkdir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)+"/DepthVista/Pictures");
        }
    }
    else
    {
        if(!QDir("Pictures").exists()) {
            QDir().mkdir("Pictures");
        }
    }
	if (ui->DepthSaveCheckBox->isChecked() || ui->IRSaveCheckBox->isChecked() || ui->RGBSaveCheckBox->isChecked() ||
		ui->DepthRawSaveCheckBox->isChecked() || ui->PLYSaveCheckBox->isChecked())
	{
        ui->FrameSaveGroupBox->setFocus();
        ui->FrameSavePushButton->setEnabled(false);
        emit startSavingFrames(ui->DepthSaveCheckBox->isChecked(), ui->IRSaveCheckBox->isChecked(), ui->RGBSaveCheckBox->isChecked(),
			ui->DepthRawSaveCheckBox->isChecked(), ui->PLYSaveCheckBox->isChecked(), save_img_dir);
	}

}

void Cameraproperties::onSavingFramesComplete()
{
    if((ui->DepthRawSaveCheckBox->isChecked()==true || ui->IRSaveCheckBox->isChecked()==true || ui->DepthSaveCheckBox->isChecked()==true || ui->RGBSaveCheckBox->isChecked()==true) || (ui->PLYSaveCheckBox->isChecked()==true))
    {
        //QMessageBox msgBox;
        //msgBox.setIcon(QMessageBox::Information);
        //msgBox.setText("Image(s) saved successfully!!");
        //int result = msgBox.exec();
        //if(result==QMessageBox::Ok)
        //{
        //    msgBox.close();
        //}
		msgBox->setWindowTitle("DepthVista");
		msgBox->setIcon(QMessageBox::Information);
		msgBox->setText("Image(s) saved successfully!!");
		msgBox->setStandardButtons(QMessageBox::Ok);
		msgBox->setWindowModality(Qt::NonModal);
		msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

		connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

		msgBox->show();
    }
    ui->FrameSavePushButton->setEnabled(true);
}

void Cameraproperties::onOthersavedoneplyfail()
{
    if(((ui->DepthRawSaveCheckBox->isChecked()==true || ui->IRSaveCheckBox->isChecked()==true || ui->DepthSaveCheckBox->isChecked()==true || ui->RGBSaveCheckBox->isChecked()==true) && ui->PLYSaveCheckBox->isChecked()==true))
    {
        //QMessageBox msgBox;
        //msgBox.setIcon(QMessageBox::Information);
        //msgBox.setText("PLY image not saved since No vertices found. Other image(s) saved successfully");
        //int result = msgBox.exec();
        //if(result == QMessageBox::Ok) {
        //    msgBox.close();
        //}
		msgBox->setWindowTitle("DepthVista");
		msgBox->setIcon(QMessageBox::Warning);
		msgBox->setText("PLY image not saved since No vertices found. Other image(s) saved successfully");
		msgBox->setStandardButtons(QMessageBox::Ok);
		msgBox->setWindowModality(Qt::NonModal);
		msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

		connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

		msgBox->show();
    }
    if((ui->DepthRawSaveCheckBox->isChecked()==false && ui->IRSaveCheckBox->isChecked()==false && ui->PLYSaveCheckBox->isChecked()==true && ui->DepthSaveCheckBox->isChecked()==false && ui->RGBSaveCheckBox->isChecked()==false))
    {
        //QMessageBox msgBox;
        //msgBox.setIcon(QMessageBox::Warning);
        //msgBox.setText("PLY image not saved since No vertices found");
        //int result = msgBox.exec();
        //if(result == QMessageBox::Ok) {
        //    msgBox.close();
        //}
		msgBox->setWindowTitle("DepthVista");
		msgBox->setIcon(QMessageBox::Warning);
		msgBox->setText("PLY image not saved since No vertices found");
		msgBox->setStandardButtons(QMessageBox::Ok);
		msgBox->setWindowModality(Qt::NonModal);
		msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

		connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

		msgBox->show();
    }
    ui->FrameSavePushButton->setEnabled(true);
}

void Cameraproperties::enableAndDisableSaveButton()
{
    if(ui->DepthRawSaveCheckBox->checkState()==Qt::Checked || ui->DepthSaveCheckBox->checkState()==Qt::Checked || ui->IRSaveCheckBox->checkState()==Qt::Checked || ui->PLYSaveCheckBox->checkState()==Qt::Checked || ui->RGBSaveCheckBox->checkState()==Qt::Checked)
    {
        ui->FrameSavePushButton->setEnabled(true);
    }
    else
    {
        ui->FrameSavePushButton->setEnabled(false);
    }
}

void Cameraproperties::onDeviceRemoved()
{
    ui->avg_depth_value->setText("0 mm");
    ui->std_depth_dev_value->setText("0 mm");
    ui->avg_irvalue->setText("0");
    ui->std_ir_dev_value->setText("0");
    disconnect(ui->device_list_cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(deviceChanged(int)));
    ui->device_list_cmb->setCurrentIndex(0);
    connect(ui->device_list_cmb, SIGNAL(currentIndexChanged(int)), this, SLOT(deviceChanged(int)));
	ui->win_size_cmb->setCurrentIndex(0);
	ui->spc_depth_grp_box->setChecked(false);
    ui->tabWidget->setEnabled(false);
    ui->rgbDCheckBox->setCheckState(Qt::CheckState::Unchecked);

}

void Cameraproperties::onTofCamModeSelected(bool ir_mode)
{
    ui->RGBSaveCheckBox->setEnabled(false);
    ui->scrollArea_2->setEnabled(false);

    if(cDataMode == DataMode::Depth_Mode || cDataMode == DataMode::Depth_IR_Mode) {
		ui->depth_ir_data_grpbox->setEnabled(true);
        ui->postProcessingGrpBox->setEnabled(true);
        ui->tof_coring_label->setEnabled(true);
        ui->tof_coring_spinBox->setEnabled(true);
        ui->avg_depth_ir_grpbox->setEnabled(true);
        ui->spc_depth_grp_box->setEnabled(true);

        if(cDataMode == Depth_Mode) {
            ui->tof_gain_label->setEnabled(false);
            ui->tof_gain_slider->setEnabled(false);
            ui->cur_tof_gain_value->setEnabled(false);
            ui->IRSaveCheckBox->setEnabled(false);
        }
        else {
            ui->tof_gain_label->setEnabled(true);
            ui->tof_gain_slider->setEnabled(true);
            ui->cur_tof_gain_value->setEnabled(true);
        }

    }
    else {
        ui->postProcessingGrpBox->setEnabled(false);
        ui->postProcessingGrpBox->setChecked(false);
        ui->tof_coring_label->setEnabled(false);
        ui->tof_coring_spinBox->setEnabled(false);
        ui->avg_depth_ir_grpbox->setEnabled(false);
        ui->spc_depth_grp_box->setEnabled(false);
        ui->spc_depth_grp_box->setChecked(false);
        ui->tof_gain_label->setEnabled(true);
        ui->tof_gain_slider->setEnabled(true);
        ui->cur_tof_gain_value->setEnabled(true);
        ui->depth_ir_data_grpbox->setEnabled(false);
    }

    //TOF range control
    ui->depth_range_label->setEnabled(true);
    ui->depth_range_cmb->setEnabled(true);
    ui->RangeApplyButton->setEnabled(true);
    //TOF range control
    ui->label_27->setEnabled(true);
    ui->label_30->setEnabled(true);
    ui->min_depth_val->setEnabled(true);
    ui->max_depth_val->setEnabled(true);
    //RGB-D mapping control
	ui->threeDgroupBox->setEnabled(ir_mode);
	//ui->threeDgroupBox->set;
	ui->rgbDCheckBox->setEnabled(false);
	ui->undistortChkBx->setEnabled(true);
    //Gain control
    ui->gain_label->setEnabled(true);
    ui->gain_slider->setEnabled(true);
    ui->cur_gain_value->setEnabled(true);
    //Exposure compensation control
    ui->expo_comp_label->setEnabled(true);
    ui->cur_expo_comp_value->setEnabled(true);
    //Frame rate control
    ui->fps_ctrl_label->setEnabled(true);
    ui->cur_fps_ctrl_value->setEnabled(true);
    //ROI Layout control
    ui->roi_layout->setEnabled(true);
}

void Cameraproperties::onDualCamModeSelected(bool vgaMode)
{
    ui->RGBSaveCheckBox->setEnabled(true);
    ui->scrollArea_2->setEnabled(true);

    //TOF range control
    ui->depth_range_label->setEnabled(true);
    ui->depth_range_cmb->setEnabled(true);
    ui->RangeApplyButton->setEnabled(true);
    //TOF range control
    ui->label_27->setEnabled(true);
    ui->label_30->setEnabled(true);
    ui->min_depth_val->setEnabled(true);
    ui->max_depth_val->setEnabled(true);

    //RGB-D mapping control
    ui->rgbDCheckBox->setEnabled(true);
    ui->undistortChkBx->setEnabled(!(ui->rgbDCheckBox->isChecked()));
	ui->threeDgroupBox->setEnabled(true);
    //Post Processing control
    ui->postProcessingGrpBox->setEnabled(true);
    //Post Processing control
    ui->tof_coring_label->setEnabled(true);
    ui->tof_coring_spinBox->setEnabled(true);
    ui->tof_gain_label->setEnabled(true);
    ui->tof_gain_slider->setEnabled(true);
    ui->cur_tof_gain_value->setEnabled(true);

    ui->avg_depth_ir_grpbox->setEnabled(true);
    ui->depth_ir_data_grpbox->setEnabled(true);
    ui->spc_depth_grp_box->setEnabled(true);

    //Gain control
    ui->gain_label->setEnabled(false);
    ui->gain_slider->setEnabled(false);
    ui->cur_gain_value->setEnabled(false);
	//Exposure controls
	ui->exposure_label->setEnabled(false);
	ui->expo_auto_btn->setEnabled(false);
	ui->exposure_slider->setEnabled(false);
	ui->cur_expo_value->setEnabled(false);
    //Exposure compensation control
    ui->expo_comp_label->setEnabled(false);
    ui->cur_expo_comp_value->setEnabled(false);
	//Orientation Ctrl
	ui->orientation_label->setEnabled(false);
	ui->orientation_cmb->setEnabled(false);

    //Frame rate control
    ui->fps_ctrl_label->setEnabled(false);
    ui->cur_fps_ctrl_value->setEnabled(false);
    //ROI Layout control
    ui->roi_layout->setEnabled(false);
}

void Cameraproperties::onRgbCamModeSelected(uint8_t frameRateCtrl, uint32_t expoComp, uint8_t ROIMode, uint8_t winSize)
{
    ui->RGBSaveCheckBox->setEnabled(true);
    ui->scrollArea_2->setEnabled(true);
	//Orientation Control
	ui->orientation_label->setEnabled(true);
	ui->orientation_cmb->setEnabled(true);
    //Gain control
    ui->gain_label->setEnabled(!ui->expo_auto_btn->isChecked());
    ui->gain_slider->setEnabled(!ui->expo_auto_btn->isChecked());
    ui->cur_gain_value->setEnabled(!ui->expo_auto_btn->isChecked());
	//Exposure compensation control
	ui->exposure_label->setEnabled(true);
	ui->expo_auto_btn->setEnabled(true);
	ui->exposure_slider->setEnabled(!ui->expo_auto_btn->isChecked());
	ui->cur_expo_value->setEnabled(!ui->expo_auto_btn->isChecked());
    //Exposure compensation control
    ui->expo_comp_label->setEnabled(true);
    ui->cur_expo_comp_value->setEnabled(true);
    ui->cur_expo_comp_value->setValue(expoComp);
	//Orientation ctrl
	ui->orientation_label->setEnabled(true);
	ui->orientation_cmb->setEnabled(true);
    //Frame rate control
    ui->fps_ctrl_label->setEnabled(true);
    ui->cur_fps_ctrl_value->setEnabled(true);
    ui->cur_fps_ctrl_value->setValue(frameRateCtrl);
    //ROI Layout control
    ui->roi_layout->setEnabled(true);
	if (!ROIMode) {
		ui->roi_disable_btn->setChecked(true);
		ui->win_size_label->setEnabled(false);

	}
	else if (ROIMode == TOF_APP_VAL_FULL_ROI) {
		ui->full_roi_btn->setChecked(true);
		ui->win_size_label->setEnabled(false);
	}
	else if (ROIMode == TOF_APP_VAL_MANUAL_ROI) {
		ui->win_size_cmb->setEnabled(true);
		ui->win_size_label->setEnabled(true);
		ui->manual_roi_btn->setChecked(true);
	}
		ui->auto_roi_btn_grp->setId(ui->roi_disable_btn, TOF_APP_VAL_DISABLE_ROI);
		ui->auto_roi_btn_grp->setId(ui->full_roi_btn, TOF_APP_VAL_FULL_ROI);
		ui->auto_roi_btn_grp->setId(ui->manual_roi_btn, TOF_APP_VAL_MANUAL_ROI);

		if (ui->manual_roi_btn->isChecked()) {
			ui->win_size_cmb->setCurrentIndex(winSize - 1);
		}
		else {
			ui->win_size_cmb->setEnabled(ui->manual_roi_btn->isChecked());
		}

    //TOF range control
    ui->depth_range_label->setEnabled(false);
    ui->depth_range_cmb->setEnabled(false);
    ui->RangeApplyButton->setEnabled(false);
    //TOF range control
    ui->label_27->setEnabled(false);
    ui->label_30->setEnabled(false);
    ui->min_depth_val->setEnabled(false);
    ui->max_depth_val->setEnabled(false);
    //RGB-D mapping control
	ui->threeDgroupBox->setEnabled(false);

    //Post Processing control
    ui->postProcessingGrpBox->setEnabled(false);
    ui->postProcessingGrpBox->setChecked(false);
    //Post Processing control
    ui->tof_coring_label->setEnabled(false);
    ui->tof_coring_spinBox->setEnabled(false);
    ui->tof_gain_label->setEnabled(false);
    ui->tof_gain_slider->setEnabled(false);
    ui->cur_tof_gain_value->setEnabled(false);

    ui->avg_depth_ir_grpbox->setEnabled(false);
    ui->avg_depth_ir_grpbox->setChecked(false);
    ui->depth_ir_data_grpbox->setEnabled(false);
    ui->depth_ir_data_grpbox->setChecked(false);
    ui->spc_depth_grp_box->setEnabled(false);
    ui->spc_depth_grp_box->setChecked(false);

}

void Cameraproperties::onFirmwareVersionRead(uint8_t gMajorVersion, uint8_t gMinorVersion1, uint16_t gMinorVersion2, uint16_t gMinorVersion3)
{
    //QMessageBox msgBox;
    //msgBox.setText("Firmware Version : "+QString::number(gMajorVersion) + "." + QString::number(gMinorVersion1) + "." + QString::number(gMinorVersion2) + "." + QString::number(gMinorVersion3));
    //int result = msgBox.exec();
    //if(result == QMessageBox::Ok) {
    //    msgBox.close();
    //}
	msgBox->setWindowTitle("Firmware Version");
	msgBox->setIcon(QMessageBox::Information);
	msgBox->setText("Firmware Version : " + QString::number(gMajorVersion) + "." + QString::number(gMinorVersion1) + "." + QString::number(gMinorVersion2) + "." + QString::number(gMinorVersion3));
	msgBox->setStandardButtons(QMessageBox::Ok);
	msgBox->setWindowModality(Qt::NonModal);
	msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

	connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

	msgBox->show();
}

void Cameraproperties::onUniqueIDRead(uint64_t uniqueID)
{
    //QMessageBox msgBox;
    //msgBox.setText("Unique ID : "+QString::number(uniqueID));
    //int result = msgBox.exec();
    //if(result == QMessageBox::Ok) {
    //    msgBox.close();
    //}
	msgBox->setWindowTitle("Unique ID");
	msgBox->setIcon(QMessageBox::Information);
	msgBox->setText("Unique ID : " + QString::number(uniqueID));
	msgBox->setStandardButtons(QMessageBox::Ok);
	msgBox->setWindowModality(Qt::NonModal);
	msgBox->setWindowFlags(msgBox->windowFlags() & ~Qt::WindowContextHelpButtonHint);

	connect(msgBox, &QMessageBox::finished, msgBox, &QMessageBox::close);

	msgBox->show();
}

void Cameraproperties::RGBDMapping_checkBox_stateChange(int state)
{
	if (state)
	{
		ui->undistortChkBx->setEnabled(false);
        ui->IR_display_checkBox->setEnabled(false);
	}
	else
	{
        if (ui->avg_depth_ir_grpbox->isChecked())
		{
			ui->IR_display_checkBox->setEnabled(true);
		}
		ui->undistortChkBx->setEnabled(true);
	}

}


Cameraproperties::~Cameraproperties()
{
    delete ui;
}

void Cameraproperties::on_DatamodeApplyButton_clicked()
{
    ui->DepthRawSaveCheckBox->setCheckState(Qt::CheckState::Unchecked);
    ui->IRSaveCheckBox->setCheckState(Qt::CheckState::Unchecked);
    ui->PLYSaveCheckBox->setCheckState(Qt::CheckState::Unchecked);
    ui->DepthSaveCheckBox->setCheckState(Qt::CheckState::Unchecked);
    ui->RGBSaveCheckBox->setCheckState(Qt::CheckState::Unchecked);

    int value = ui->data_mode_cmb->currentIndex();
    if(dataModeCurrentIndex != value) {
        dataModeCurrentIndex = value;
        value = (DataMode)((value == 0) ? value : value = value + 1);

        if(value==IR_Mode || value>=RGB_VGA_Mode /*|| value==Raw_Mode*/){
            if(ui->avg_depth_ir_grpbox->isChecked()){
                ui->avg_depth_ir_grpbox->setChecked(false);
            }

        }
        if (value == Depth_IR_Mode)
        {
            ui->avg_depth_ir_grpbox->setChecked(false);
            ui->IR_display_checkBox->setEnabled(ui->avg_depth_ir_grpbox->isChecked());
            ui->DepthRawSaveCheckBox->setEnabled(true);
            ui->IRSaveCheckBox->setEnabled(true);
            ui->PLYSaveCheckBox->setEnabled(true);
            ui->DepthSaveCheckBox->setEnabled(true);
            ui->RGBSaveCheckBox->setEnabled(false);
        }
        else if (value == Depth_Mode)
        {
			ui->avg_depth_ir_grpbox->setChecked(false);
            ui->IR_display_checkBox->setEnabled(false);
            ui->IR_display_checkBox->setChecked(false);
            ui->DepthRawSaveCheckBox->setEnabled(true);
            ui->IRSaveCheckBox->setEnabled(false);
            ui->PLYSaveCheckBox->setEnabled(true);
            ui->DepthSaveCheckBox->setEnabled(true);
            ui->RGBSaveCheckBox->setEnabled(false);
        }
        else if (value == IR_Mode)
        {
            ui->IR_display_checkBox->setChecked(false);
            ui->DepthRawSaveCheckBox->setEnabled(false);
            ui->IRSaveCheckBox->setEnabled(true);
            ui->PLYSaveCheckBox->setEnabled(false);
            ui->DepthSaveCheckBox->setEnabled(false);
            ui->RGBSaveCheckBox->setEnabled(false);
        }
        else if (value == Depth_IR_RGB_VGA_Mode || value == Depth_IR_RGB_HD_Mode)
        {
			if (ui->orientation_cmb->currentIndex() != 0)
			{
				ui->orientation_cmb->setCurrentIndex(0);
			}
			if (ui->expo_auto_btn->isChecked() == false)
			{
				ui->expo_auto_btn->setChecked(true);
			}
            ui->avg_depth_ir_grpbox->setChecked(false);
			ui->IR_display_checkBox->setEnabled(ui->avg_depth_ir_grpbox->isChecked() && !ui->rgbDCheckBox->isChecked());
            ui->DepthRawSaveCheckBox->setEnabled(true);
            ui->IRSaveCheckBox->setEnabled(true);
            ui->PLYSaveCheckBox->setEnabled(true);
            ui->DepthSaveCheckBox->setEnabled(true);
            ui->RGBSaveCheckBox->setEnabled(true);
        }
        else if (value >= RGB_VGA_Mode)
        {
			ui->orientation_cmb->setEnabled(true);
            ui->IR_display_checkBox->setChecked(false);
            ui->DepthRawSaveCheckBox->setEnabled(false);
            ui->IRSaveCheckBox->setEnabled(false);
            ui->PLYSaveCheckBox->setEnabled(false);
            ui->DepthSaveCheckBox->setEnabled(false);
            ui->RGBSaveCheckBox->setEnabled(true);
        }
        cDataMode = (DataMode)value;
        value = (value == 0) ? value : value - 1;
        emit setCamProperty(TOF_UVC_EXT_CID_DATA_MODE,value);

        ui->avg_depth_value->setText("0 mm");
        ui->std_depth_dev_value->setText("0 mm");
        ui->avg_irvalue->setText("0");
        ui->std_ir_dev_value->setText("0");
    }
}

void Cameraproperties::on_RangeApplyButton_clicked()
{
    int value = ui->depth_range_cmb->currentIndex();
    if(depthRangeCurrentIndex != value) {
        depthRangeCurrentIndex = value;
        if (value) {
            ui->spc_depth_min_value->setMinimum(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
            ui->spc_depth_min_value->setMaximum(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
            ui->spc_depth_max_value->setMinimum(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
            ui->spc_depth_max_value->setMaximum(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
            ui->spc_depth_min_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MIN);
            ui->spc_depth_max_value->setValue(TOF_APP_VAL_FAR_SPC_DEPTH_MAX);
        }
        else {
            ui->spc_depth_min_value->setMinimum(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
            ui->spc_depth_min_value->setMaximum(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
            ui->spc_depth_max_value->setMinimum(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
            ui->spc_depth_max_value->setMaximum(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
            ui->spc_depth_min_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MIN);
            ui->spc_depth_max_value->setValue(TOF_APP_VAL_NEAR_SPC_DEPTH_MAX);
        }
        emit setCamProperty(TOF_UVC_EXT_CID_DEPTH_RANGE,value);
    }
}
