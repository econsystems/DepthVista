#include "openglpcl.h"

#include <QOpenGLShader>
#include <QOpenGLTexture>
#include <QCoreApplication>
#include <QResizeEvent>

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_TEXCOORD_ATTRIBUTE 1

#define ATTRIB_VERTEX 0
#define ATTRIB_TEXTURE 1

float *imgdata = NULL;


struct OpenGLPCL::OpenGLPCLImpl
{
    OpenGLPCLImpl()
        :mFrameSize(0)
    {}

    int                     mFrameSize;

    QOpenGLShader*          mVShader;
    QOpenGLShader*          mFShader;
    QOpenGLShaderProgram*   mShaderProgram;
    GLsizei                 mVideoW, mVideoH;

};

OpenGLPCL::OpenGLPCL(QWidget* parent , PixelformatPCL pixformat)
    : QOpenGLWidget(parent)
    , pixelFormat(pixformat)
    , impl(new OpenGLPCLImpl)
{

    srcBuffer = NULL;
	renderMutex.lock();

	if (imgdata == NULL)
	{
		imgdata = (float*)malloc(640 * 480 * 3 * sizeof(float));
		if (imgdata == NULL)
		{
			qDebug() << "imgaData memory allocation failed";
		}else
			qDebug() << "imgaData memory allocation success";

	}

	renderMutex.unlock();

	currentWidth = 640;
	currentHeight = 480;
    connect(this,SIGNAL(maximized(QWidget*)),parent,SLOT(onwidgetMaximized(QWidget*)));
    connect(this,SIGNAL(renderframe()),this, SLOT(updateFrame()));
    connect(this,SIGNAL(ply_file_save(int)),this, SLOT(end_ply_saving_thread()));
    connect(this,SIGNAL(ply_file_save(int)), parent, SLOT(savingPLYFramesOver(int)));
    lastMousePos.setX(1);
    lastMousePos.setY(1);
    initialFoV = 15;
    local_pos = {0.0,0.0,0.0};
    initialPos = true;
    local_rot = {0.0,0.0,0.0};
    position = { 0.0,0.0,5.0};
    cameraMatrix.setToIdentity();
    cameraMatrix.translate(position);
    horizontalAngle = 0.0f;
    verticalAngle = 0.0f;
}
void OpenGLPCL::updateFrame()
{
    update();
}
OpenGLPCL::~OpenGLPCL()
{
	renderMutex.lock();
    if (imgdata) {
        free(imgdata);
        imgdata = NULL;
		if(imgdata == NULL)
			qDebug() << "imgaData memory free success";
		else
			qDebug() << "imgaData memory free failed";

    }
	renderMutex.unlock();


}

void OpenGLPCL::dataModeChanged(uint8_t dataMode)
{
	renderMutex.lock();

	if (imgdata != NULL) {
		free(imgdata);
		imgdata = NULL;
		if (imgdata == NULL)
			qDebug() << "imgaData memory free success datamode change";
		else
			qDebug() << "imgaData memory free failed";
	}
	if (imgdata == NULL) {
		if (dataMode == Depth_IR_RGB_VGA_Mode) {
			currentWidth = 640;
			currentHeight = 480;
			imgdata = (float*)malloc(640 * 480 * 3 * sizeof(float));
			if (imgdata == NULL) {
				qDebug() << "imgaData memory allocation failed";
			}
			else
				qDebug() << "imgaData memory allocation success VGA";
		}
		else if (dataMode == Depth_IR_RGB_HD_Mode) {
			currentWidth = 1280;
			currentHeight = 720;
			imgdata = (float*)malloc(1280 * 720 * 3 * sizeof(float));
			if (imgdata == NULL) {
				qDebug() << "imgaData memory allocation failed";
			}
			else
				qDebug() << "imgaData memory allocation success HD";
		}


	}
	renderMutex.unlock();

}

void OpenGLPCL::InitDrawBuffer(unsigned bsize)
{
    renderMutex.lock();
    gotFrame = false;
    impl->mFrameSize = bsize;
    renderMutex.unlock();
}

void OpenGLPCL::setSavePLYfile(QString file_name, bool save_specific_range, uint16_t depth_min_val, uint16_t depth_max_val)
{
    plyFileName = file_name;
    savePLYSpecificRange = save_specific_range;
	depth_min_val_ply = depth_min_val;
	depth_max_val_ply = depth_max_val;
    savePLYfile = true;
    points.clear();
    if (ply_thread_running == false) {
        ply_thread_running = true;
        savePLYThread = QtConcurrent::run(this, &OpenGLPCL::saveplyFile);
    }
    
}

void OpenGLPCL::end_ply_saving_thread()
{
    if (ply_thread_running) {
        ply_thread_running = false;
        if (savePLYThread.isRunning()) {
            savePLYThread.waitForFinished();
        }
    }
}


void OpenGLPCL::saveplyFile()
{

    while(ply_thread_running)
    {
#ifdef  __linux__
        usleep(1000);
#elif _WIN32
        Sleep(1);
#endif
        if (save_cloud_ready)
        {
            int ret;
            if(points.size() > 0)
            {
                ret = 0;
            }
            else
            {
                ret = -1;
                save_cloud_ready = false;
                emit ply_file_save(ret);
                return;
            }

            #define PLY_START_HEADER "ply"
            #define PLY_END_HEADER "end_header"
            #define PLY_ASCII "format ascii 1.0"
            #define PLY_ELEMENT_VERTEX "element vertex"

            // save to the ply file
            std::ofstream ofs(plyFileName.toStdString()); // text mode first
            ofs << PLY_START_HEADER << std::endl;
            ofs << PLY_ASCII << std::endl;
            ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
            ofs << "property float x" << std::endl;
            ofs << "property float y" << std::endl;
            ofs << "property float z" << std::endl;
            ofs << "property uchar red" << std::endl;
            ofs << "property uchar green" << std::endl;
            ofs << "property uchar blue" << std::endl;
            ofs << PLY_END_HEADER << std::endl;
            ofs.close();
            std::stringstream ss;
            for (size_t i = 0; i < points.size(); ++i)
            {
                // image data is BGR
                ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
                ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
                ss << std::endl;
            }

            std::ofstream ofs_text(plyFileName.toStdString(), std::ios::out | std::ios::app);
            ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());

            save_cloud_ready = false;
            emit ply_file_save(ret);
        }
    }
}

void OpenGLPCL::DisplayVideoFrame(unsigned char* depth_data, unsigned char* rgb_data, int frameWidth, int frameHeight)
{
    renderMutex.lock();
    impl->mVideoW = frameWidth;
    impl->mVideoH = frameHeight;
	cv::Mat DepthFloat;
    color_point_t pclPoint;


    switch (pixelFormat) {
    case RGB_PCL:
        srcBuffer = depth_data;
        break;
    }
	float fx, fy, cx, cy;
    float zoom_factor = 1.0;
    int count = 0;
    float Z;
    uint16_t raw_depth;
    int u, v;
    float x,y,z;

    DepthImg = cv::Mat(frameHeight,frameWidth,CV_16UC1);
	memcpy(DepthImg.data, (uint8_t*)srcBuffer, frameHeight * frameWidth * 2);
	if (rgb_data != NULL) {
		Depthcolormap = cv::Mat(frameHeight, frameWidth, CV_8UC3, (uint8_t*)rgb_data);
	}

    if (savePLYfile) {
        points.clear();
    }

    DepthImg.convertTo(DepthFloat, CV_32F);
    if (!DepthFloat.data) {
        qDebug() << "convertTo Failed";
    }

    if (rgb_data == NULL) {
		DepthImg.convertTo(PrevDepthFrame, CV_8UC1, alpha, -beta);

        applyColorMap(PrevDepthFrame, Depthcolormap, colorMap);
    }

    for (v = 0; v < DepthFloat.rows; v++) {
        for (u = 0; u < DepthFloat.cols; u++) {
            Z = DepthFloat.at<float>(v, u) / zoom_factor;
            raw_depth = DepthImg.at<uint16_t>(v, u);

            if (raw_depth < depth_min_val || raw_depth > depth_max_val) {
                imgdata[count] = 0;
                imgdata[count + 1] = 0;
                imgdata[count + 2] = 0;
                count += 3;
                continue;
            }
            if (currentDepthRange == 1) {
                z = Z * 2;
                x = (u - principlePointx) * Z * 2/ focalLengthx;
                y = ((v - principlePointy) * Z * 2/ focalLengthy);
            }
            else {
                z = Z;
                x = (u - principlePointx) * Z / focalLengthx;
                y = ((v - principlePointy) * Z / focalLengthy);
            }
            imgdata[count] = x ;
            imgdata[count+1] = -y ;
            imgdata[count+2] = -z ;
            count += 3;
            if (savePLYfile) {
                if (savePLYSpecificRange && ((raw_depth < depth_min_val_ply) || (raw_depth > depth_max_val_ply)))
                    continue;
                pclPoint.xyz[0] = x / 1000;
                pclPoint.xyz[1] = -y / 1000;
                pclPoint.xyz[2] = -z / 1000;
                pclPoint.rgb[0] = Depthcolormap.at<cv::Vec3b>(v, u)[0];
                pclPoint.rgb[1] = Depthcolormap.at<cv::Vec3b>(v, u)[1];
                pclPoint.rgb[2] = Depthcolormap.at<cv::Vec3b>(v, u)[2];
                points.push_back(pclPoint);
            }
        }
    }
	

    if (savePLYfile) {
        save_cloud_ready = true;
        savePLYfile = false;
    }
    count = 0;

    gotFrame = true;
    renderMutex.unlock();
    emit renderframe();

}

void OpenGLPCL::shaderRGB()
{
    // Initialize the vertex shader object
    impl->mVShader = new QOpenGLShader(QOpenGLShader::Vertex, this);

    //Vertex shader source
    const char *vsrc = "attribute vec3 vertexIn; \
        attribute vec3 textureIn; \
        varying vec3 textureOut;  \
        uniform mat4 view;       \
        uniform mat4 projection;  \
        void main(void)           \
        {                         \
            gl_Position = projection*view*vec4(vertexIn/1000.0,1.0); \
            textureOut = textureIn; \
        }";

    //Compile the vertex shader program
    bool bCompile = impl->mVShader->compileSourceCode(vsrc);
    if(!bCompile) {
        throw OpenGlExceptionPCL();
    }

    // Initialize the fragment shader function yuv converted to rgb
    impl->mFShader = new QOpenGLShader(QOpenGLShader::Fragment, this);

    // Fragment shader source code
    const char *fsrc = "#ifdef GL_ES\n"
                     "precision highp float;\n"
                     "#endif\n"

                     "varying vec3 textureOut;\n"

                     "void main()\n"
                     "{\n"
                       "   gl_FragColor = vec4(textureOut.bgr,1.0);"
                       "}";



    bCompile = impl->mFShader->compileSourceCode(fsrc);
    if(!bCompile) {
        throw OpenGlExceptionPCL();
    }

    // Create a shader program container
    impl->mShaderProgram = new QOpenGLShaderProgram(this);
    // Add the fragment shader to the program container
    impl->mShaderProgram->addShader(impl->mFShader);
    // Add a vertex shader to the program container
    impl->mShaderProgram->addShader(impl->mVShader);
    // Bind the property vertexIn to the specified location ATTRIB_VERTEX, this property
    // has a declaration in the vertex shader source
    impl->mShaderProgram->bindAttributeLocation("vertexIn", ATTRIB_VERTEX);
    // Bind the attribute textureIn to the specified location ATTRIB_TEXTURE, the attribute
    // has a declaration in the vertex shader source
    impl->mShaderProgram->bindAttributeLocation("textureIn", ATTRIB_TEXTURE);
    //Link all the shader programs added to
    impl->mShaderProgram->link();

    mPositionLoc = impl->mShaderProgram->attributeLocation("vertexIn");
    mTexCoordLoc = impl->mShaderProgram->attributeLocation("textureIn");

    modelID = impl->mShaderProgram->uniformLocation("model");
    projectionID = impl->mShaderProgram->uniformLocation("projection");
    viewID = impl->mShaderProgram->uniformLocation("view");

    impl->mShaderProgram->bind();

    impl->mShaderProgram->enableAttributeArray(0);
    impl->mShaderProgram->enableAttributeArray(1);

}

void OpenGLPCL::renderRGB()
{
    vao.bind();
    vbo.bind();
    vbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo.allocate(imgdata, currentWidth * currentHeight * 3 * sizeof(float));
    impl->mShaderProgram->enableAttributeArray(mPositionLoc);
    impl->mShaderProgram->setAttributeBuffer(mPositionLoc, GL_FLOAT, 0, 3, 0);
    vbo.release();

    vbo1.bind();
    vbo1.setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo1.allocate(Depthcolormap.data,(impl->mVideoW*impl->mVideoH*3));
    impl->mShaderProgram->enableAttributeArray(mTexCoordLoc);
    impl->mShaderProgram->setAttributeBuffer(mTexCoordLoc, GL_UNSIGNED_BYTE, 0, 3, 0);
    vbo1.release();

    computeMatricesFromInputs();
    impl->mShaderProgram->setUniformValue(viewID,ViewMatrix);
    impl->mShaderProgram->setUniformValue(projectionID,ProjectionMatrix);
    renderMutex.lock();
    if(gotFrame) {
        glDrawArrays(GL_POINTS, 0, impl->mVideoW*impl->mVideoH);
    }
    vao.release();
    renderMutex.unlock();
}

void OpenGLPCL::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    vao.create();

    switch (pixelFormat) {
    case RGB_PCL:
        shaderRGB();
        break;
    }
    vao.bind();
    vbo.create();                       //vbo for xyz position data
    vbo.bind();
    vbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo.allocate(imgdata, currentWidth * currentHeight * 3 * sizeof(float));
    impl->mShaderProgram->enableAttributeArray(mPositionLoc);
    impl->mShaderProgram->setAttributeBuffer(mPositionLoc, GL_FLOAT, 0, 3, 0);
    vbo.release();

    vbo1.create();                      //vbo for rgb color data
    vbo1.bind();
    vbo1.setUsagePattern(QOpenGLBuffer::StaticDraw);
    vbo1.allocate(Depthcolormap.data,impl->mVideoW*impl->mVideoH*3);
    impl->mShaderProgram->enableAttributeArray(mTexCoordLoc);
    impl->mShaderProgram->setAttributeBuffer(mTexCoordLoc, GL_UNSIGNED_BYTE, 0, 3, 0);
    vbo1.release();

    vao.release();
    glClearColor (0, 0, 0, 1);
}

void OpenGLPCL::resizeGL(int w, int h)
{
    if(h == 0)// prevents being divided by zero
        h = 1;// set the height to 1

    // Set the viewport
    glViewport(0, 0, w, h);
    update();
}

void OpenGLPCL::paintGL()
{
    if(gotFrame) {

        if(impl->mShaderProgram) {
            switch (pixelFormat) {
            case RGB_PCL:
                renderRGB();
                break;
            }
        }

    }
}

void OpenGLPCL::computeMatricesFromInputs()
{
    if(initialPos) {
        initialFoV = 15;
        local_qrot = local_qrot.fromEulerAngles(local_rot);
        ViewMatrix.setToIdentity();
        ViewMatrix.translate(local_pos);
        ViewMatrix.rotate(local_qrot);
        ViewMatrix = cameraMatrix.inverted() * ViewMatrix;
        initialPos = false;
    }
    ProjectionMatrix.setToIdentity();
    ProjectionMatrix.perspective(initialFoV, 4.0f / 3.0f, 0.1f, 100.0f);        //initialFoV  - initial Field of view for a given window
                                                                                //aspect ratio - 4.0f / 3.0f
                                                                                //near range - 0.1, far range - 100
}
void OpenGLPCL::StopFrame()
{
    gotFrame = false;
}
void OpenGLPCL::mouseDoubleClickEvent(QMouseEvent *event)
{
    emit maximized((QWidget*)this);
}

void OpenGLPCL::mousePressEvent(QMouseEvent *event)
{
    lastMousePos = event->pos();                    //getting the mouse position to calculate direction
}

void OpenGLPCL::mouseMoveEvent(QMouseEvent *event){

    // Compute new orientation
    horizontalAngle += mouseSpeed * float( lastMousePos.x() - event->pos().x());
    verticalAngle   += mouseSpeed * float(lastMousePos.y() - event->pos().y());

    local_rot = {verticalAngle,horizontalAngle,0.0};
    local_qrot = local_qrot.fromEulerAngles(local_rot);

    ViewMatrix.setToIdentity();
    ViewMatrix.translate(local_pos);
    ViewMatrix.rotate(local_qrot);
    ViewMatrix = cameraMatrix.inverted() * ViewMatrix;
    lastMousePos = event->pos();
}

void OpenGLPCL::wheelEvent(QWheelEvent *event)
{
    if(event->delta()>0) {
        initialFoV -= 0.5f;                         //for zoom out have to decrease FoV
    }
    else if(event->delta()<=0) {
        initialFoV += 0.5f;                         //for zoom in have to increase FoV
    }

    if (initialFoV < 0.5f)              //maximum zoom
         initialFoV = 0.5f;
    else if(initialFoV > 177.5)         //minimum zoom
        initialFoV = 177.5;

}
void OpenGLPCL::get3DIntrinsic(double fx, double fy, double cx, double cy)
{
	if (fx != NAN && fy != NAN && cx != NAN && cx != NAN)
	{
		principlePointx = cx;
		principlePointy = cy;
		focalLengthx = fx;
		focalLengthy = fy;
	}

}


void OpenGLPCL::getColorMapProp(uint16_t depthMin, uint16_t depthMax, uint16_t colormap, uint16_t depthRange)
{
    if (colormap <= COLORMAP_LIMIT)
    {
        colorMap = colormap;
    }
    depth_min_val = depthMin;
    depth_max_val = depthMax;
    alpha = (255 / (double)(depthMax - depthMin));
    beta = ((double)alpha) * depthMin;
	currentDepthRange = depthRange;
}
void OpenGLPCL::setInitialPos()
{
    initialFoV = 15;
    initialPos = true;
}
