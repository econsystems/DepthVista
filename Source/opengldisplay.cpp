#include "opengldisplay.h"

#include <QOpenGLShader>
#include <QOpenGLTexture>
#include <QCoreApplication>
#include <QResizeEvent>

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_TEXCOORD_ATTRIBUTE 1

#define ATTRIB_VERTEX 0
#define ATTRIB_TEXTURE 1

static GLfloat mVerticesDataPosition[] = {
    -1.f, 1.f, 0.0f, // Position 0
    -1.f, -1.f, 0.0f, // Position 1
    1.f, -1.f, 0.0f, // Position 2
    1.f, 1.f, 0.0f, // Position 3
};

static GLfloat mVerticesDataTextCord[] = {
    0.0f, 0.0f, // TexCoord 0
    0.0f, 1.0f, // TexCoord 1
    1.0f, 1.0f, // TexCoord 2
    1.0f, 0.0f // TexCoord 3
};
static unsigned short mIndicesData[] = { 0, 1, 2, 0, 2, 3 };

struct OpenGLDisplay::OpenGLDisplayImpl
{
    OpenGLDisplayImpl()
        :mFrameSize(0)
    {}

    int                     mFrameSize;

    QOpenGLShader*          mVShader;
    QOpenGLShader*          mFShader;
    QOpenGLShaderProgram*   mShaderProgram;
    GLsizei                 mVideoW, mVideoH;

};

OpenGLDisplay::OpenGLDisplay(QWidget* parent , Pixelformat pixformat)
    : QOpenGLWidget(parent)
    , pixelformat(pixformat)
    , impl(new OpenGLDisplayImpl)
{
    srcBuffer = NULL;
    connect(this,SIGNAL(maximized(QWidget*)),parent,SLOT(onwidgetMaximized(QWidget*)));
    connect(this,SIGNAL(renderframe()),this, SLOT(updateFrame()));

}
void OpenGLDisplay::updateFrame()
{
    update();
}

OpenGLDisplay::~OpenGLDisplay()
{
    if(pixelformat==Pixelformat::Y16) {
        if(srcBuffer) {
            free(srcBuffer); srcBuffer = NULL;
        }
    }
}

void OpenGLDisplay::InitDrawBuffer(unsigned bsize)
{
    renderMutex.lock();
    gotFrame = false;
    impl->mFrameSize = bsize;
    if(pixelformat==Pixelformat::Y16){
        if(srcBuffer) {
            free(srcBuffer); srcBuffer = NULL;
        }
        srcBuffer = (unsigned char *)malloc(bsize);
    }
    renderMutex.unlock();
}

void OpenGLDisplay::DisplayVideoFrame(unsigned char *data, int frameWidth, int frameHeight)
{
    renderMutex.lock();
    impl->mVideoW = frameWidth;
    impl->mVideoH = frameHeight;

    switch (pixelformat) {
    case UYVY:
        srcBuffer = data;
        break;
    case Y16:
        pfmb = (uint8_t *)srcBuffer;
        if(pfmb){
            for(uint32_t l=0; l<(frameWidth * frameHeight*2); l=l+2) {
                *pfmb++ = data[l+1];
                *pfmb++ = 0x80;
            }
        }

        break;
    case RGB:
        srcBuffer = data;
        break;
    }
    gotFrame = true;
    renderMutex.unlock();
    emit renderframe();
}
void OpenGLDisplay::shaderUYVY()
{
    /* Modern opengl rendering pipeline relies on shaders to handle incoming data.
     *  Shader: is a small function written in OpenGL Shading Language (GLSL).
     * GLSL is the language that makes up all OpenGL shaders.
     * The syntax of the specific GLSL language requires the reader to find relevant information. */

    // Initialize the vertex shader object
    impl->mVShader = new QOpenGLShader(QOpenGLShader::Vertex, this);

    //Vertex shader source
    const char *vsrc = "attribute vec4 vertexIn; \
        attribute vec2 textureIn; \
        varying vec2 textureOut;  \
        void main(void)           \
        {                         \
            gl_Position = vertexIn; \
            textureOut = textureIn; \
        }";

    //Compile the vertex shader program
    bool bCompile = impl->mVShader->compileSourceCode(vsrc);
    if(!bCompile)
    {
        throw OpenGlException();
    }

    // Initialize the fragment shader function yuv converted to rgb
    impl->mFShader = new QOpenGLShader(QOpenGLShader::Fragment, this);

    // Fragment shader source code
    const char *fsrc = "#ifdef GL_ES\n"
                     "precision highp float;\n"
                     "#endif\n"

                     "varying vec2 textureOut;\n"
                     "uniform sampler2D uyvy_texture;\n"

                     "uniform float texel_width;"
                     "uniform float texture_width;"
                     "uniform float texture_height;"


                     "void main()\n"
                     "{\n"
                     "float r, g, b, y, u, v;\n"

                     "   vec4 luma_chroma;\n"
                     "   float xcoord = floor(textureOut.x * texture_width);\n"
                     "   float ycoord = floor(textureOut.y * texture_height);\n"

                     //We had put the Y values of each pixel to the R,G,B components by
                     //GL_LUMINANCE, that's why we're pulling it from the R component,
                     //we can also use G or B

                     "if (mod(xcoord, 2.0) == 0.0) {\n"
                     "   luma_chroma = texture2D(uyvy_texture, textureOut);\n"
                     "   y = luma_chroma.g;\n"
                     "} else {\n"
                     "   luma_chroma = texture2D(uyvy_texture, vec2(textureOut.x - texel_width, textureOut.y));\n"
                     "   y = luma_chroma.a;\n"
                     "}\n"
                     "u = luma_chroma.r - 0.5;\n"
                     "v = luma_chroma.b - 0.5;\n"

                     //We had put the U and V values of each pixel to the A and R,G,B
                     //components of the texture respectively using GL_LUMINANCE_ALPHA.
                     //Since U,V bytes are interspread in the texture, this is probably
                     //the fastest way to use them in the shader

                     //The numbers are just YUV to RGB conversion constants
                     "r = y + 1.13983*v;\n"
                     "g = y - 0.39465*u - 0.58060*v;\n"
                     "b = y + 2.03211*u;\n"

                     "gl_FragColor = vec4(r,g,b,1.0);\n"
                     "}\n";

    bCompile = impl->mFShader->compileSourceCode(fsrc);
    if(!bCompile)
    {
        throw OpenGlException();
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
    //activate all links

    mPositionLoc = impl->mShaderProgram->attributeLocation("vertexIn");
    mTexCoordLoc = impl->mShaderProgram->attributeLocation("textureIn");

    // Get the sampler location
    samplerLoc = impl->mShaderProgram->uniformLocation("uyvy_texture");
    glEnable(GL_TEXTURE_2D);
    // Generate a texture object
    glGenTextures (1, &TextureId);
    glActiveTexture (GL_TEXTURE1);

    // Bind the texture object
    glBindTexture (GL_TEXTURE_2D, TextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    impl->mShaderProgram->bind();

    glVertexAttribPointer(mPositionLoc, 3, GL_FLOAT, false, 12, mVerticesDataPosition);
    glVertexAttribPointer(mTexCoordLoc, 2, GL_FLOAT, false, 8, mVerticesDataTextCord);
    
    // set active texture and give input y buffer
    impl->mShaderProgram->enableAttributeArray(0);
    impl->mShaderProgram->enableAttributeArray(1);
}

void OpenGLDisplay::renderUYVY()
{
    glActiveTexture(GL_TEXTURE1);
    glBindTexture (GL_TEXTURE_2D, TextureId);
    glUniform1i(samplerLoc, 1);
    renderMutex.lock();
    if(gotFrame){

        if (srcBuffer != NULL){
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, impl->mVideoW/2, impl->mVideoH, 0, GL_RGBA, GL_UNSIGNED_BYTE, (unsigned char*)srcBuffer);
        }
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, mIndicesData);
    }
    renderMutex.unlock();
}

void OpenGLDisplay::shaderY16()
{
    /* Modern opengl rendering pipeline relies on shaders to handle incoming data.
     *  Shader: is a small function written in OpenGL Shading Language (GLSL).
     * GLSL is the language that makes up all OpenGL shaders.
     * The syntax of the specific GLSL language requires the reader to find relevant information. */

    impl->mVShader = new QOpenGLShader(QOpenGLShader::Vertex, this);

    //Vertex shader source
    const char *vsrc = "attribute vec4 a_position;\n"
                       "attribute vec2 a_texCoord;\n"
                       "varying vec2 v_texCoord;\n"
                       "void main()\n"
                       "{\n"
                       "gl_Position = a_position;\n"
                       "v_texCoord = a_texCoord;\n"
                       "}\n";

    //Compile the vertex shader program
    bool bCompile = impl->mVShader->compileSourceCode(vsrc);
    if(!bCompile)
    {
        throw OpenGlException();
    }

    // Initialize the fragment shader function yuv converted to rgb
    impl->mFShader = new QOpenGLShader(QOpenGLShader::Fragment, this);

    // Fragment shader source code


    const char *fsrc = "#ifdef GL_ES\n"
                       "precision highp float;\n"
                       "#endif\n"

                       "varying vec2 v_texCoord;\n"
                       "uniform sampler2D yuyv_texture;\n"

                       "uniform float texel_width;"
                       "uniform float texture_width;"
                       "uniform float texture_height;"


                       "void main()\n"
                       "{\n"
                       "float r, g, b, y, u, v;\n"

                       "   vec4 luma_chroma;\n"
                       "   float xcoord = floor(v_texCoord.x * texture_width);\n"
                       "   float ycoord = floor(v_texCoord.y * texture_height);\n"

                       //We had put the Y values of each pixel to the R,G,B components by
                       //GL_LUMINANCE, that's why we're pulling it from the R component,
                       //we can also use G or B

                       "if (mod(xcoord, 2.0) == 0.0) {\n"
                       "   luma_chroma = texture2D(yuyv_texture, v_texCoord);\n"
                       "   y = luma_chroma.r;\n"
                       "} else {\n"
                       "   luma_chroma = texture2D(yuyv_texture, vec2(v_texCoord.x - texel_width, v_texCoord.y));\n"
                       "   y = luma_chroma.b;\n"
                       "}\n"
                       "u = luma_chroma.g - 0.5;\n"
                       "v = luma_chroma.a - 0.5;\n"

                       //We had put the U and V values of each pixel to the A and R,G,B
                       //components of the texture respectively using GL_LUMINANCE_ALPHA.
                       //Since U,V bytes are interspread in the texture, this is probably
                       //the fastest way to use them in the shader

                       //The numbers are just YUV to RGB conversion constants
                       "r = y + 1.13983*v;\n"
                       "g = y - 0.39465*u - 0.58060*v;\n"
                       "b = y + 2.03211*u;\n"

                       "gl_FragColor = vec4(r,g,b,1.0);\n"
                       "}\n";

    bCompile = impl->mFShader->compileSourceCode(fsrc);
    if(!bCompile)
    {
        throw OpenGlException();
    }

    // Create a shader program container
    impl->mShaderProgram = new QOpenGLShaderProgram(this);
    // Add the fragment shader to the program container
    impl->mShaderProgram->addShader(impl->mFShader);
    // Add a vertex shader to the program container
    impl->mShaderProgram->addShader(impl->mVShader);

    impl->mShaderProgram->bindAttributeLocation("a_position", 0);
    impl->mShaderProgram->bindAttributeLocation("a_texCoord", 1);
    impl->mShaderProgram->link();

    mPositionLoc = impl->mShaderProgram->attributeLocation("a_position");
    mTexCoordLoc = impl->mShaderProgram->attributeLocation("a_texCoord");


    // Get the sampler location
    samplerLoc = impl->mShaderProgram->uniformLocation("yuyv_texture");
    glEnable(GL_TEXTURE_2D);
    // Generate a texture object
    glGenTextures (1, &TextureId);
    glActiveTexture (GL_TEXTURE2);

    // Bind the texture object
    glBindTexture (GL_TEXTURE_2D, TextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    impl->mShaderProgram->bind();
    glVertexAttribPointer(mPositionLoc, 3, GL_FLOAT, false, 12, mVerticesDataPosition);
    glVertexAttribPointer(mTexCoordLoc, 2, GL_FLOAT, false, 8, mVerticesDataTextCord);

    impl->mShaderProgram->enableAttributeArray(0);
    impl->mShaderProgram->enableAttributeArray(1);

}

void OpenGLDisplay::renderY16()
{
    // set active texture and give input y buffer
    glActiveTexture(GL_TEXTURE2);
    glBindTexture (GL_TEXTURE_2D, TextureId);
    glUniform1i(samplerLoc, 2);

    renderMutex.lock();
    if(gotFrame){
        if (srcBuffer != NULL){
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, impl->mVideoW/2, impl->mVideoH, 0, GL_RGBA, GL_UNSIGNED_BYTE, (unsigned char*)srcBuffer);
        }
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, mIndicesData);
    }
    renderMutex.unlock();
}
void OpenGLDisplay::shaderRGB()
{
    /* Modern opengl rendering pipeline relies on shaders to handle incoming data.
     *  Shader: is a small function written in OpenGL Shading Language (GLSL).
     * GLSL is the language that makes up all OpenGL shaders.
     * The syntax of the specific GLSL language requires the reader to find relevant information. */

    // Initialize the vertex shader object
    impl->mVShader = new QOpenGLShader(QOpenGLShader::Vertex, this);

    //Vertex shader source
    const char *vsrc = "attribute vec4 vertexIn; \
        attribute vec2 textureIn; \
        varying vec2 textureOut;  \
        void main(void)           \
        {                         \
            gl_Position = vertexIn; \
            textureOut = textureIn; \
        }";

    //Compile the vertex shader program
    bool bCompile = impl->mVShader->compileSourceCode(vsrc);
    if(!bCompile)
    {
        throw OpenGlException();
    }

    // Initialize the fragment shader function yuv converted to rgb
    impl->mFShader = new QOpenGLShader(QOpenGLShader::Fragment, this);

    // Fragment shader source code
    const char *fsrc = "#ifdef GL_ES\n"
                     "precision highp float;\n"
                     "#endif\n"

                     "varying vec2 textureOut;\n"
                     "uniform sampler2D uyvy_texture;\n"

                     "uniform float texel_width;"
                     "uniform float texture_width;"
                     "uniform float texture_height;"


                     "void main()\n"
                     "{\n"
                       "   float ycoord = floor(textureOut.y * texture_height);"
                       "   vec4 color = texture2D(uyvy_texture, textureOut);"
                       "   float a = color.a;"
                       "   float r = color.b;"
                       "   float g = color.g;"
                       "   float b = color.r;"
                       "   gl_FragColor = vec4(r, g, b, a);"
                       "}";


    bCompile = impl->mFShader->compileSourceCode(fsrc);
    if(!bCompile) {
        throw OpenGlException();
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

    // Get the sampler location
    samplerLoc = impl->mShaderProgram->uniformLocation("uyvy_texture");
    glEnable(GL_TEXTURE_2D);

    // Generate a texture object
    glGenTextures (1, &TextureId);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, TextureId);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    impl->mShaderProgram->bind();
    glVertexAttribPointer(mPositionLoc, 3, GL_FLOAT, false, 12, mVerticesDataPosition);
    glVertexAttribPointer(mTexCoordLoc, 2, GL_FLOAT, false, 8, mVerticesDataTextCord);

    impl->mShaderProgram->enableAttributeArray(0);
    impl->mShaderProgram->enableAttributeArray(1);
}

void OpenGLDisplay::renderRGB()
{
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, TextureId);
    glUniform1i(samplerLoc, 0);

    renderMutex.lock();
    if(gotFrame) {

        if (srcBuffer != NULL) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, impl->mVideoW, impl->mVideoH, 0, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)srcBuffer);
        }
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, mIndicesData);
    }
    renderMutex.unlock();
}

void OpenGLDisplay::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    switch (pixelformat) {
    case UYVY:
        shaderUYVY();
        break;
    case Y16:
        shaderY16();
        break;
    case RGB:
        shaderRGB();
        break;
    }
    glClearColor (0, 0, 0, 0); // set the background color
}

void OpenGLDisplay::resizeGL(int w, int h)
{
    if(h == 0)// prevents being divided by zero
        h = 1;// set the height to 1

    // Set the viewport
    glViewport(0, 0, w, h);
    update();
}

void OpenGLDisplay::paintGL()
{

    if(gotFrame) {

        if(impl->mShaderProgram) {

            switch (pixelformat) {

            case UYVY:
                renderUYVY();
                break;
            case Y16:

                renderY16();
                break;
            case RGB:
                renderRGB();
                break;
            }
        }

    }
}
void OpenGLDisplay::StopFrame()
{
    gotFrame = false;
}
void OpenGLDisplay::mouseDoubleClickEvent(QMouseEvent *event)
{
    emit maximized((QWidget*)this);
}
