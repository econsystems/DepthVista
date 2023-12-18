#ifndef OPENGLDISPLAY_H
#define OPENGLDISPLAY_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QScopedPointer>
#include <QException>
#include <QMutex>

typedef enum {
    UYVY = 1,
    Y16 = 2,
    RGB = 3,
}Pixelformat;


class OpenGLDisplay : public QOpenGLWidget, public QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit OpenGLDisplay(QWidget* parent = nullptr,Pixelformat pixformat = UYVY);
    ~OpenGLDisplay();
    QMutex renderMutex;
    bool gotFrame;
    Pixelformat pixelformat;
    void InitDrawBuffer(unsigned bsize);
    void DisplayVideoFrame(unsigned char *data, int frameWidth, int frameHeight);
    void StopFrame();
    void paintGL() override;


protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
signals:
    Q_SIGNAL void maximized(QWidget* widget);
    Q_SIGNAL void renderframe();
protected slots:
    Q_SLOT void updateFrame();
private:
    struct OpenGLDisplayImpl;
    QScopedPointer<OpenGLDisplayImpl> impl;
    int mPositionLoc;
    int mTexCoordLoc;
    GLint samplerLoc;
    GLuint TextureId;
    void shaderUYVY();
    void renderUYVY();
    void shaderY16();
    void renderY16();
    void shaderRGB();
    void renderRGB();
    unsigned char *srcBuffer;
    uint8_t *pfmb;
};

class OpenGlException: public QException
{
public:
     void raise() const { throw *this; }
     OpenGlException *clone() const { return new OpenGlException(*this); }
};

#endif // OPENGLDISPLAY_H
