/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *joint1Left;
    QPushButton *joint1Right;
    QTextBrowser *textBrowser;
    QTextBrowser *textBrowser_2;
    QPushButton *joint2Left;
    QPushButton *joint2Right;
    QPushButton *joint3Right;
    QTextBrowser *textBrowser_3;
    QPushButton *joint3Left;
    QPushButton *joint6Left;
    QTextBrowser *textBrowser_4;
    QPushButton *joint6Right;
    QPushButton *joint4Left;
    QTextBrowser *textBrowser_5;
    QTextBrowser *textBrowser_6;
    QPushButton *joint5Right;
    QPushButton *joint5Left;
    QPushButton *joint4Right;
    QSlider *joint1Slider;
    QSlider *joint2Slider;
    QSlider *joint3Slider;
    QSlider *joint4Slider;
    QSlider *joint5Slider;
    QSlider *joint6Slider;
    QTextBrowser *lb_joint1;
    QTextBrowser *lb_joint2;
    QTextBrowser *lb_joint3;
    QTextBrowser *lb_joint4;
    QTextBrowser *lb_joint5;
    QTextBrowser *lb_joint6;
    QPushButton *connect_button;
    QPushButton *disconnect_button;
    QLineEdit *lineEdit;
    QTextBrowser *textBrowser_7;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(491, 580);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        joint1Left = new QPushButton(centralwidget);
        joint1Left->setObjectName(QStringLiteral("joint1Left"));
        joint1Left->setGeometry(QRect(340, 70, 51, 51));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(joint1Left->sizePolicy().hasHeightForWidth());
        joint1Left->setSizePolicy(sizePolicy);
        QFont font;
        font.setPointSize(30);
        joint1Left->setFont(font);
        joint1Left->setAutoRepeat(false);
        joint1Right = new QPushButton(centralwidget);
        joint1Right->setObjectName(QStringLiteral("joint1Right"));
        joint1Right->setGeometry(QRect(410, 70, 51, 51));
        sizePolicy.setHeightForWidth(joint1Right->sizePolicy().hasHeightForWidth());
        joint1Right->setSizePolicy(sizePolicy);
        QFont font1;
        font1.setPointSize(20);
        joint1Right->setFont(font1);
        textBrowser = new QTextBrowser(centralwidget);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(30, 80, 91, 31));
        textBrowser_2 = new QTextBrowser(centralwidget);
        textBrowser_2->setObjectName(QStringLiteral("textBrowser_2"));
        textBrowser_2->setGeometry(QRect(30, 150, 91, 31));
        joint2Left = new QPushButton(centralwidget);
        joint2Left->setObjectName(QStringLiteral("joint2Left"));
        joint2Left->setGeometry(QRect(340, 140, 51, 51));
        sizePolicy.setHeightForWidth(joint2Left->sizePolicy().hasHeightForWidth());
        joint2Left->setSizePolicy(sizePolicy);
        joint2Left->setFont(font);
        joint2Left->setAutoRepeat(false);
        joint2Right = new QPushButton(centralwidget);
        joint2Right->setObjectName(QStringLiteral("joint2Right"));
        joint2Right->setGeometry(QRect(410, 140, 51, 51));
        sizePolicy.setHeightForWidth(joint2Right->sizePolicy().hasHeightForWidth());
        joint2Right->setSizePolicy(sizePolicy);
        joint2Right->setFont(font1);
        joint3Right = new QPushButton(centralwidget);
        joint3Right->setObjectName(QStringLiteral("joint3Right"));
        joint3Right->setGeometry(QRect(410, 210, 51, 51));
        sizePolicy.setHeightForWidth(joint3Right->sizePolicy().hasHeightForWidth());
        joint3Right->setSizePolicy(sizePolicy);
        joint3Right->setFont(font1);
        textBrowser_3 = new QTextBrowser(centralwidget);
        textBrowser_3->setObjectName(QStringLiteral("textBrowser_3"));
        textBrowser_3->setGeometry(QRect(30, 220, 91, 31));
        joint3Left = new QPushButton(centralwidget);
        joint3Left->setObjectName(QStringLiteral("joint3Left"));
        joint3Left->setGeometry(QRect(340, 210, 51, 51));
        sizePolicy.setHeightForWidth(joint3Left->sizePolicy().hasHeightForWidth());
        joint3Left->setSizePolicy(sizePolicy);
        joint3Left->setFont(font);
        joint3Left->setAutoRepeat(false);
        joint6Left = new QPushButton(centralwidget);
        joint6Left->setObjectName(QStringLiteral("joint6Left"));
        joint6Left->setGeometry(QRect(340, 420, 51, 51));
        sizePolicy.setHeightForWidth(joint6Left->sizePolicy().hasHeightForWidth());
        joint6Left->setSizePolicy(sizePolicy);
        joint6Left->setFont(font);
        joint6Left->setAutoRepeat(false);
        textBrowser_4 = new QTextBrowser(centralwidget);
        textBrowser_4->setObjectName(QStringLiteral("textBrowser_4"));
        textBrowser_4->setGeometry(QRect(30, 360, 91, 31));
        joint6Right = new QPushButton(centralwidget);
        joint6Right->setObjectName(QStringLiteral("joint6Right"));
        joint6Right->setGeometry(QRect(410, 420, 51, 51));
        sizePolicy.setHeightForWidth(joint6Right->sizePolicy().hasHeightForWidth());
        joint6Right->setSizePolicy(sizePolicy);
        joint6Right->setFont(font1);
        joint4Left = new QPushButton(centralwidget);
        joint4Left->setObjectName(QStringLiteral("joint4Left"));
        joint4Left->setGeometry(QRect(340, 280, 51, 51));
        sizePolicy.setHeightForWidth(joint4Left->sizePolicy().hasHeightForWidth());
        joint4Left->setSizePolicy(sizePolicy);
        joint4Left->setFont(font);
        joint4Left->setAutoRepeat(false);
        textBrowser_5 = new QTextBrowser(centralwidget);
        textBrowser_5->setObjectName(QStringLiteral("textBrowser_5"));
        textBrowser_5->setGeometry(QRect(30, 430, 91, 31));
        textBrowser_6 = new QTextBrowser(centralwidget);
        textBrowser_6->setObjectName(QStringLiteral("textBrowser_6"));
        textBrowser_6->setGeometry(QRect(30, 290, 91, 31));
        joint5Right = new QPushButton(centralwidget);
        joint5Right->setObjectName(QStringLiteral("joint5Right"));
        joint5Right->setGeometry(QRect(410, 350, 51, 51));
        sizePolicy.setHeightForWidth(joint5Right->sizePolicy().hasHeightForWidth());
        joint5Right->setSizePolicy(sizePolicy);
        joint5Right->setFont(font1);
        joint5Left = new QPushButton(centralwidget);
        joint5Left->setObjectName(QStringLiteral("joint5Left"));
        joint5Left->setGeometry(QRect(340, 350, 51, 51));
        sizePolicy.setHeightForWidth(joint5Left->sizePolicy().hasHeightForWidth());
        joint5Left->setSizePolicy(sizePolicy);
        joint5Left->setFont(font);
        joint5Left->setAutoRepeat(false);
        joint4Right = new QPushButton(centralwidget);
        joint4Right->setObjectName(QStringLiteral("joint4Right"));
        joint4Right->setGeometry(QRect(410, 280, 51, 51));
        sizePolicy.setHeightForWidth(joint4Right->sizePolicy().hasHeightForWidth());
        joint4Right->setSizePolicy(sizePolicy);
        joint4Right->setFont(font1);
        joint1Slider = new QSlider(centralwidget);
        joint1Slider->setObjectName(QStringLiteral("joint1Slider"));
        joint1Slider->setGeometry(QRect(140, 70, 181, 21));
        joint1Slider->setTracking(true);
        joint1Slider->setOrientation(Qt::Horizontal);
        joint2Slider = new QSlider(centralwidget);
        joint2Slider->setObjectName(QStringLiteral("joint2Slider"));
        joint2Slider->setGeometry(QRect(140, 140, 181, 21));
        joint2Slider->setOrientation(Qt::Horizontal);
        joint3Slider = new QSlider(centralwidget);
        joint3Slider->setObjectName(QStringLiteral("joint3Slider"));
        joint3Slider->setGeometry(QRect(140, 210, 181, 21));
        joint3Slider->setOrientation(Qt::Horizontal);
        joint4Slider = new QSlider(centralwidget);
        joint4Slider->setObjectName(QStringLiteral("joint4Slider"));
        joint4Slider->setGeometry(QRect(140, 280, 181, 21));
        joint4Slider->setOrientation(Qt::Horizontal);
        joint5Slider = new QSlider(centralwidget);
        joint5Slider->setObjectName(QStringLiteral("joint5Slider"));
        joint5Slider->setGeometry(QRect(140, 350, 181, 21));
        joint5Slider->setOrientation(Qt::Horizontal);
        joint6Slider = new QSlider(centralwidget);
        joint6Slider->setObjectName(QStringLiteral("joint6Slider"));
        joint6Slider->setGeometry(QRect(140, 420, 181, 21));
        joint6Slider->setOrientation(Qt::Horizontal);
        lb_joint1 = new QTextBrowser(centralwidget);
        lb_joint1->setObjectName(QStringLiteral("lb_joint1"));
        lb_joint1->setGeometry(QRect(140, 90, 181, 31));
        lb_joint1->viewport()->setProperty("cursor", QVariant(QCursor(Qt::ArrowCursor)));
        lb_joint1->setAutoFillBackground(false);
        lb_joint2 = new QTextBrowser(centralwidget);
        lb_joint2->setObjectName(QStringLiteral("lb_joint2"));
        lb_joint2->setGeometry(QRect(140, 160, 181, 31));
        lb_joint3 = new QTextBrowser(centralwidget);
        lb_joint3->setObjectName(QStringLiteral("lb_joint3"));
        lb_joint3->setGeometry(QRect(140, 230, 181, 31));
        lb_joint4 = new QTextBrowser(centralwidget);
        lb_joint4->setObjectName(QStringLiteral("lb_joint4"));
        lb_joint4->setGeometry(QRect(140, 300, 181, 31));
        lb_joint5 = new QTextBrowser(centralwidget);
        lb_joint5->setObjectName(QStringLiteral("lb_joint5"));
        lb_joint5->setGeometry(QRect(140, 370, 181, 31));
        lb_joint6 = new QTextBrowser(centralwidget);
        lb_joint6->setObjectName(QStringLiteral("lb_joint6"));
        lb_joint6->setGeometry(QRect(140, 440, 181, 31));
        connect_button = new QPushButton(centralwidget);
        connect_button->setObjectName(QStringLiteral("connect_button"));
        connect_button->setGeometry(QRect(80, 490, 111, 31));
        disconnect_button = new QPushButton(centralwidget);
        disconnect_button->setObjectName(QStringLiteral("disconnect_button"));
        disconnect_button->setGeometry(QRect(270, 490, 111, 31));
        lineEdit = new QLineEdit(centralwidget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setGeometry(QRect(250, 10, 101, 22));
        textBrowser_7 = new QTextBrowser(centralwidget);
        textBrowser_7->setObjectName(QStringLiteral("textBrowser_7"));
        textBrowser_7->setGeometry(QRect(110, 10, 111, 31));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 491, 19));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        joint1Left->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        joint1Right->setText(QApplication::translate("MainWindow", "+", Q_NULLPTR));
        textBrowser->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600;\">Joint 1</span></p></body></html>", Q_NULLPTR));
        textBrowser_2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600;\">Joint 2</span></p></body></html>", Q_NULLPTR));
        joint2Left->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        joint2Right->setText(QApplication::translate("MainWindow", "+", Q_NULLPTR));
        joint3Right->setText(QApplication::translate("MainWindow", "+", Q_NULLPTR));
        textBrowser_3->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600;\">Joint 3</span></p></body></html>", Q_NULLPTR));
        joint3Left->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        joint6Left->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        textBrowser_4->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600;\">Joint 5</span></p></body></html>", Q_NULLPTR));
        joint6Right->setText(QApplication::translate("MainWindow", "+", Q_NULLPTR));
        joint4Left->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        textBrowser_5->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600;\">Joint 6</span></p></body></html>", Q_NULLPTR));
        textBrowser_6->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt; font-weight:600;\">Joint 4</span></p></body></html>", Q_NULLPTR));
        joint5Right->setText(QApplication::translate("MainWindow", "+", Q_NULLPTR));
        joint5Left->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        joint4Right->setText(QApplication::translate("MainWindow", "+", Q_NULLPTR));
#ifndef QT_NO_STATUSTIP
        joint1Slider->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        lb_joint1->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", Q_NULLPTR));
        connect_button->setText(QApplication::translate("MainWindow", "Connect", Q_NULLPTR));
        disconnect_button->setText(QApplication::translate("MainWindow", "Disconnect", Q_NULLPTR));
        lineEdit->setText(QApplication::translate("MainWindow", "192.168.3.100", Q_NULLPTR));
        textBrowser_7->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:11pt; font-weight:600;\">Robot IP</span></p></body></html>", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
