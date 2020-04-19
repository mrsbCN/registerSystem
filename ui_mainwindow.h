/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTabWidget *tabWidget;
    QWidget *tab_camera;
    QLabel *clolorframe;
    QLabel *irFrame;
    QLabel *depthFrame;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *startCameraButton;
    QPushButton *savePicButton;
    QPushButton *saveCloudButton;
    QWidget *tab_continue;
    QPushButton *beginButton;
    QWidget *continue_widget;
    QLabel *label_11;
    QWidget *tab_import;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_3;
    QFormLayout *formLayout;
    QLabel *label;
    QLineEdit *lineEdit;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QPushButton *selectCADFileButton;
    QFrame *line;
    QFrame *line_2;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_4;
    QFormLayout *formLayout_2;
    QLabel *label_5;
    QLineEdit *lineEdit_3;
    QLabel *label_6;
    QLineEdit *lineEdit_4;
    QLabel *label_7;
    QLineEdit *lineEdit_5;
    QPushButton *selectPointCloudFileButton;
    QFrame *line_3;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_8;
    QFormLayout *formLayout_3;
    QLabel *label_9;
    QLineEdit *lineEdit_6;
    QLineEdit *lineEdit_7;
    QLabel *label_10;
    QPushButton *selectPointCloudFileButton_2;
    QWidget *single_widge;
    QLabel *label_hit;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1024, 768);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 0, 1031, 768));
        tab_camera = new QWidget();
        tab_camera->setObjectName(QString::fromUtf8("tab_camera"));
        clolorframe = new QLabel(tab_camera);
        clolorframe->setObjectName(QString::fromUtf8("clolorframe"));
        clolorframe->setGeometry(QRect(20, 20, 480, 320));
        irFrame = new QLabel(tab_camera);
        irFrame->setObjectName(QString::fromUtf8("irFrame"));
        irFrame->setGeometry(QRect(20, 370, 480, 320));
        depthFrame = new QLabel(tab_camera);
        depthFrame->setObjectName(QString::fromUtf8("depthFrame"));
        depthFrame->setGeometry(QRect(512, 20, 480, 320));
        verticalLayoutWidget = new QWidget(tab_camera);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(650, 470, 160, 111));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        startCameraButton = new QPushButton(verticalLayoutWidget);
        startCameraButton->setObjectName(QString::fromUtf8("startCameraButton"));

        verticalLayout->addWidget(startCameraButton);

        savePicButton = new QPushButton(verticalLayoutWidget);
        savePicButton->setObjectName(QString::fromUtf8("savePicButton"));

        verticalLayout->addWidget(savePicButton);

        saveCloudButton = new QPushButton(verticalLayoutWidget);
        saveCloudButton->setObjectName(QString::fromUtf8("saveCloudButton"));

        verticalLayout->addWidget(saveCloudButton);

        tabWidget->addTab(tab_camera, QString());
        tab_continue = new QWidget();
        tab_continue->setObjectName(QString::fromUtf8("tab_continue"));
        beginButton = new QPushButton(tab_continue);
        beginButton->setObjectName(QString::fromUtf8("beginButton"));
        beginButton->setGeometry(QRect(870, 400, 121, 71));
        continue_widget = new QWidget(tab_continue);
        continue_widget->setObjectName(QString::fromUtf8("continue_widget"));
        continue_widget->setGeometry(QRect(30, 20, 800, 640));
        label_11 = new QLabel(tab_continue);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(890, 350, 81, 31));
        tabWidget->addTab(tab_continue, QString());
        tab_import = new QWidget();
        tab_import->setObjectName(QString::fromUtf8("tab_import"));
        verticalLayoutWidget_2 = new QWidget(tab_import);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(810, 20, 198, 171));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(verticalLayoutWidget_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setAutoFillBackground(false);
        label_3->setLocale(QLocale(QLocale::Chinese, QLocale::China));
        label_3->setAlignment(Qt::AlignCenter);
        label_3->setWordWrap(true);

        verticalLayout_2->addWidget(label_3);

        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label = new QLabel(verticalLayoutWidget_2);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        lineEdit = new QLineEdit(verticalLayoutWidget_2);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        formLayout->setWidget(0, QFormLayout::FieldRole, lineEdit);

        label_2 = new QLabel(verticalLayoutWidget_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        lineEdit_2 = new QLineEdit(verticalLayoutWidget_2);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

        formLayout->setWidget(1, QFormLayout::FieldRole, lineEdit_2);


        verticalLayout_2->addLayout(formLayout);

        selectCADFileButton = new QPushButton(verticalLayoutWidget_2);
        selectCADFileButton->setObjectName(QString::fromUtf8("selectCADFileButton"));

        verticalLayout_2->addWidget(selectCADFileButton);

        line = new QFrame(tab_import);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(800, 200, 221, 20));
        line->setLineWidth(2);
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(tab_import);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(800, 410, 220, 20));
        line_2->setLineWidth(2);
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        verticalLayoutWidget_3 = new QWidget(tab_import);
        verticalLayoutWidget_3->setObjectName(QString::fromUtf8("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(810, 230, 198, 171));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(verticalLayoutWidget_3);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setAutoFillBackground(false);
        label_4->setLocale(QLocale(QLocale::Chinese, QLocale::China));
        label_4->setAlignment(Qt::AlignCenter);
        label_4->setWordWrap(true);

        verticalLayout_3->addWidget(label_4);

        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        label_5 = new QLabel(verticalLayoutWidget_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_5);

        lineEdit_3 = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, lineEdit_3);

        label_6 = new QLabel(verticalLayoutWidget_3);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_6);

        lineEdit_4 = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, lineEdit_4);

        label_7 = new QLabel(verticalLayoutWidget_3);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, label_7);

        lineEdit_5 = new QLineEdit(verticalLayoutWidget_3);
        lineEdit_5->setObjectName(QString::fromUtf8("lineEdit_5"));

        formLayout_2->setWidget(2, QFormLayout::FieldRole, lineEdit_5);


        verticalLayout_3->addLayout(formLayout_2);

        selectPointCloudFileButton = new QPushButton(verticalLayoutWidget_3);
        selectPointCloudFileButton->setObjectName(QString::fromUtf8("selectPointCloudFileButton"));

        verticalLayout_3->addWidget(selectPointCloudFileButton);

        line_3 = new QFrame(tab_import);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(790, 0, 20, 720));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        verticalLayoutWidget_4 = new QWidget(tab_import);
        verticalLayoutWidget_4->setObjectName(QString::fromUtf8("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(810, 440, 198, 171));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_8 = new QLabel(verticalLayoutWidget_4);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setAutoFillBackground(false);
        label_8->setLocale(QLocale(QLocale::Chinese, QLocale::China));
        label_8->setAlignment(Qt::AlignCenter);
        label_8->setWordWrap(true);

        verticalLayout_4->addWidget(label_8);

        formLayout_3 = new QFormLayout();
        formLayout_3->setObjectName(QString::fromUtf8("formLayout_3"));
        label_9 = new QLabel(verticalLayoutWidget_4);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        formLayout_3->setWidget(0, QFormLayout::LabelRole, label_9);

        lineEdit_6 = new QLineEdit(verticalLayoutWidget_4);
        lineEdit_6->setObjectName(QString::fromUtf8("lineEdit_6"));

        formLayout_3->setWidget(0, QFormLayout::FieldRole, lineEdit_6);

        lineEdit_7 = new QLineEdit(verticalLayoutWidget_4);
        lineEdit_7->setObjectName(QString::fromUtf8("lineEdit_7"));

        formLayout_3->setWidget(1, QFormLayout::FieldRole, lineEdit_7);

        label_10 = new QLabel(verticalLayoutWidget_4);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setWordWrap(false);

        formLayout_3->setWidget(1, QFormLayout::LabelRole, label_10);


        verticalLayout_4->addLayout(formLayout_3);

        selectPointCloudFileButton_2 = new QPushButton(verticalLayoutWidget_4);
        selectPointCloudFileButton_2->setObjectName(QString::fromUtf8("selectPointCloudFileButton_2"));

        verticalLayout_4->addWidget(selectPointCloudFileButton_2);

        single_widge = new QWidget(tab_import);
        single_widge->setObjectName(QString::fromUtf8("single_widge"));
        single_widge->setGeometry(QRect(0, 30, 800, 640));
        label_hit = new QLabel(tab_import);
        label_hit->setObjectName(QString::fromUtf8("label_hit"));
        label_hit->setEnabled(true);
        label_hit->setGeometry(QRect(820, 640, 161, 31));
        tabWidget->addTab(tab_import, QString());
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1024, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        clolorframe->setText(QCoreApplication::translate("MainWindow", "clolorframe", nullptr));
        irFrame->setText(QCoreApplication::translate("MainWindow", "irFrame", nullptr));
        depthFrame->setText(QCoreApplication::translate("MainWindow", "depthFrame", nullptr));
        startCameraButton->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
        savePicButton->setText(QCoreApplication::translate("MainWindow", "savePic", nullptr));
        saveCloudButton->setText(QCoreApplication::translate("MainWindow", "saveCloud", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_camera), QCoreApplication::translate("MainWindow", "\346\267\261\345\272\246\347\233\270\346\234\272", nullptr));
        beginButton->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "\345\205\210\345\257\274\345\205\245\346\250\241\345\236\213 ", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_continue), QCoreApplication::translate("MainWindow", "\350\277\236\347\273\255\351\205\215\345\207\206 ", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "stl,obj,ply\346\240\274\345\274\217CAD\346\250\241\345\236\213\346\226\207\344\273\266\345\244\232\350\247\206\350\247\222\351\207\207\346\240\267", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "\345\210\206\350\276\250\347\216\207", nullptr));
        lineEdit->setText(QCoreApplication::translate("MainWindow", "100", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "\347\273\206\345\210\206\346\225\260", nullptr));
        lineEdit_2->setText(QCoreApplication::translate("MainWindow", "2", nullptr));
        selectCADFileButton->setText(QCoreApplication::translate("MainWindow", "\351\200\211\346\213\251\346\250\241\345\236\213\346\226\207\344\273\266\345\271\266\345\274\200\345\247\213\351\207\207\346\240\267", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "pcd,ply\347\202\271\344\272\221\347\224\237\346\210\220\347\202\271\345\257\271\347\211\271\345\276\201", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\351\207\207\346\240\267\346\240\205\346\240\274", nullptr));
        lineEdit_3->setText(QCoreApplication::translate("MainWindow", "0.03", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "\350\267\235\347\246\273\347\246\273\346\225\243\346\255\245\351\225\277", nullptr));
        lineEdit_4->setText(QCoreApplication::translate("MainWindow", "0.03", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "\350\247\222\345\272\246\347\246\273\346\225\243\346\255\245\351\225\277", nullptr));
        lineEdit_5->setText(QCoreApplication::translate("MainWindow", "30", nullptr));
        selectPointCloudFileButton->setText(QCoreApplication::translate("MainWindow", "\351\200\211\346\213\251\347\202\271\344\272\221\347\224\237\346\210\220PPF", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "\351\200\211\346\213\251\345\234\272\346\231\257\347\202\271\344\272\221\350\277\233\350\241\214\345\214\271\351\205\215", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "\347\202\271\344\272\221\351\207\207\346\240\267\346\240\205\346\240\274", nullptr));
        lineEdit_6->setText(QCoreApplication::translate("MainWindow", "0.05", nullptr));
        lineEdit_7->setText(QCoreApplication::translate("MainWindow", "0.3", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "\345\234\272\346\231\257\347\202\271\346\257\224\344\276\213", nullptr));
        selectPointCloudFileButton_2->setText(QCoreApplication::translate("MainWindow", "\351\200\211\346\213\251\347\202\271\344\272\221\350\277\233\350\241\214\345\214\271\351\205\215", nullptr));
        label_hit->setText(QCoreApplication::translate("MainWindow", "\350\257\267\345\205\210\347\202\271\345\207\273\345\267\246\344\276\247continue", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_import), QCoreApplication::translate("MainWindow", "\345\215\225\345\270\247\351\205\215\345\207\206\345\217\212\346\250\241\345\236\213\345\257\274\345\205\245", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
