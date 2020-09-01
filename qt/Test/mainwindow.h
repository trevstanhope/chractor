#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QtCore/QFile>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void updateLocation();

    void restoreRecord();

    void on_pushButton_0_clicked();

    void on_pushButton_00_clicked();

    void on_pushButton_1_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void updateText();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

private:
    Ui::MainWindow *ui;
    QProcess *launch_process;
    QProcess *record_process;
    QProcess *pause_process;
    QProcess *sstop_process;
    QProcess *exam_process;
    QFile *rFile;
    QFile *record_file;
    QFile *tFile;
    QFile *lFile;
};

#endif // MAINWINDOW_H
