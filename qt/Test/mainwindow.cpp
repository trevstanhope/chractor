#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QString>
#include <QStringList>
#include <QCheckBox>
#include <QComboBox>
#include <QProcess>
#include <QtCore/QTextStream>
#include <QtCore/QFile>
#include <QtCore/QIODevice>
#include <QDebug>
#include <QDateTime>
#include <QMessageBox>

QString record_folder = "";
QString record_str;
QList<QComboBox*> combobox_list;
QList<QCheckBox*> checkbox_list;
bool launch_flag = FALSE;
bool record_flag = FALSE;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->textEdit_show->setReadOnly(true);

    combobox_list << ui->comboBox_1 << ui->comboBox_2 << ui->comboBox_3 << ui->comboBox_4 << ui->comboBox_5 << ui->comboBox_6 << ui->comboBox_7 << ui->comboBox_8
                  << ui->comboBox_9 << ui->comboBox_10;

    record_str = "2018, 2019, 2020, 2021, 2022, 2023, 2024, 2025";
    QStringList record_date_year = record_str.split(", ");
    ui->comboBox_1->addItem("    ");
    for (int i=0;i<record_date_year.size();i++) {
        ui->comboBox_1->addItem(record_date_year.value(i));
    }
    record_str = "01, 02, 03, 04, 05, 06, 07, 08, 09, 10, 11, 12";
    QStringList record_date_month = record_str.split(", ");
    ui->comboBox_2->addItem("    ");
    for (int i=0;i<record_date_month.size();i++) {
        ui->comboBox_2->addItem(record_date_month.value(i));
    }
    record_str = "01, 02, 03, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31";
    QStringList record_date_day = record_str.split(", ");
    ui->comboBox_3->addItem("    ");
    for (int i=0;i<record_date_day.size();i++) {
        ui->comboBox_3->addItem(record_date_day.value(i));
    }
    record_str = "12:00AM, 01:00AM, 02:00AM, 03:00AM, 04:00AM, 05:00AM, 06:00AM, 07:00AM, 08:00AM, 09:00AM, 10:00AM, 11:00AM, "
                 "12:00PM, 01:00PM, 02:00PM, 03:00PM, 04:00PM, 05:00PM, 06:00PM, 07:00PM, 08:00PM, 09:00PM, 10:00PM, 11:00PM";
    QStringList record_date_time = record_str.split(", ");
    ui->comboBox_4->addItem("    ");
    for (int i=0;i<record_date_time.size();i++) {
        ui->comboBox_4->addItem(record_date_time.value(i));
    }
    record_str = "USA, Canada, Brazil, Australia, New Zealand, Austria, Benelux, France, Germany, Italy, United Kingdom, Russia, Spain, Other";
    QStringList record_location_country = record_str.split(", ");
    ui->comboBox_5->addItem("    ");
    for (int i=0;i<record_location_country.size();i++) {
        ui->comboBox_5->addItem(record_location_country.value(i));
    }
    record_str = "4WD Tractor, LF CCH Tractor, SF CCH Tractor, CCM HD Tractor, CCM LWB Tractor, CCM SWB Tractor, APH Tractor, APL Tractor, Utility Tractor, Specialty Tractor (Orchard/Vineyard), "
                 "Turkey Tractor, Economy Tractor, Mexico Tractor, Compact Tractor, Crawler Tractor, FIH Combine Harvester, FNH Combine Harvester, MRR Combine Harvester, MRC Combine Harvester, Sugar Cane Harvester, "
                 "Coffee Harvester, Cotton Harvester, Grape Harvester, Header Platform, Self Propelled Forage Harvester, Pull-Type Forage Harvester, Large Square Baler, Small Square Baler, Variable Chamber Round Baler, Fixed Chamber Round Baler, "
                 "Self Propelled Windrower, Disc Mower Conditioner, Sicklebar Mower Conditioner, Self Propelled Bale Wagon, Planter Platform, Seeder Platform, Sprayer Platform, Tillage Platform, Telehandler Platform, Skid Steer Loader/Compact Track Loader, "
                 "Other";
    QStringList record_vehicle_model = record_str.split(", ");
    ui->comboBox_7->addItem("    ");
    for (int i=0;i<record_vehicle_model.size();i++) {
        ui->comboBox_7->addItem(record_vehicle_model.value(i));
    }
    record_str = "Case IH, New Holland";
    QStringList record_vehicle_brand = record_str.split(", ");
    ui->comboBox_8->addItem("    ");
    for (int i=0;i<record_vehicle_brand.size();i++) {
        ui->comboBox_8->addItem(record_vehicle_brand.value(i));
    }
    record_str = "Tillage, Planting, Seeding, Spraying, Irrigating, Mowing, Baling, Harvesting, Hauling, Other";
    QStringList record_implement = record_str.split(", ");
    ui->comboBox_9->addItem("    ");
    for (int i=0;i<record_implement.size();i++) {
        ui->comboBox_9->addItem(record_implement.value(i));
    }
    record_str = "Wheat, Rice, Barley/Rye, Corn, Soybean, Triticale, Canola/Rape, Suger Beet, Alfalfa/Lucerne, Pea, Grass, Cotton, Other";
    QStringList record_crop = record_str.split(", ");
    ui->comboBox_10->addItem("    ");
    for (int i=0;i<record_crop.size();i++) {
        ui->comboBox_10->addItem(record_crop.value(i));
    }

    checkbox_list << ui->checkBox_1 << ui->checkBox_2 << ui->checkBox_3 << ui->checkBox_4 << ui->checkBox_5 << ui->checkBox_6 << ui->checkBox_7 << ui->checkBox_8
                  << ui->checkBox_9 << ui->checkBox_10 << ui->checkBox_11 << ui->checkBox_12 << ui->checkBox_13 << ui->checkBox_14;

    restoreRecord();

    connect(ui->comboBox_5, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(updateLocation()));

    launch_process = new QProcess(this);
    record_process = new QProcess(this);
    pause_process = new QProcess(this);
    sstop_process = new QProcess(this);
    exam_process = new QProcess(this);
    connect(launch_process, SIGNAL(readyReadStandardOutput()), this, SLOT(updateText()));
//  connect(launch_process, SIGNAL(readyReadStandardError()) , this, SLOT(updateText()));
//  connect(record_process, SIGNAL(readyReadStandardOutput()), this, SLOT(updateText()));
    connect(record_process, SIGNAL(readyReadStandardError()) , this, SLOT(updateText()));
}

MainWindow::~MainWindow()
{
    if (record_flag) {
        pause_process->start("pkill -SIGINT -f record");
        if (pause_process->waitForFinished()) {
            record_folder = "";
            record_flag = FALSE;
        }
    }
    if (launch_flag) {
        sstop_process->start("pkill roslaunch");
        if (sstop_process->waitForFinished()) {
            launch_flag = FALSE;
        }
    }
    delete ui;
}

void MainWindow::updateLocation()
{
    QString state_str;
    if (ui->comboBox_5->currentIndex() == 1) {
        state_str = "Alabama, AL; Alaska, AK; Arizona, AZ; Arkansas, AR; California, CA; Colorado, CO; Connecticut, CT; Delaware, DE; Florida, FL; Georgia, GA; "
                    "Hawaii, HI; Idaho, ID; Illinois, IL; Indiana, IN; Iowa, IA; Kansas, KS; Kentucky, KY; Louislana, LA; Maine, ME; Maryland, MD; "
                    "Massachusetts, MA; Michigan, MI; Minnesota, MN; Mississippi, MS; Missouri, MO; Montana, MT; Nebraska, NE; Nevada, NV; New Hampshire, NH; New Jersey, NJ; "
                    "New Mexico, NM; New York, NY; North Carolina, NC; North Dakota, ND; Ohio, OH; Oklahoma, OK; Oregon, OR; Pennsylvania, PA; Rhode Island, RI; South Carolina, SC; "
                    "South Dakota, SD; Tennessee, TN; Texas, TX; Utah, UT; Vermont, VT; Virginia, VA; Washington, WA; West Virginia, WV; Wisconsin, WI; Wyoming, WY; "
                    "District of Columbia, DC";
    }
    else if (ui->comboBox_5->currentIndex() == 2) {
        state_str = "Ontario, ON; Quebec, QC; Nova Scotia, NS; New Brunswick, NB; Manitoba, MB; British Columbia, BC; "
                    "Prince Edward Island, PE; Saskatchewan, SK; Alberta, AB; Newfoundland and Labrador, NL";
    }
    else if (ui->comboBox_5->currentIndex() == 3) {
        state_str = "Acre, AC; Alagoas, AL; Amapá, AP; Amazonas, AM; Bahia, BA; Ceará, CE; Distrito Federal, DF; Espirito Santo, ES; Goiás, GO; Maranhão, MA; "
                    "MatoGrosso, MT; MatoGrosso do Sul, MS; Minas Gerais, MG; Pará, PA; Paraiba, PB; Paraná, PR; Pernambuco, PE; Piaui, PI; Rio de Janeiro, RJ; Rio Grande do Norte, RN; "
                    "Rio Grande do Sul, RS; Rondônia, RO; Roraima, RR; Santa Catarina, SC; São Paulo, SP; Sergipe, SE; Tocantins, TO";
    }
    else if (ui->comboBox_5->currentIndex() == 4) {
        state_str = "New South Wales, NSW; Queensland, Qld; South Australia, SA; Tasmania, Tas; Victoria, Vic; Western Australia, WA";
    }
    else {
        state_str = "";
    }
    QStringList record_location_state = state_str.split("; ");
    ui->comboBox_6->clear();
    ui->comboBox_6->addItem("    ");
    for (int i=0;i<record_location_state.size();i++) {
        ui->comboBox_6->addItem(record_location_state.value(i));
    }
}

void MainWindow::restoreRecord()
{
    record_file = new QFile("record_file.txt");
    QString record_line;
    if(record_file->open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream record_stream(record_file);
        for (int i=0;i<combobox_list.size()/2;i++) {
            record_line = record_stream.readLine();
            combobox_list.value(i)->setCurrentIndex(record_line.toInt());
        }
        updateLocation();
        for (int i=combobox_list.size()/2;i<combobox_list.size();i++) {
            record_line = record_stream.readLine();
            combobox_list.value(i)->setCurrentIndex(record_line.toInt());
        }
        record_line = record_stream.readLine();
        QStringList check_list = record_line.split("  ");
        for (int i=0;i<checkbox_list.size();i++) {
            if (check_list.value(i) == "1") {
                checkbox_list.value(i)->setChecked(TRUE);
            }
            else {
                checkbox_list.value(i)->setChecked(FALSE);
            }
        }
    }
    record_file->close();
}

void MainWindow::on_pushButton_0_clicked()
{
    if (launch_flag && record_flag) {
        QString rFilePath = "/home/innovations/BagFiles/"+record_folder+"/record.txt";
        rFile = new QFile(rFilePath);
        if (!rFile->open(QIODevice::WriteOnly|QIODevice::Text)) {
            qDebug()<<"Open Failed.";
        }

        QTextStream record_out(rFile);
        record_out<<"Date & Time:        "<<ui->comboBox_1->currentText()<<"-"<<ui->comboBox_2->currentText()<<"-"<<ui->comboBox_3->currentText()<<"-"<<ui->comboBox_4->currentText()<<endl;
        record_out<<"Location:           "<<ui->comboBox_5->currentText()<<"    "<<ui->comboBox_6->currentText()<<endl;
        record_out<<"Vehicle:            "<<ui->comboBox_7->currentText()<<"    "<<ui->comboBox_8->currentText()<<endl;
        record_out<<"Implement:          "<<ui->comboBox_9->currentText()<<endl;
        record_out<<"Crop:               "<<ui->comboBox_10->currentText()<<endl;
        record_out<<"Weather Condition:  ";
        for (int i=0;i<checkbox_list.size();i++) {
            if (checkbox_list.value(i)->isChecked()) {
                record_out<<checkbox_list.value(i)->text()<<"    ";
            }
        }
        record_out<<endl;
        ui->textEdit_show->append("All setting was saved to record.txt.\n");
        record_out.flush();
        rFile->close();

        if (!record_file->open(QIODevice::WriteOnly|QIODevice::Text)) {
            qDebug()<<"Open Failed.";
        }

        QTextStream backup_out(record_file);
        for (int i=0;i<combobox_list.size();i++) {
            backup_out<<combobox_list.value(i)->currentIndex()<<endl;
        }
        for (int i=0;i<checkbox_list.size();i++) {
            backup_out<<checkbox_list.value(i)->isChecked()<<"  ";
        }
        backup_out<<endl;
        backup_out.flush();
        record_file->close();
    }
    else {
        if (!launch_flag) {
            QMessageBox message_1(QMessageBox::NoIcon, "WARNING", "Sensors are not launched.");
            message_1.exec();
        }
        else {
        QMessageBox message_2(QMessageBox::NoIcon, "WARNING", "Start recording to create a new data folder firstly.");
        message_2.exec();
        }
    }
}

void MainWindow::on_pushButton_00_clicked()
{
    restoreRecord();
}

void MainWindow::on_pushButton_1_clicked()
{
    if (!launch_flag) {
        launch_process->start("bash", QStringList()<<"-c"<<"echo Demeter | sudo -S ethtool -G enp4s0 rx 2048;"
                                                           "echo Demeter | sudo -S modprobe peak_usb;"
                                                           "echo Demeter | sudo -S ip link set can0 up type can bitrate 250000;echo Demeter | sudo -S ifconfig can0 up;"
                                                           "echo Demeter | sudo -S ip link set can1 up type can bitrate 250000;echo Demeter | sudo -S ifconfig can1 up;"
                                                           "echo Demeter | sudo -S ip link set can2 up type can bitrate 250000;echo Demeter | sudo -S ifconfig can2 up;"
                                                           "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;source ~/catkin_ws/devel/setup.sh;"
                                                           "roslaunch ~/QT_Projects/sensor.launch");
        if (launch_process->waitForStarted())
            ui->textEdit_show->append("Launch process is started ...\n");
        launch_process->waitForFinished(4000);

        exam_process->start("bash", QStringList()<<"-c"<<"source /opt/ros/kinetic/setup.bash;rostopic list");
        exam_process->waitForFinished();
        QString rostopic_list = exam_process->readAll();

        if (rostopic_list.contains("/right/scan")) {
            ui->textEdit_show->append("Sick 2D-Lidar 571 is launched successfully.\n");
            launch_flag = TRUE;
        }
        else {
            ui->textEdit_show->append("Sick 2D-Lidar 571 launch failed.\n");
        }

        if (rostopic_list.contains("/rtsp_1/image_raw")) {
            ui->textEdit_show->append("Orlaco 90 camera is launched successfully.\n");
            launch_flag = TRUE;
        }
        else {
            ui->textEdit_show->append("Orlaco 90 camera launch failed.\n");
        }

        if (rostopic_list.contains("/rtsp_2/image_raw")) {
            ui->textEdit_show->append("Orlaco 120 camera is launched successfully.\n");
            launch_flag = TRUE;
        }
        else {
            ui->textEdit_show->append("Orlaco 120 camera launch failed.\n");
        }

        if (rostopic_list.contains("/can0/received_messages")) {
            ui->textEdit_show->append("CAN bus No.0 is launched successfully.\n");
            launch_flag = TRUE;
        }
        else {
            ui->textEdit_show->append("CAN bus No.0 launch failed.\n");
        }

        if (rostopic_list.contains("/can1/received_messages")) {
            ui->textEdit_show->append("CAN bus No.1 is launched successfully.\n");
            launch_flag = TRUE;
        }
        else {
            ui->textEdit_show->append("CAN bus No.1 launch failed.\n");
        }

        if (rostopic_list.contains("/can2/received_messages")) {
            ui->textEdit_show->append("CAN bus No.2 is launched successfully.\n");
            launch_flag = TRUE;
        }
        else {
            ui->textEdit_show->append("CAN bus No.2 launch failed.\n");
        }

    }
    else {
        ui->textEdit_show->append("Sensors have been launched.");
    }
//  launch_process->startDetached("/bin/bash", QStringList()<<"/home/innovations/QT_Projects/start.sh");
}

void MainWindow::on_pushButton_2_clicked()
{
    if (launch_flag && !record_flag) {
        QDateTime record_timestamp = QDateTime::currentDateTime();
        QDateTime local_timestamp = record_timestamp.toLocalTime();
        record_folder = local_timestamp.toString("yyyy-MM-dd-hh-mm-ss");
        record_process->start("bash", QStringList()<<"-c"<<"mkdir /home/innovations/BagFiles/"+record_folder);
        record_process->waitForFinished();

        record_process->start("bash", QStringList()<<"-c"<<"source /opt/ros/kinetic/setup.bash;"
                                                           "rosbag record /right/scan /rtsp_1/image_raw /rtsp_2/image_raw /can0/received_messages /can1/received_messages /can2/received_messages "
                                                           "-b 8000 -o ~/BagFiles/"+record_folder+"/data.bag --size=2000 --split");
        record_process->waitForStarted();
        record_process->waitForFinished(2000);

        exam_process->start("bash", QStringList()<<"-c"<<"cd ~/BagFiles/"+record_folder+";ls");
        exam_process->waitForFinished();
        QString bagfile_list = exam_process->readAll();

        if (bagfile_list.contains(".bag.active")) {
            ui->textEdit_show->append("Recording is in process ...\n");
            record_flag = TRUE;
        }
        else {
            ui->textEdit_show->append("Recording failed.\n");
        }
    }
    else {
        if (!launch_flag) {
            QMessageBox message_1(QMessageBox::NoIcon, "WARNING", "Sensors are not launched.");
            message_1.exec();
        }
        else {
            ui->textEdit_show->append("Recording is in process.");
        }
    }
//  record_process->startDetached("/bin/bash", QStringList()<<"/home/innovations/QT_Projects/record.sh");
}

void MainWindow::on_pushButton_3_clicked()
{
    if (launch_flag && record_flag) {
        exam_process->start("bash", QStringList()<<"-c"<<"cd /home/innovations/BagFiles/"+record_folder+";find *.txt");
        exam_process->waitForFinished();
        QString txtfile_list = exam_process->readAll();
        if (txtfile_list.contains("record.txt")) {
            pause_process->start("pkill -SIGINT -f record");
            if (pause_process->waitForFinished()) {
                ui->textEdit_show->append("Recording is stopped, and all .bag files are stored in "+record_folder+" folder.\n");
                record_folder = "";
                record_flag = FALSE;
            }
        }
        else {
            QMessageBox message_3(QMessageBox::NoIcon, "WARNING", "Please create a record file before stop recording.");
            message_3.exec();
        }
    }
    else {
        if (launch_flag) {
            ui->textEdit_show->append("No recording in process.\n");
        }
        else {
            ui->textEdit_show->append("Sensors are not launched.\n");
        }
    }
}

void MainWindow::on_pushButton_4_clicked()
{
    if (launch_flag && record_flag) {
        QMessageBox message_4(QMessageBox::NoIcon, "WARNING", "Recording is in process. Stop record before disconnect.");
        message_4.exec();
    }
    else {
        if (launch_flag) {
            sstop_process->start("pkill roslaunch");
            if (sstop_process->waitForFinished())
                launch_flag = FALSE;
        }
        else {
            ui->textEdit_show->append("Sensors are not launched.\n");
        }
    }
}

void MainWindow::updateText()
{
    QByteArray output_launch_1 = launch_process->readAllStandardOutput();
//  QByteArray output_launch_2 = launch_process->readAllStandardError();
//  QByteArray output_record_1 = record_process->readAllStandardOutput();
    QByteArray output_record_2 = record_process->readAllStandardError();
    ui->textEdit_show->append(output_launch_1);
//  ui->textEdit_show->append(output_launch_2);
//  ui->textEdit_show->append(output_record_1);
    ui->textEdit_show->append(output_record_2);
}

void MainWindow::on_pushButton_5_clicked()
{
    if (launch_flag && record_flag) {
        QString tFilePath = "/home/innovations/BagFiles/"+record_folder+"/trigger.txt";
        tFile = new QFile(tFilePath);
        if (!tFile->open(QIODevice::Append|QIODevice::Text)) {
            qDebug()<<"Open Failed.";
        }

        QDateTime trigger_timestamp = QDateTime::currentDateTime();

        QTextStream trigger_out(tFile);
        trigger_out<<trigger_timestamp.toTime_t()<<endl;
        ui->textEdit_show->append("Current timestamp was saved to "+record_folder+"/trigger.txt.\n");
        trigger_out.flush();
        tFile->close();
    }
    else {
        ui->textEdit_show->append("No recording in process.\n");
    }
}

void MainWindow::on_pushButton_6_clicked()
{

    if (launch_flag && record_flag) {
        QString lFilePath = "/home/innovations/BagFiles/"+record_folder+"/log.txt";
        lFile = new QFile(lFilePath);
        if (!lFile->open(QIODevice::Append|QIODevice::Text)) {
            qDebug()<<"Open Failed.";
        }

        QDateTime log_timestamp = QDateTime::currentDateTime();
        QString log_message = ui->plainTextEdit->toPlainText();

        QTextStream log_out(lFile);
        log_out<<log_timestamp.toTime_t()<<": "<<log_message<<"\n"<<endl;
        ui->textEdit_show->append("Current message was saved to log.txt.\n");
        ui->plainTextEdit->clear();
        log_out.flush();
        lFile->close();
    }
    else {
        ui->textEdit_show->append("No recording in process.\n");
        ui->plainTextEdit->clear();
    }
}
