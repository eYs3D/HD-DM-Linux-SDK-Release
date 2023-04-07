#include <QMessageBox>
#include <QRegExpValidator>
#include "CSelfCalibrationWidget.h"
#include "ui_CSelfCalibrationWidget.h"
#include "CVideoDeviceController.h"

CSelfCalibrationWidget::CSelfCalibrationWidget(CVideoDeviceController *pVideoDeviceController,
                                               QWidget *parent) :
QWidget(parent),
ui(new Ui::CSelfCalibrationWidget),
m_pVideoDeviceController(pVideoDeviceController)
{
    ui->setupUi(this);
    mRun = false;
    mBasicSchedule = 1;
    p_self_k = m_pVideoDeviceController->GetSelfcalibrationController()->GetSelf_calibration();
    Check_BasicSchedule_Timer = new QTimer(this);
    connect(Check_BasicSchedule_Timer, &QTimer::timeout, [=](){
        m_pVideoDeviceController->GetSelfcalibrationController()->executeSelfK();
    });
    InitUI();
}

CSelfCalibrationWidget::~CSelfCalibrationWidget()
{
    delete ui;
}

void CSelfCalibrationWidget::showEvent(QShowEvent *event)
{
    UpdateUI();
}


void CSelfCalibrationWidget::InitUI()
{
    ui->checkBox_run_self_k->setChecked(false);
    ui->checkBox_advance->setChecked(false);
    ui->lineEdit_schedule_value->setValidator(new QRegExpValidator(QRegExp("[0-9]+$")));
    ui->lineEdit_monitor_after_score->setEnabled(false);
    ui->lineEdit_monitor_after_state->setEnabled(false);
    ui->lineEdit_monitor_before_score->setEnabled(false);
    ui->lineEdit_monitor_before_state->setEnabled(false);
    ui->lineEdit_monitor_state->setEnabled(false);
    ui->lineEdit_strategy_accuracy_level->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_auto_calibration->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_calibration_mode->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_do_qulity_monitor->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_max_error_trials->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_sleep_time_for_quality_check->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_tare_target->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_target_quality->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_threshold_auto_calibrarting->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
    ui->lineEdit_strategy_threshold_for_bad_quality->setValidator(new QRegExpValidator(QRegExp(QString("^-?\\d{1,}(\\.[0-9]+)?"))));
}


void CSelfCalibrationWidget::UpdateSelf()
{
    ui->lineEdit_schedule_value->setText(QString::number(mBasicSchedule));
    ui->lineEdit_schedule_value->setEnabled(ui->checkBox_run_self_k->checkState() == Qt::Unchecked);
    ui->pushButton_factory_reset->setEnabled(ui->checkBox_run_self_k->checkState() == Qt::Unchecked);
    ui->widget_auto_layout_l->setEnabled(ui->checkBox_advance->checkState() == Qt::Unchecked);
    ui->widget_advance_layout->setEnabled(ui->checkBox_advance->checkState() == Qt::Checked);
}

void CSelfCalibrationWidget::on_checkBox_run_self_k_stateChanged(int arg1)
{
    bool bIsChecked = Qt::Checked == arg1 ? true : false;
    if (bIsChecked) {
        if (!m_pVideoDeviceController->GetSelfcalibrationController()->checkLRDMode()) {
            ui->checkBox_run_self_k->setCheckState(Qt::Unchecked);
            CMessageManager::Error("Must preview L+R+D");
            return;
        }  else if (mBasicSchedule < 1) {
            ui->checkBox_run_self_k->setCheckState(Qt::Unchecked);
            CMessageManager::Error("schedule must be more than one second");
            return;
        } else {
            m_pVideoDeviceController->GetSelfcalibrationController()->getImageParams();
            Check_BasicSchedule_Timer->start(mBasicSchedule * 1000);
        }
    } else {
        Check_BasicSchedule_Timer->stop();
    }
    UpdateSelf();
}

void CSelfCalibrationWidget::on_lineEdit_schedule_value_textChanged(const QString &arg1)
{
    if (!arg1.size() == 0) {
        mBasicSchedule = arg1.toInt();
    }
    UpdateSelf();
}

void CSelfCalibrationWidget::on_pushButton_factory_reset_clicked()
{
    ui->pushButton_factory_reset->setEnabled(false);
    QMessageBox::StandardButton result = QMessageBox::question(
                this, "Dangerous action !!",
                "Pressed Yes, all your calibration files in G2 section "
                "will be override by the value in G1.",
                QMessageBox::Yes|QMessageBox::No);

    if (result == QMessageBox::Yes) {
        auto resultString = m_pVideoDeviceController->GetVideoDeviceModel()->CopyAllFileToG2();
        QMessageBox::StandardButton result = QMessageBox::information(
                    this, "Result",
                    resultString.c_str());
    }
    ui->pushButton_factory_reset->setEnabled(true);
}

void CSelfCalibrationWidget::on_checkBox_advance_stateChanged(int arg1)
{
    bool bIsChecked = Qt::Checked == arg1 ? true : false;
    if (bIsChecked) {
        if (!m_pVideoDeviceController->GetSelfcalibrationController()->checkLRDMode()) {
            ui->checkBox_advance->setCheckState(Qt::Unchecked);
            CMessageManager::Error("Must preview L+R+D");
        } else {
            m_pVideoDeviceController->GetSelfcalibrationController()->getImageParams();
            ui->checkBox_run_self_k->setChecked(false);
        }
    }
    UpdateSelf();
}

void CSelfCalibrationWidget::on_pushButton_strategy_read_clicked()
{
       bool is_monitor = 0;
       int second = 0;
       float score1 = 0;
       float score2 = 0;
       bool is_auto = 0;
       float target = 0;
       self_calibration::calibration_mode mode = self_calibration::calibration_mode::Tare;
       int level = 0;
       int number = 0;
       float distance = 0;

       if (p_self_k)
       {
           self_calibration::self_calibration_issue ret = self_calibration::self_calibration_issue::Self_Calibration_Success;
           ret = p_self_k->get_do_quality_monitoring(&is_monitor);
           ret = p_self_k->get_period_for_quality_check(&second);
           ret = p_self_k->get_low_quality_threshold_to_trigger_warning(&score1);
           ret = p_self_k->get_quality_threshold_to_trigger_parameter_estimation_for_recovery(&score2);
           ret = p_self_k->get_automatically_triger_parameter_estimation_for_recovery(&is_auto);
           ret = p_self_k->get_target_quality_threshold_for_recovery(&target);
           ret = p_self_k->get_recovery_mode(&mode);
           ret = p_self_k->get_accuracy_level(&level);
           ret = p_self_k->get_max_recovery_trials(&number);
           ret = p_self_k->get_tare_distance(&distance);

           ui->lineEdit_strategy_do_qulity_monitor->setText(QString::number(is_monitor));
           ui->lineEdit_strategy_sleep_time_for_quality_check->setText(QString::number(second));
           ui->lineEdit_strategy_threshold_for_bad_quality->setText(QString::number(score1, 'f', 2));
           ui->lineEdit_strategy_threshold_auto_calibrarting->setText(QString::number(score2, 'f', 2));
           ui->lineEdit_strategy_auto_calibration->setText(QString::number(is_auto));
           ui->lineEdit_strategy_target_quality->setText(QString::number(target, 'f', 2));
           ui->lineEdit_strategy_calibration_mode->setText(QString::number(mode));
           ui->lineEdit_strategy_accuracy_level->setText(QString::number(level));
           ui->lineEdit_strategy_max_error_trials->setText(QString::number(number));
           ui->lineEdit_strategy_tare_target->setText(QString::number(distance, 'f', 2));
       }
}

void CSelfCalibrationWidget::on_pushButton_strategy_write_clicked()
{
    QString str_is_monitor = ui->lineEdit_strategy_do_qulity_monitor->text();
    QString str_secondr = ui->lineEdit_strategy_sleep_time_for_quality_check->text();
    QString str_score1 = ui->lineEdit_strategy_threshold_for_bad_quality->text();
    QString str_score2 = ui->lineEdit_strategy_threshold_auto_calibrarting->text();
    QString str_is_auto = ui->lineEdit_strategy_auto_calibration->text();
    QString str_target = ui->lineEdit_strategy_target_quality->text();
    QString str_mode = ui->lineEdit_strategy_calibration_mode->text();
    QString str_level = ui->lineEdit_strategy_accuracy_level->text();
    QString str_number = ui->lineEdit_strategy_max_error_trials->text();
    QString str_distance = ui->lineEdit_strategy_tare_target->text();

    if (str_is_monitor.size() == 0) {
        CMessageManager::Error("Strategy input is_monitor is empty!");
        return;
    } else if (str_secondr.size() == 0) {
        CMessageManager::Error("Strategy input second is empty!");
        return;
    } else if (str_score1.size() == 0) {
        CMessageManager::Error("Strategy input score1 is empty!");
        return;
    } else if (str_score2.size() == 0) {
        CMessageManager::Error("Strategy input score2 is empty!");
        return;
    } else if (str_is_auto.size() == 0) {
        CMessageManager::Error("Strategy input is_auto is empty!");
        return;
    } else if (str_target.size() == 0) {
        CMessageManager::Error("Strategy input target is empty!");
        return;
    } else if (str_mode.size() == 0) {
        CMessageManager::Error("Strategy input mode is empty!");
        return;
    } else if (str_level.size() == 0) {
        CMessageManager::Error("Strategy input level is empty!");
        return;
    } else if (str_number.size() == 0) {
        CMessageManager::Error("Strategy input number is empty!");
        return;
    } else if (str_distance.size() == 0) {
        CMessageManager::Error("Strategy input distance is empty!");
        return;
    } else {
        bool is_monitor = str_is_monitor.toInt();
        int second = str_secondr.toInt();
        float score1 = str_score1.toFloat();
        float score2 = str_score1.toFloat();
        bool is_auto = str_is_auto.toInt();
        float target = str_score1.toFloat();
        self_calibration::calibration_mode mode  = (self_calibration::calibration_mode) str_mode.toInt();
        int level = str_level.toInt();
        int number = str_number.toInt();
        float distance = str_distance.toFloat();

        if (p_self_k)
        {
            self_calibration::self_calibration_issue ret = self_calibration::self_calibration_issue::Self_Calibration_Success;
            ret = p_self_k->set_do_quality_monitoring(is_monitor);
            ret = p_self_k->set_period_for_quality_check(second);
            ret = p_self_k->set_low_quality_threshold_to_trigger_warning(score1);
            ret = p_self_k->set_quality_threshold_to_trigger_parameter_estimation_for_recovery(score2);
            ret = p_self_k->set_automatically_triger_parameter_estimation_for_recovery(is_auto);
            ret = p_self_k->set_target_quality_threshold_for_recovery(target);
            ret = p_self_k->set_recovery_mode(mode);
            ret = p_self_k->set_accuracy_level(level);
            ret = p_self_k->set_max_recovery_trials(number);
            ret = p_self_k->set_tare_distance(distance);
        }
    }
}

void CSelfCalibrationWidget::on_pushButton_strategy_reset_clicked()
{
    if (p_self_k)
    {
        self_calibration::self_calibration_issue ret = p_self_k->resetConfiguration();
        on_pushButton_strategy_read_clicked();
    }
}

void CSelfCalibrationWidget::on_pushButton_operator_calculate_score_clicked()
{
    self_calibration::self_calibration_issue ret = m_pVideoDeviceController->GetSelfcalibrationController()->operator_calculate_score();
    if (ret == self_calibration::Self_Calibration_Success && updateMonitorState() && updateCurrentPars())
    {
        QMessageBox::information(this, "Info", "Done");

    }
    else
    {
        QString error_message("info:");
        getErrorMessage(ret, error_message);
        QMessageBox::information(this, "Error", error_message);
    }
}

void CSelfCalibrationWidget::on_pushButton_operator_do_calibration_clicked()
{
    self_calibration::self_calibration_issue ret = m_pVideoDeviceController->GetSelfcalibrationController()->operator_do_calibration();
    if (ret == self_calibration::Self_Calibration_Success && updateMonitorState() && updateCurrentPars() && updateEstimatedPars())
    {
        QMessageBox::information(this, "Info", "Done");
    }
    else
    {
        QString error_message("info:");
        getErrorMessage(ret, error_message);
        QMessageBox::information(this, "Error", error_message);
    }
}

void CSelfCalibrationWidget::on_pushButton_operator_apply_before_reg_clicked()
{
    m_pVideoDeviceController->GetSelfcalibrationController()->operator_apply_before_reg();
    QMessageBox::information(this, "Info", "Done");
}

void CSelfCalibrationWidget::on_pushButton_operator_apply_after_reg_clicked()
{
    m_pVideoDeviceController->GetSelfcalibrationController()->operator_apply_after_reg();
    QMessageBox::information(this, "Info", "Done");
}

void CSelfCalibrationWidget::on_pushButton_operator_apply_after_flash_clicked()
{
    m_pVideoDeviceController->GetSelfcalibrationController()->operator_apply_after_flash();
    QMessageBox::information(this, "Info", "Done");
}

void CSelfCalibrationWidget::on_pushButton_operator_reset_all_clicked()
{
    m_pVideoDeviceController->GetSelfcalibrationController()->operator_reset_all();
    QMessageBox::information(this, "Info", "Done");
}

void CSelfCalibrationWidget::getErrorMessage(self_calibration::self_calibration_issue error, QString &message)
{
    switch (error)
    {
        case self_calibration::self_calibration_issue::Self_Calibration_Success:
            message.append("Success");
            break;
        case self_calibration::self_calibration_issue::Self_Calibration_Fail:
            message.append("Fail");
            break;
        case self_calibration::self_calibration_issue::Self_Calibration_ParameterError:
            message.append("ParameterError");
            break;
        case self_calibration::self_calibration_issue::Self_Calibration_FrameError:
            message.append("FrameError");
            break;
        case self_calibration::self_calibration_issue::Self_Calibration_TareDepthError:
            message.append("TareDepthError");
            break;
        case self_calibration::self_calibration_issue::Self_Calibration_PlaneError:
            message.append("PlaneError");
            break;
        case self_calibration::self_calibration_issue::Self_Calibration_QualityBad:
            message.append("QualityBad");
            break;
        case self_calibration::self_calibration_issue::Self_Calibration_NotReady:
            message.append("NotReady");
            break;
    }
}

bool CSelfCalibrationWidget::updateMonitorState()
{
    if (!p_self_k) {
        return false;
    }
    self_calibration::monitor_state state;
    self_calibration::self_calibration_issue ret = p_self_k->getMonitorState(&state);
    if (ret != self_calibration::Self_Calibration_Success)
        return false;

    switch (state)
    {
        case self_calibration::monitor_state::Initing:
            ui->lineEdit_monitor_state->setText("Initing");
            return true;
        case self_calibration::monitor_state::Idle:
            ui->lineEdit_monitor_state->setText("Idle");
            return true;
        case self_calibration::monitor_state::Suspended:
            ui->lineEdit_monitor_state->setText("Suspended");
            return true;
        case self_calibration::monitor_state::Calibrating:
            ui->lineEdit_monitor_state->setText("Calibrating");
            return true;
        case self_calibration::monitor_state::CalculatingHealth:
            ui->lineEdit_monitor_state->setText("CalculatingHealth");
            return true;
    }
    return true;
}

bool CSelfCalibrationWidget::updateCurrentPars()
{
    if (!p_self_k) {
        return false;
    }
    self_calibration::parameter_state current_state;
    self_calibration::self_calibration_issue ret = p_self_k->getStateOfCurrentPars(&current_state);
    if (ret == self_calibration::Self_Calibration_Success)
    {
        if (current_state == self_calibration::Waiting)
        {
            ui->lineEdit_monitor_before_state->setText("Waiting");
        }
        else if (current_state == self_calibration::Good)
        {
            ui->lineEdit_monitor_before_state->setText("Good");
        }
        else if (current_state == self_calibration::CouldbeImproved)
        {
            ui->lineEdit_monitor_before_state->setText("CouldBeImproved");
        }
        else if (current_state == self_calibration::NeedCalibration)
        {
            ui->lineEdit_monitor_before_state->setText("NeedCalibration");
        }
        else
        {
            ui->lineEdit_monitor_before_state->setText("Unknown");
        }
    }
    else
    {
        return false;
    }

    float score = 0;
    ret = p_self_k->getScroeOfCurrentPars(&score);
    if (ret == self_calibration::Self_Calibration_Success)
    {
        ui->lineEdit_monitor_before_score->setText(QString::number(score, 'f', 2));
        return true;
    }
    return false;
}

bool CSelfCalibrationWidget::updateEstimatedPars()
{
    if (!p_self_k) {
        return false;
    }
    self_calibration::parameter_state current_state;
    self_calibration::self_calibration_issue ret = p_self_k->getStateOfEstimatedPars(&current_state);
    if (ret == self_calibration::Self_Calibration_Success)
    {
        if (current_state == self_calibration::Waiting)
        {
            ui->lineEdit_monitor_after_state->setText("Waiting");
        }
        else if (current_state == self_calibration::Good)
        {
            ui->lineEdit_monitor_after_state->setText("Good");
        }
        else if (current_state == self_calibration::CouldbeImproved)
        {
            ui->lineEdit_monitor_after_state->setText("CouldBeImproved");
        }
        else if (current_state == self_calibration::NeedCalibration)
        {
            ui->lineEdit_monitor_after_state->setText("NeedCalibration");
        }
        else
        {
            ui->lineEdit_monitor_after_state->setText("Unknown");
        }
    }
    else
    {
        return false;
    }

    float score = 0;
    ret = p_self_k->getScroeOfEstimatedPars(&score);
    if (ret == self_calibration::Self_Calibration_Success)
    {
        ui->lineEdit_monitor_after_score->setText(QString::number(score, 'f', 2));
        return true;
    }
    return false;
}
