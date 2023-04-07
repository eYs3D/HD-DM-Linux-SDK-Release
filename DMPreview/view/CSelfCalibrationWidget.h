#ifndef CSELFCALIBRATIONWIDGET_H
#define CSELFCALIBRATIONWIDGET_H

#include <QWidget>
#include <QTimer>
#include "CEYSDUIView.h"
#include "ModuleCalibParWrite.h"
#include "Self_Calibration_API.h"

namespace Ui {
class CSelfCalibrationWidget;
}

class CSelfCalibrationWidget : public QWidget,
                               public CEYSDUIView
{
    Q_OBJECT

public:
    explicit CSelfCalibrationWidget(CVideoDeviceController *pVideoDeviceController,
                                    QWidget *parent = nullptr);
    ~CSelfCalibrationWidget();

    virtual void showEvent(QShowEvent *event);

    virtual void UpdateSelf();

private:
    void getErrorMessage(self_calibration::self_calibration_issue error, QString &message);
    bool updateMonitorState();
    bool updateCurrentPars();
    bool updateEstimatedPars();

private slots:
    void on_checkBox_run_self_k_stateChanged(int arg1);

    void on_lineEdit_schedule_value_textChanged(const QString &arg1);

    void on_pushButton_factory_reset_clicked();

    void on_checkBox_advance_stateChanged(int arg1);

    void on_pushButton_strategy_read_clicked();

    void on_pushButton_strategy_write_clicked();

    void on_pushButton_strategy_reset_clicked();

    void on_pushButton_operator_calculate_score_clicked();

    void on_pushButton_operator_do_calibration_clicked();

    void on_pushButton_operator_apply_before_reg_clicked();

    void on_pushButton_operator_apply_after_reg_clicked();

    void on_pushButton_operator_apply_after_flash_clicked();

    void on_pushButton_operator_reset_all_clicked();

private:
    void InitUI();


private:
    Ui::CSelfCalibrationWidget *ui;
    CVideoDeviceController *m_pVideoDeviceController;
    QTimer *Check_BasicSchedule_Timer;
    BOOL mRun;
    int mBasicSchedule;
    self_calibration::self_calibration_api *p_self_k = nullptr;
};

#endif // CSELFCALIBRATIONWIDGET_H
