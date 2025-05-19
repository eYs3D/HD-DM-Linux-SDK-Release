#ifndef CVIDEODEVICEREGISTERWIDGET_H
#define CVIDEODEVICEREGISTERWIDGET_H

#include <QWidget>
#include <vector>
#include "CEYSDUIView.h"

namespace Ui {
class CVideoDeviceRegisterWidget;
}

class CRegisterReadWriteController;
class QLineEdit;
class CVideoDeviceRegisterWidget : public QWidget,
                                   public CEYSDUIView
{
    Q_OBJECT
public:
    explicit CVideoDeviceRegisterWidget(CRegisterReadWriteController *pRegisterReadWriteController,
                                        QWidget *parent = nullptr);
    ~CVideoDeviceRegisterWidget();

    virtual void UpdateSelf();

    virtual void showEvent(QShowEvent *event);
    virtual void paintEvent(QPaintEvent *event);

private slots:
    void on_radioButton_ic2_clicked();

    void on_radioButton_asic_clicked();

    void on_radioButton_fw_clicked();

    void on_lineEdit_slave_id_textChanged(const QString &text);

    void on_checkBox_address_2_byte_stateChanged(int state);

    void on_checkBox_value_2_byte_stateChanged(int state);

    void on_radioButton_sensor_1_clicked();

    void on_radioButton_sensor_2_clicked();

    void on_radioButton_sensor_3_clicked();

    void on_radioButton_sensor_4_clicked();

    void on_radioButton_sensor_all_clicked();

    void on_checkBox_peroiodic_read_stateChanged(int state);

    void on_spinBox_periodic_read_time_valueChanged(int value);

    void on_checkBox_save_log_stateChanged(int state);

    void on_pushButton_read_clicked();

    void on_pushButton_stop_reading_clicked();

    void on_pushButton_write_clicked();

    void on_checkBox_swap_bytes_order_stateChanged(int arg1);

    void on_comboBox_device_selector_currentIndexChanged(int index);

private:
    void UpdateDevice();
    void UpdateType();
    void UpdateSlaveID();
    void UpdateAddressSize();
    void UpdateValueSize();
    void UpdateSensorMode();
    void UpdateAddressAndValue();
    void UpdateOptions();
    void UpdateButtonState();
    void UpdateSwapBytesOrderCheckBox();


private:
    CRegisterReadWriteController *m_pRegisterReadWriteController;
    Ui::CVideoDeviceRegisterWidget *ui;

    std::vector<QLineEdit *> m_requestAddressEdit;
    std::vector<QLineEdit *> m_requestValueEdit;
    std::map<std::string, int> m_mapDevDesc;
};

#endif // CVIDEODEVICEREGISTERWIDGET_H
