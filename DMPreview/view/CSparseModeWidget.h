#ifndef CSPARSEMODEWIDGET_H
#define CSPARSEMODEWIDGET_H

#include <QWidget>
#include "CEYSDUIView.h"
#include "CVideoDeviceController.h"
#include "CSparseModeController.h"



namespace Ui {
class CSparseModeWidget;
}

class CSparseModeWidget : public QWidget,
                          public CEYSDUIView
{
    Q_OBJECT
public:
    explicit CSparseModeWidget(CVideoDeviceController *pVideoDeviceController, QWidget *parent = nullptr);
    ~CSparseModeWidget();
    virtual void showEvent(QShowEvent *event);
    virtual void UpdateSelf();

private slots:
    void on_checkBox_enable_sparse_mode_stateChanged(int arg1);

    void on_pushButton_reset_config_clicked();

    void on_pushButton_save_config_clicked();

    void on_Slider_DEPTH_LRCHECK_DIFF_valueChanged(int value);

    void on_Slider_SGBM_SAD_THD_valueChanged(int value);

    void on_Slider_SGBM_SAD_RATIO_valueChanged(int value);

    void on_Slider_TEXT_LMT_valueChanged(int value);

    void on_Slider_TEXT_PGAIN_valueChanged(int value);

    void on_Slider_TEXT_NGAIN_valueChanged(int value);

    void on_Slider_SPARSE_THD_valueChanged(int value);

private:
    void InitUI();
    void EnableSparseMode(bool enable);
    void UpdateDefaultUI();
    void UpdateUI_SliderCtrl_DEPTH_LRCHECK_DIFF(long value);
    void UpdateUI_SliderCtrl_DEPTH_LRCHECK_DIFF_Range(long min, long max);
    void UpdateUI_SliderCtrl_SGBM_SAD_THD(long value);
    void UpdateUI_SliderCtrl_SGBM_SAD_THD_Range(long min, long max);
    void UpdateUI_SliderCtrl_SGBM_SAD_RATIO(long value);
    void UpdateUI_SliderCtrl_SGBM_SAD_RATIO_Range(long min, long max);
    void UpdateUI_SliderCtrl_TEXT_LMT(long value);
    void UpdateUI_SliderCtrl_TEXT_LMT_Range(long min, long max);
    void UpdateUI_SliderCtrl_TEXT_PGAIN(long value);
    void UpdateUI_SliderCtrl_TEXT_PGAIN_Range(long min, long max);
    void UpdateUI_SliderCtrl_TEXT_NGAIN(long value);
    void UpdateUI_SliderCtrl_TEXT_NGAIN_Range(long min, long max);
    void UpdateUI_SliderCtrl_SPARSE_THD(long value);
    void UpdateUI_SliderCtrl_SPARSE_THD_Range(long min, long max);




private:
    Ui::CSparseModeWidget *ui;
    CVideoDeviceController *m_pVideoDeviceController;
    CSparseModeController *m_pSparseModeController;

};

#endif // CSPARSEMODEWIDGET_H
