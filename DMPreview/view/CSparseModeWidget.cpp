#include "CSparseModeWidget.h"
#include "ui_CSparseModeWidget.h"


CSparseModeWidget::CSparseModeWidget(CVideoDeviceController *pVideoDeviceController,
                                     QWidget *parent) :
QWidget(parent),
ui(new Ui::CSparseModeWidget),
m_pVideoDeviceController(pVideoDeviceController)
{
    ui->setupUi(this);
    m_pSparseModeController = m_pVideoDeviceController->GetSparseModeController();
    InitUI();
}

CSparseModeWidget::~CSparseModeWidget()
{
    delete ui;
}

void CSparseModeWidget::showEvent(QShowEvent *event)
{
    UpdateUI();
}

void CSparseModeWidget::UpdateSelf()
{

}

void CSparseModeWidget::InitUI()
{
    UpdateDefaultUI();
    EnableSparseMode(false);
}

void CSparseModeWidget::EnableSparseMode(bool enable)
{
     ui->Slider_DEPTH_LRCHECK_DIFF->setEnabled(enable);
     ui->Slider_SGBM_SAD_THD->setEnabled(enable);
     ui->Slider_SGBM_SAD_RATIO->setEnabled(enable);
     ui->Slider_TEXT_LMT->setEnabled(enable);
     ui->Slider_TEXT_PGAIN->setEnabled(enable);
     ui->Slider_TEXT_NGAIN->setEnabled(enable);
     ui->Slider_SPARSE_THD->setEnabled(enable);
     ui->pushButton_reset_config->setEnabled(enable);
     ui->pushButton_save_config->setEnabled(enable);
}

void CSparseModeWidget::UpdateDefaultUI()
{
    m_pSparseModeController->ReadConfig();
    m_pSparseModeController->DumpConfig();

    UpdateUI_SliderCtrl_DEPTH_LRCHECK_DIFF_Range(0, 7);
    UpdateUI_SliderCtrl_DEPTH_LRCHECK_DIFF(m_pSparseModeController->Default_DEPTH_LRCHECK_DIFF_Value);

    UpdateUI_SliderCtrl_SGBM_SAD_THD_Range(0, 15);
    UpdateUI_SliderCtrl_SGBM_SAD_THD(m_pSparseModeController->Default_SGBM_SAD_THD_Value);

    UpdateUI_SliderCtrl_SGBM_SAD_RATIO_Range(0, 15);
    UpdateUI_SliderCtrl_SGBM_SAD_RATIO(m_pSparseModeController->Default_SGBM_SAD_RATIO_Value);

    UpdateUI_SliderCtrl_TEXT_LMT_Range(0, 127);
    UpdateUI_SliderCtrl_TEXT_LMT(m_pSparseModeController->Default_TEXT_LMT_Value);

    UpdateUI_SliderCtrl_TEXT_PGAIN_Range(0, 7);
    UpdateUI_SliderCtrl_TEXT_PGAIN(m_pSparseModeController->Default_TEXT_PGAIN_Value);

    UpdateUI_SliderCtrl_TEXT_NGAIN_Range(0, 7);
    UpdateUI_SliderCtrl_TEXT_NGAIN(m_pSparseModeController->Default_TEXT_NGAIN_Value);

    UpdateUI_SliderCtrl_SPARSE_THD_Range(0, 31);
    UpdateUI_SliderCtrl_SPARSE_THD(m_pSparseModeController->Default_SPARSE_THD_Value);
}

void CSparseModeWidget::UpdateUI_SliderCtrl_DEPTH_LRCHECK_DIFF(long value)
{
    ui->Slider_DEPTH_LRCHECK_DIFF->setValue(value);
    ui->label_DEPTH_LRCHECK_DIFF_Current->setText(QString::number(value));
}

void CSparseModeWidget::UpdateUI_SliderCtrl_DEPTH_LRCHECK_DIFF_Range(long min, long max)
{
    ui->Slider_DEPTH_LRCHECK_DIFF->blockSignals(true);
    ui->Slider_DEPTH_LRCHECK_DIFF->setMinimum(min);
    ui->Slider_DEPTH_LRCHECK_DIFF->setMaximum(max);
    ui->label_DEPTH_LRCHECK_DIFF_Min->setText(QString::number(min));
    ui->label_DEPTH_LRCHECK_DIFF_Max->setText(QString::number(max));
    ui->Slider_DEPTH_LRCHECK_DIFF->blockSignals(false);
}

void CSparseModeWidget::UpdateUI_SliderCtrl_SGBM_SAD_THD(long value)
{
    ui->Slider_SGBM_SAD_THD->setValue(value);
    ui->label_SGBM_SAD_THD_Current->setText(QString::number(value));
}

void CSparseModeWidget::UpdateUI_SliderCtrl_SGBM_SAD_THD_Range(long min, long max)
{
    ui->Slider_SGBM_SAD_THD->blockSignals(true);
    ui->Slider_SGBM_SAD_THD->setMinimum(min);
    ui->Slider_SGBM_SAD_THD->setMaximum(max);
    ui->label_SGBM_SAD_THD_Min->setText(QString::number(min));
    ui->label_SGBM_SAD_THD_Max->setText(QString::number(max));
    ui->Slider_SGBM_SAD_THD->blockSignals(false);
}

void CSparseModeWidget::UpdateUI_SliderCtrl_SGBM_SAD_RATIO(long value)
{
    ui->Slider_SGBM_SAD_RATIO->setValue(value);
    ui->label_SGBM_SAD_RATIO_Current->setText(QString::number(value));
}

void CSparseModeWidget::UpdateUI_SliderCtrl_SGBM_SAD_RATIO_Range(long min, long max)
{
    ui->Slider_SGBM_SAD_RATIO->blockSignals(true);
    ui->Slider_SGBM_SAD_RATIO->setMinimum(min);
    ui->Slider_SGBM_SAD_RATIO->setMaximum(max);
    ui->label_SGBM_SAD_RATIO_Min->setText(QString::number(min));
    ui->label_SGBM_SAD_RATIO_Max->setText(QString::number(max));
    ui->Slider_SGBM_SAD_RATIO->blockSignals(false);
}

void CSparseModeWidget::UpdateUI_SliderCtrl_TEXT_LMT(long value)
{
    ui->Slider_TEXT_LMT->setValue(value);
    ui->label_TEXT_LMT_Current->setText(QString::number(value));
}

void CSparseModeWidget::UpdateUI_SliderCtrl_TEXT_LMT_Range(long min, long max)
{
    ui->Slider_TEXT_LMT->blockSignals(true);
    ui->Slider_TEXT_LMT->setMinimum(min);
    ui->Slider_TEXT_LMT->setMaximum(max);
    ui->label_TEXT_LMT_Min->setText(QString::number(min));
    ui->label_TEXT_LMT_Max->setText(QString::number(max));
    ui->Slider_TEXT_LMT->blockSignals(false);
}

void CSparseModeWidget::UpdateUI_SliderCtrl_TEXT_PGAIN(long value)
{
    ui->Slider_TEXT_PGAIN->setValue(value);
    ui->label_TEXT_PGAIN_Current->setText(QString::number(value));
}

void CSparseModeWidget::UpdateUI_SliderCtrl_TEXT_PGAIN_Range(long min, long max)
{
    ui->Slider_TEXT_PGAIN->blockSignals(true);
    ui->Slider_TEXT_PGAIN->setMinimum(min);
    ui->Slider_TEXT_PGAIN->setMaximum(max);
    ui->label_TEXT_PGAIN_Min->setText(QString::number(min));
    ui->label_TEXT_PGAIN_Max->setText(QString::number(max));
    ui->Slider_TEXT_PGAIN->blockSignals(false);
}

void CSparseModeWidget::UpdateUI_SliderCtrl_TEXT_NGAIN(long value)
{
    ui->Slider_TEXT_NGAIN->setValue(value);
    ui->label_TEXT_NGAIN_Current->setText(QString::number(value));
}

void CSparseModeWidget::UpdateUI_SliderCtrl_TEXT_NGAIN_Range(long min, long max)
{
    ui->Slider_TEXT_NGAIN->blockSignals(true);
    ui->Slider_TEXT_NGAIN->setMinimum(min);
    ui->Slider_TEXT_NGAIN->setMaximum(max);
    ui->label_TEXT_NGAIN_Min->setText(QString::number(min));
    ui->label_TEXT_NGAIN_Max->setText(QString::number(max));
    ui->Slider_TEXT_NGAIN->blockSignals(false);
}

void CSparseModeWidget::UpdateUI_SliderCtrl_SPARSE_THD(long value)
{
    ui->Slider_SPARSE_THD->setValue(value);
    ui->label_SPARSE_THD_Current->setText(QString::number(value));
}

void CSparseModeWidget::UpdateUI_SliderCtrl_SPARSE_THD_Range(long min, long max)
{
    ui->Slider_SPARSE_THD->blockSignals(true);
    ui->Slider_SPARSE_THD->setMinimum(min);
    ui->Slider_SPARSE_THD->setMaximum(max);
    ui->label_SPARSE_THD_Min->setText(QString::number(min));
    ui->label_SPARSE_THD_Max->setText(QString::number(max));
    ui->Slider_SPARSE_THD->blockSignals(false);
}


void CSparseModeWidget::on_checkBox_enable_sparse_mode_stateChanged(int arg1)
{
    bool bIsChecked = Qt::Checked == arg1 ? true : false;
    if (!bIsChecked)
            return;
    ui->checkBox_enable_sparse_mode->setEnabled(false);
    m_pSparseModeController->UpdateSparseModeConfig(bIsChecked);
    UpdateDefaultUI();
    m_pSparseModeController->UpdateSparseModeDefault();
    EnableSparseMode(bIsChecked);
}

void CSparseModeWidget::on_pushButton_reset_config_clicked()
{
    UpdateDefaultUI();
    m_pSparseModeController->UpdateSparseModeDefault();
}

void CSparseModeWidget::on_pushButton_save_config_clicked()
{
    int value = 0;
    value = ui->Slider_DEPTH_LRCHECK_DIFF->value();
    m_pSparseModeController->WriteConfig(m_pSparseModeController->CONFIG_DEPTH_LRCHECK_DIFF, value);
    value = ui->Slider_SGBM_SAD_THD->value();
    m_pSparseModeController->WriteConfig(m_pSparseModeController->CONFIG_SGBM_SAD_THD, value);
    value = ui->Slider_SGBM_SAD_RATIO->value();
    m_pSparseModeController->WriteConfig(m_pSparseModeController->CONFIG_SGBM_SAD_RATIO, value);
    value = ui->Slider_TEXT_LMT->value();
    m_pSparseModeController->WriteConfig(m_pSparseModeController->CONFIG_TEXT_LMT, value);
    value = ui->Slider_TEXT_PGAIN->value();
    m_pSparseModeController->WriteConfig(m_pSparseModeController->CONFIG_TEXT_PGAIN, value);
    value = ui->Slider_TEXT_NGAIN->value();
    m_pSparseModeController->WriteConfig(m_pSparseModeController->CONFIG_TEXT_NGAIN, value);
    value = ui->Slider_SPARSE_THD->value();
    m_pSparseModeController->WriteConfig(m_pSparseModeController->CONFIG_SPARSE_THD, value);
}

void CSparseModeWidget::on_Slider_DEPTH_LRCHECK_DIFF_valueChanged(int value)
{
    m_pSparseModeController->UpdateSparseModeReg(m_pSparseModeController->CONFIG_DEPTH_LRCHECK_DIFF, value);
    UpdateUI_SliderCtrl_DEPTH_LRCHECK_DIFF(value);
}

void CSparseModeWidget::on_Slider_SGBM_SAD_THD_valueChanged(int value)
{
    m_pSparseModeController->UpdateSparseModeReg(m_pSparseModeController->CONFIG_SGBM_SAD_THD, value);
    UpdateUI_SliderCtrl_SGBM_SAD_THD(value);
}

void CSparseModeWidget::on_Slider_SGBM_SAD_RATIO_valueChanged(int value)
{
    m_pSparseModeController->UpdateSparseModeReg(m_pSparseModeController->CONFIG_SGBM_SAD_RATIO, value);
    UpdateUI_SliderCtrl_SGBM_SAD_RATIO(value);
}

void CSparseModeWidget::on_Slider_TEXT_LMT_valueChanged(int value)
{
    m_pSparseModeController->UpdateSparseModeReg(m_pSparseModeController->CONFIG_TEXT_LMT, value);
    UpdateUI_SliderCtrl_TEXT_LMT(value);
}

void CSparseModeWidget::on_Slider_TEXT_PGAIN_valueChanged(int value)
{
    m_pSparseModeController->UpdateSparseModeReg(m_pSparseModeController->CONFIG_TEXT_PGAIN, value);
    UpdateUI_SliderCtrl_TEXT_PGAIN(value);
}

void CSparseModeWidget::on_Slider_TEXT_NGAIN_valueChanged(int value)
{
    m_pSparseModeController->UpdateSparseModeReg(m_pSparseModeController->CONFIG_TEXT_NGAIN, value);
    UpdateUI_SliderCtrl_TEXT_NGAIN(value);
}

void CSparseModeWidget::on_Slider_SPARSE_THD_valueChanged(int value)
{
    m_pSparseModeController->UpdateSparseModeReg(m_pSparseModeController->CONFIG_SPARSE_THD, value);
    UpdateUI_SliderCtrl_SPARSE_THD(value);
}

