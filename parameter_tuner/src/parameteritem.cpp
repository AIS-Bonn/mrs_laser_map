//subclass for slider
//Author: Sebastian Schüller

#include "parameteritem.h"
#include <QHBoxLayout>
#include <QEvent>

#include <ros/console.h>

namespace parametertuner
{

WheelFilter::WheelFilter(QObject* parent): QObject(parent)
{}

bool WheelFilter::eventFilter(QObject* object, QEvent* event )
{
	if (event->type() == QEvent::Wheel)
		return true;
	return false;
}



FloatParameterWidget::FloatParameterWidget(ros::NodeHandle& nh, const config_server::ParameterDescription& description)
 : ParameterWidgetBase()
 , m_parameter(description, &nh, false)
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_slider = new QSlider(Qt::Horizontal,this);
	m_spinbox = new QDoubleSpinBox(this);
	layout->addWidget(m_slider);
	layout->addWidget(m_spinbox);

	layout->setContentsMargins(QMargins());

	WheelFilter* filter = new WheelFilter(this);

	m_slider->installEventFilter(filter);
	m_spinbox->installEventFilter(filter);

	m_sliderStepRatio = 1.0 / (double)description.step;

	m_slider->setMinimum(description.min * m_sliderStepRatio);
	m_slider->setMaximum(description.max * m_sliderStepRatio);
	m_slider->setSingleStep(1);
	m_slider->setValue(m_parameter() * m_sliderStepRatio);

	m_spinbox->setMinimum(description.min);
	m_spinbox->setMaximum(description.max);
	m_spinbox->setSingleStep(description.step);
	m_spinbox->setDecimals(4);
	m_spinbox->setValue(m_parameter());
	m_spinbox->setMinimumWidth(80);
	m_spinbox->setAlignment(Qt::AlignRight);

	m_parameter.setCallback(boost::bind(&FloatParameterWidget::called, this, _1));

	connect(m_slider, SIGNAL(valueChanged(int)), this, SLOT(handleSlider()));
	connect(m_spinbox, SIGNAL(editingFinished()), this, SLOT(handleSpinbox()));
	connect(this, SIGNAL(called(float)), this, SLOT(handleCallback(float)), Qt::QueuedConnection);
}

FloatParameterWidget::~FloatParameterWidget()
{
}

void FloatParameterWidget::DecValue()
{
	m_spinbox->setValue(m_spinbox->value() - m_spinbox->singleStep());
	handleSpinbox();
}

void FloatParameterWidget::IncValue()
{
	m_spinbox->setValue(m_spinbox->value() + m_spinbox->singleStep());
	handleSpinbox();
}



void FloatParameterWidget::handleSpinbox()
{
	float value = m_spinbox->value();
	m_slider->blockSignals(true);
	m_slider->setValue(value * m_sliderStepRatio);
	m_slider->blockSignals(false);
	m_parameter.set(value);
}


void FloatParameterWidget::handleCallback(float value)
{
	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);
	m_slider->setValue(value * m_sliderStepRatio);
	m_spinbox->setValue(value);
	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);

}

void FloatParameterWidget::handleSlider()
{
	float value = ((float)m_slider->value()) / m_sliderStepRatio;
	m_spinbox->blockSignals(true);
	m_spinbox->setValue(value);
	m_spinbox->blockSignals(false);
	m_parameter.set(value);
}

IntParameterWidget::IntParameterWidget(ros::NodeHandle& nh, const config_server::ParameterDescription& description)
 : ParameterWidgetBase()
 , m_parameter(description, &nh, false)
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_slider = new QSlider(Qt::Horizontal,this);
	m_spinbox = new QSpinBox(this);
	layout->addWidget(m_slider);
	layout->addWidget(m_spinbox);

	layout->setContentsMargins(QMargins());

	m_slider->setMinimum(description.min);
	m_slider->setMaximum(description.max);
	m_slider->setSingleStep(description.step);
	m_slider->setValue(m_parameter());

	m_spinbox->setMinimum(description.min);
	m_spinbox->setMaximum(description.max);
	m_spinbox->setSingleStep(description.step);
	m_spinbox->setValue(m_parameter());
	m_spinbox->setMinimumWidth(80);
	m_spinbox->setAlignment(Qt::AlignRight);

	m_parameter.setCallback(boost::bind(&IntParameterWidget::called, this, _1));

	connect(m_slider, SIGNAL(valueChanged(int)), this, SLOT(handleSlider()));
	connect(m_spinbox, SIGNAL(editingFinished()), this, SLOT(handleSpinbox()));
	connect(this, SIGNAL(called(int)), this, SLOT(handleCallback(int)), Qt::QueuedConnection);
}

IntParameterWidget::~IntParameterWidget()
{
}

void IntParameterWidget::DecValue()
{
	m_slider->setValue(m_slider->value() - 1);
	handleSpinbox();
}


void IntParameterWidget::IncValue()
{
	m_slider->setValue(m_slider->value() + 1);
	handleSpinbox();
}



void IntParameterWidget::handleSlider()
{
	int value = m_slider->value();
	m_spinbox->blockSignals(true);
	m_spinbox->setValue(value);
	m_spinbox->blockSignals(false);
	m_parameter.set(value);
}

void IntParameterWidget::handleSpinbox()
{
	int value = m_spinbox->value();
	m_slider->blockSignals(true);
	m_slider->setValue(value);
	m_slider->blockSignals(false);
	m_parameter.set(value);
}


void IntParameterWidget::handleCallback(int value)
{
	m_slider->blockSignals(true);
	m_spinbox->blockSignals(true);
	m_slider->setValue(value);
	m_spinbox->setValue(value);
	m_slider->blockSignals(false);
	m_spinbox->blockSignals(false);
}



StringParameterWidget::StringParameterWidget(ros::NodeHandle& nh, const config_server::ParameterDescription& description)
 : ParameterWidgetBase()
 , m_parameter(description, &nh, false)
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_lineEdit = new QLineEdit(this);
	layout->addWidget(m_lineEdit);

	layout->setContentsMargins(QMargins());

	m_lineEdit->setText(QString::fromStdString(m_parameter()));

	m_parameter.setCallback(boost::bind(&StringParameterWidget::called, this, _1));

	connect(m_lineEdit, SIGNAL(editingFinished()), this, SLOT(handleLineEdit()));
	connect(this, SIGNAL(called(std::string)), this, SLOT(handleCallback(std::string)), Qt::QueuedConnection);
}

StringParameterWidget::~StringParameterWidget()
{}

void StringParameterWidget::DecValue()
{}

void StringParameterWidget::IncValue()
{}


void StringParameterWidget::handleLineEdit()
{
	std::string text = m_lineEdit->text().toStdString();
	m_parameter.set(text);
}

void StringParameterWidget::handleCallback(std::string text)
{
	m_lineEdit->blockSignals(true);
	m_lineEdit->setText(QString::fromStdString(text));
	m_lineEdit->blockSignals(false);
}


BoolParameterWidget::BoolParameterWidget(ros::NodeHandle& nh, const config_server::ParameterDescription& description)
 : ParameterWidgetBase()
 , m_parameter(description, &nh, false)
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	m_checkBox = new QCheckBox(this);
	layout->addWidget(m_checkBox);

	m_checkBox->setChecked(m_parameter());

	m_parameter.setCallback(boost::bind(&BoolParameterWidget::called, this, _1));

	connect(m_checkBox, SIGNAL(toggled(bool)), this, SLOT(handleCheckbox()));
	connect(this, SIGNAL(called(bool)), this, SLOT(handleCallback(bool)), Qt::QueuedConnection);
}

BoolParameterWidget::~BoolParameterWidget()
{}

void BoolParameterWidget::handleCallback(bool value)
{
	m_checkBox->blockSignals(true);
	m_checkBox->setChecked(value);
	m_checkBox->blockSignals(false);
}

void BoolParameterWidget::handleCheckbox()
{
	bool value = m_checkBox->isChecked();
	m_parameter.set(value);
}

void BoolParameterWidget::DecValue()
{
	m_checkBox->setChecked(false);
}

void BoolParameterWidget::IncValue()
{
	m_checkBox->setChecked(true);
}

}
