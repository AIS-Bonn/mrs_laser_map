//subclass for slider
//Author: Sebastian Schüller

#ifndef PARAMETERITEM_H
#define PARAMETERITEM_H

#include <QTreeWidget>
#include <QSpinBox>
#include <QLineEdit>
#include <QCheckBox>

#include <config_server/parameter.h>

namespace parametertuner
{

class WheelFilter : public QObject
{
Q_OBJECT
public:
	explicit WheelFilter(QObject* parent = 0);

protected:
    virtual bool eventFilter(QObject* object, QEvent* event);
};



class ParameterWidgetBase : public QWidget
{
Q_OBJECT
public:
	virtual void IncValue() = 0;
	virtual void DecValue() = 0;
};

class IntParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit IntParameterWidget(ros::NodeHandle& nh, const config_server::ParameterDescription& description);
	virtual ~IntParameterWidget();
    virtual void DecValue();
    virtual void IncValue();

Q_SIGNALS:
	void called(int);

private Q_SLOTS:
	void handleSlider();
	void handleSpinbox();
	void handleCallback(int value);

private:
	config_server::Parameter<int> m_parameter;

	QSlider* m_slider;
	QSpinBox* m_spinbox;
};


class FloatParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit FloatParameterWidget(ros::NodeHandle& nh, const config_server::ParameterDescription& description);
	virtual ~FloatParameterWidget();
    virtual void DecValue();
    virtual void IncValue();

Q_SIGNALS:
	void called(float);

private Q_SLOTS:
	void handleSlider();
	void handleSpinbox();
	void handleCallback(float value);

private:
	config_server::Parameter< float > m_parameter;

	double m_sliderStepRatio;
	QSlider* m_slider;
	QDoubleSpinBox* m_spinbox;
};


class StringParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit StringParameterWidget(ros::NodeHandle& nh, const config_server::ParameterDescription& description);
	virtual ~StringParameterWidget();
	virtual void DecValue();
	virtual void IncValue();

Q_SIGNALS:
	void called(std::string);

private Q_SLOTS:
	void handleLineEdit();
	void handleCallback(std::string text);

private:
	config_server::Parameter< std::string > m_parameter;

	QLineEdit* m_lineEdit;
};

class BoolParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit BoolParameterWidget(ros::NodeHandle& nh, const config_server::ParameterDescription& description);
	virtual ~BoolParameterWidget();
	virtual void DecValue();
	virtual void IncValue();

Q_SIGNALS:
	void called(bool);

private Q_SLOTS:
	void handleCheckbox();
	void handleCallback(bool value);

private:
	config_server::Parameter< bool > m_parameter;

	QCheckBox* m_checkBox;
};

}

#endif