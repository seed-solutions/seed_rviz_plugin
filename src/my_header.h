#ifndef MY_HEADER_H
#define MY_HEADER_H

#include <QtWidgets>

#include <QtWidgets>

class MyHeader : public QHeaderView
{
public:
  MyHeader(Qt::Orientation orientation, QWidget * parent = nullptr) : QHeaderView(orientation, parent)
  {}

protected:
  void paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const
  {
    painter->save();
    QHeaderView::paintSection(painter, rect, logicalIndex);
    painter->restore();
    if (logicalIndex == 0)
    {
      QStyleOptionButton option;
      option.rect = QRect(10,10,10,10);
      if (isOn)
        option.state = QStyle::State_On;
      else
        option.state = QStyle::State_Off;
      this->style()->drawPrimitive(QStyle::PE_IndicatorCheckBox, &option, painter);
    }

  }
  void mousePressEvent(QMouseEvent *event)
  {
    if (isOn)
      isOn = false;
    else
      isOn = true;
    this->update();
    QHeaderView::mousePressEvent(event);
  }
private:
  bool isOn;
};
#endif // MY_HEADER_H
