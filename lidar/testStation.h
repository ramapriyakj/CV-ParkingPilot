#pragma once
#include "cil/cImg.h"
#include "cbl/c2DPoint.h"
#include"veFramework\vebasetypes.h"
#include "QtCore/qvector.h"

class testStation : public cStation
{
  STATIONDECL(testStation);
  testStation();
  virtual ~testStation();
  void callback(cBufferT<veEgomotion> *i_buffer);
  void startPointChange();
  void m_startPointChange();
  void magnituteChange();
  void m_magnituteChange();

  QVector<c2DPoint<double >> points;
  QVector<c2DPoint<double >> m_points;
  c2DPoint<double> lastPosition;
  cBufferStamp lastTimeStamp;
  cBufferStamp dT;

  c2DPoint<double> startPoint_factor;
  c2DPoint<double> startPoint_factor2;
  float magnitute_factor;
  float magnitute_factor2;

  PORT(veEgomotion) m_ipData;
  cOPort<QVector<c2DPoint<double>>> m_opData;
  cOPort<QVector<c2DPoint<double>>> m_opData2;
  cOPort<cBufferStamp> m_dtOut;
  cOPort<vePose> m_outPose;

  // Test ego path without dropping

  vePose PositionfromEgo{ -1.0, -1.0 , 0.0, 0.0, 0.0, 0.0 };  //Calculated global position from egomotion
  QVector<c2DPoint<double>> Path;
  cOPort<QVector<c2DPoint<double>>> m_opPath;


  vePose calculate_egoPose(veEgomotion egomotion);
};

