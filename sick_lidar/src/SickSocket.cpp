/*********************************************************************
//  created:    2014/02/02
//  filename:  SickSocket.cpp
//
//  author:     Cyril Fougeray
//              Copyright Heudiasyc UMR UTC/CNRS 6599
// 
//  version:    $Id: $
//
//  purpose:    Management of the socket connection with Sick sensors
*********************************************************************/

#include "SickSocket.h"
#include <ros/ros.h>
#include <QCoreApplication>
#include <QDebug>
#include "DebugUtil.h"

namespace sick_lidar {


SickSocket::SickSocket(SickLDMRSSensor * parent)
    : myParent(parent)
{
    socket = new QTcpSocket(this);
    connect( socket, SIGNAL(connected()), this,SLOT(socketConnected()) );
    connect( socket, SIGNAL(disconnected()),this, SLOT(socketConnectionClosed()) );
    connect( socket, SIGNAL(readyRead()),this, SLOT(socketReadyRead()) );
    connect( socket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(socketError(QAbstractSocket::SocketError)) );
}


SickSocket::~SickSocket()
{
    delete socket;
}


void SickSocket::connectToServer(QString host, int port)
{
  qDebug("trying to connect to server");
  socket->connectToHost(host,port);

  if(!socket->waitForConnected(3000))
  {
      qDebug() << "Error: " << socket->errorString();
  }

}


int SickSocket::socketConnected()
{ 
  qDebug() << "we are connected to the server " << socket->peerName().toLatin1() 
    << " at the port " << socket->peerPort() << " via the socket " << socket->socketDescriptor();
  emit configuration(); 
  
 return 1;
}

//////////////////////////////////////////////////////////////////////////
/// protected slot
void SickSocket::socketConnectionClosed()
{
  qDebug() << "the connection was closed";
}


void SickSocket::socketReadyRead()
{   
  SickFrame * frame = new SickFrame();
  frame->time = ros::Time::now();
  frame->size = socket->bytesAvailable(); 
  frame->msg = new char[frame->size]; 

  if (!frame->msg) {
      qDebug() <<"cannot allocate memory";
  }

  frame->size = socket->read(frame->msg, (qint64) frame->size);


  SickFrameEvent *e = new SickFrameEvent;
  e->frame = frame; 
  QCoreApplication::postEvent((QObject*)myParent, e);
}


void SickSocket::socketError(QAbstractSocket::SocketError e)
{
  qWarning("socket error number %d",e); 
}


void SickSocket::sendToServer(QByteArray data)
{
  socket->write(data);
}


}
