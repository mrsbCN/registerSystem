//
// Created by mrsb on 2020/5/15.
//

#include "myWebsocketServer.h"
#include "QtWebSockets/qwebsocketserver.h"
#include "QtWebSockets/qwebsocket.h"
#include <QtCore/QDebug>

QT_USE_NAMESPACE

//! [constructor]
myWebsocketServer::myWebsocketServer(quint16 port, bool debug, QObject *parent) :
        QObject(parent),
        m_pWebSocketServer(new QWebSocketServer(QStringLiteral("Echo Server"),
                                                QWebSocketServer::NonSecureMode, this)),
        m_debug(debug)
{
    if (m_pWebSocketServer->listen(QHostAddress::AnyIPv4, port)) {
        if (m_debug)
            qDebug() << "myWebsocketServer listening on port" << port;
        connect(m_pWebSocketServer, &QWebSocketServer::newConnection,
                this, &myWebsocketServer::onNewConnection);
        //connect(m_pWebSocketServer, &QWebSocketServer::closed, this, &myWebsocketServer::closed);
    }
}
//! [constructor]

myWebsocketServer::~myWebsocketServer()
{
    m_pWebSocketServer->close();
    qDeleteAll(m_clients.begin(), m_clients.end());
}

//! [onNewConnection]
void myWebsocketServer::onNewConnection()
{
    QWebSocket *pSocket = m_pWebSocketServer->nextPendingConnection();

    connect(pSocket, &QWebSocket::textMessageReceived, this, &myWebsocketServer::processTextMessage);
    connect(pSocket, &QWebSocket::binaryMessageReceived, this, &myWebsocketServer::processBinaryMessage);
    connect(pSocket, &QWebSocket::disconnected, this, &myWebsocketServer::socketDisconnected);
    connect(pSocket, SIGNAL(error(QAbstractSocket::SocketError)),this,SLOT(onErrorRecived(QAbstractSocket::SocketError)));
    m_clients << pSocket;
    qDebug() <<pSocket->peerAddress();
}
//! [onNewConnection]

//! [processTextMessage]
void myWebsocketServer::processTextMessage(QString message)
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug)
        qDebug() << "Message received:" << message;
    if (pClient) {
        pClient->sendTextMessage(message);
        //test();
    }
}
//! [processTextMessage]

//! [processBinaryMessage]
void myWebsocketServer::processBinaryMessage(QByteArray message)
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug)
        qDebug() << "Binary Message received:" << message;
    if (pClient) {
        test();
        //pClient->sendBinaryMessage(message);
    }
}
//! [processBinaryMessage]

//! [socketDisconnected]
void myWebsocketServer::socketDisconnected()
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    if (m_debug)
        qDebug() << "socketDisconnected:" << pClient;
    if (pClient) {
        m_clients.removeAll(pClient);
        pClient->deleteLater();
    }
}
//! [socketDisconnected]

void myWebsocketServer::onErrorRecived(QAbstractSocket::SocketError error)
{
    qDebug() << error;
}

void myWebsocketServer::sendBinaryMessage(QByteArray &message)
{
    if(m_clients.size() > 0) {
        m_clients[0]->sendBinaryMessage(message);
    } else
    {
        qDebug()<<m_clients.size();
    }
}

void myWebsocketServer::test()
{
    flatbuffers::FlatBufferBuilder FlatBuilder(1024);
    ExchangeSerialization::PoseDateBuilder PoseBuilder(FlatBuilder);
    auto pose = ExchangeSerialization::Vec7(1.1, 2, 3, 4, 5, 6, 7);
    PoseBuilder.add_Pose(&pose);
    PoseBuilder.add_Timestamp(QDateTime::currentMSecsSinceEpoch());
    auto orc = PoseBuilder.Finish();
    FlatBuilder.Finish(orc);
    qDebug() << FlatBuilder.GetSize();
    QByteArray buf((char *) FlatBuilder.GetBufferPointer(), FlatBuilder.GetSize());
    this->sendBinaryMessage(buf);
}