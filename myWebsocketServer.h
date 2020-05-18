#ifndef REGISTER_MYWEBSOCKETSERVER_H
#define REGISTER_MYWEBSOCKETSERVER_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>
#include <QtNetwork/qabstractsocket.h>
#include <QDateTime>
#include "flatbuffers/flatbuffers.h"
#include "SaveSchema_generated.h"


QT_FORWARD_DECLARE_CLASS(QWebSocketServer)
QT_FORWARD_DECLARE_CLASS(QWebSocket)

class myWebsocketServer : public QObject
{
Q_OBJECT
public:
    explicit myWebsocketServer(quint16 port, bool debug = false, QObject *parent = nullptr);
    ~myWebsocketServer();

    void sendBinaryMessage(QByteArray &message);

Q_SIGNALS:
    void closed();

private Q_SLOTS:
    void onNewConnection();
    void processTextMessage(QString message);
    void processBinaryMessage(QByteArray message);
    void socketDisconnected();
    void onErrorRecived(QAbstractSocket::SocketError error);

private:
    QWebSocketServer *m_pWebSocketServer;
    QList<QWebSocket *> m_clients;
    bool m_debug;
    void test();
};

#endif //REGISTER_MYWEBSOCKETSERVER_H
