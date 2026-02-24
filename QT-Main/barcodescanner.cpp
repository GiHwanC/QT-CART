#include "barcodescanner.h"
#include <QNetworkRequest>
#include <QUrl>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVariant> 

BarcodeScanner::BarcodeScanner(QObject *parent)
    : QObject(parent),
      manager(new QNetworkAccessManager(this))
{
    connect(manager, &QNetworkAccessManager::finished,
            this, &BarcodeScanner::onNetworkReply);
}

void BarcodeScanner::setApiBaseUrl(const QString &ip, int port)
{
    m_apiBaseUrl = QString("http://%1:%2").arg(ip).arg(port);
    qDebug() << "[Scanner] API Base URL set to:" << m_apiBaseUrl;
}

// 상품 추가
void BarcodeScanner::fetchItemDetails(const QString& itemId)
{
    QString path = QString("%1/cart/add/%2").arg(m_apiBaseUrl).arg(itemId);
    QUrl url(path);
    
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    
    manager->post(request, QByteArray());

    qDebug() << "==================================================";
    qDebug() << "[Scanner] Requesting ADD -> ID:" << itemId;
    qDebug() << "[Scanner] URL:" << url.toString();
}

// 상품 제거
void BarcodeScanner::removeItem(const QString& itemId)
{
    QString path = QString("%1/cart/remove/%2").arg(m_apiBaseUrl).arg(itemId);
    QUrl url(path);
    
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    manager->post(request, QByteArray());

    qDebug() << "==================================================";
    qDebug() << "[Scanner] Requesting REMOVE -> ID:" << itemId;
    qDebug() << "[Scanner] URL:" << url.toString();
}

void BarcodeScanner::onNetworkReply(QNetworkReply *reply)
{
    if (reply->error() != QNetworkReply::NoError) {
        qDebug() << "[Scanner] Network Error:" << reply->errorString();
        emit fetchFailed(reply->errorString());
        reply->deleteLater();
        return;
    }

    QByteArray data = reply->readAll();
    qDebug() << "[Scanner] Raw Response Data:" << data;

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject()) {
        qDebug() << "[Scanner] Invalid JSON response";
        reply->deleteLater();
        return;
    }

    QJsonObject obj = doc.object();
    QString action = obj["action"].toString(); 

    qDebug() << "[Scanner] Parsed Action:" << action;

    // 데이터 유효성 검사
    if (action == "add") {
        Item item;

        if (obj.contains("item")) item.name = obj["item"].toString();
        else if (obj.contains("name")) item.name = obj["name"].toString();
        else item.name = "알수없음";

        if (obj.contains("id")) {
            item.id = obj["id"].toVariant().toString(); 
        }

        if (obj.contains("price")) item.price = obj["price"].toDouble();

        double expectedWeight = 0.0;
        if (obj.contains("expected_weight")) {
             expectedWeight = obj["expected_weight"].toDouble();
             item.weight = expectedWeight;
        }

        qDebug() << "[Scanner] Item Parsed: " << item.name << ", " << item.price;
        
        emit itemFetched(item, expectedWeight);
    }
    else if (action == "remove") {
        qDebug() << "[Scanner] Remove confirmed.";
    }

    reply->deleteLater();
}