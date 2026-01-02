#include "barcodescanner.h"
#include <QNetworkRequest>
#include <QUrl>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>

BarcodeScanner::BarcodeScanner(QObject *parent)
    : QObject{parent}, manager(new QNetworkAccessManager(this))
{
    connect(manager, &QNetworkAccessManager::finished,
            this, &BarcodeScanner::onNetworkReply);
}

void BarcodeScanner::fetchItemDetails(const QString& barcodeId)
{
    QUrl url(QString("%1/scan/%2").arg(SERVER_BASE_URL).arg(barcodeId));
    QNetworkRequest request(url);

    manager->get(request);
    qDebug() << "Fetching item details for ID:" << barcodeId;
}

void BarcodeScanner::removeItem(int itemId)
{
    QUrl url(QString("%1/cart/remove/%2")
             .arg(SERVER_BASE_URL)
             .arg(itemId));

    QNetworkRequest request(url);
    manager->post(request, QByteArray());
}

void BarcodeScanner::checkCartStatus()
{
    QUrl url(QString("%1/cart/check").arg(SERVER_BASE_URL));
    QNetworkRequest request(url);
    manager->get(request);
}

void BarcodeScanner::onNetworkReply(QNetworkReply *reply)
{
    // 1. 네트워크 오류
    if (reply->error() != QNetworkReply::NoError) {

        // cart/update-weight 응답이면 조용히 처리
        if (reply->url().path().contains("/cart/update-weight")) {
            qDebug() << "[WEIGHT UPDATE ERROR]" << reply->errorString();
            reply->deleteLater();
            return;
        }

        // item 조회 에러만 사용자에게 알림
        emit fetchFailed(reply->errorString());
        reply->deleteLater();
        return;
    }

    // 2. 응답 데이터
    QByteArray responseData = reply->readAll();
    qDebug() << "[NETWORK RAW]" << responseData;

    // 3. JSON 파싱
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(responseData, &parseError);

    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        emit fetchFailed("Invalid JSON response");
        reply->deleteLater();
        return;
    }

    QJsonObject obj = doc.object();

    if (obj.contains("movable")) {
        bool movable = obj["movable"].toBool();
        if (!movable) emit requestStop();
    }

    if (obj.contains("item")) {
        QJsonObject it = obj["item"].toObject();
        Item item;
        item.id = it["id"].toInt();
        item.name = it["name"].toString();
        item.price = it["price"].toDouble();
        item.weight = it["weight"].toDouble();
        emit itemFetched(item, 0.0);
    }

    Item item;
    item.id     = obj.value("id").toInt();
    item.name   = obj.value("name").toString();
    item.price  = obj.value("price").toDouble();

    emit itemFetched(item, 0.0); // cart_weight 이제 서버가 관리하므로 의미 없음

    reply->deleteLater();
}
