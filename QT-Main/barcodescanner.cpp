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
    QUrl url(QString("%1/items/%2").arg(SERVER_BASE_URL).arg(barcodeId));
    QNetworkRequest request(url);

    manager->get(request);
    qDebug() << "Fetching item details for ID:" << barcodeId;
}


void BarcodeScanner::onNetworkReply(QNetworkReply *reply)
{
    QByteArray raw = reply->readAll();
    qDebug() << "[SCAN] server replied, raw:" << raw;

    // 1. HTTP 상태 코드 확인 (4xx, 5xx)
    int statusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

    if (reply->error() != QNetworkReply::NoError) {
        // 2. 네트워크 자체 오류 (e.g., 서버 연결 실패)
        emit fetchFailed(QString("네트워크 오류: %1").arg(reply->errorString()));
        reply->deleteLater();
        return;
    }
    else if (statusCode >= 400) {
        // 3. HTTP 상태 오류 (e.g., 404 Not Found, 400 Bad Request)
        if (statusCode == 404) {
            emit fetchFailed(QString("상품 ID %1는 존재하지 않습니다.").arg(reply->url().path().split("/").last()));
        } else {
            emit fetchFailed(QString("서버 오류 발생. Status: %1").arg(statusCode));
        }
        reply->deleteLater();
        return;
    }

    // 4. 응답 데이터 읽기 및 JSON 파싱
    QByteArray responseData = reply->readAll();
    QJsonDocument doc = QJsonDocument::fromJson(responseData);

    if (doc.isNull() || !doc.isObject()) {
        emit fetchFailed("서버로부터 유효하지 않은 응답 데이터를 받았습니다.");
        reply->deleteLater();
        return;
    }

    QJsonObject itemJson = doc.object();

    // 5. JSON 데이터를 Item 구조체에 매핑
    Item item;
    item.id = itemJson["id"].toInt();
    item.name = itemJson["name"].toString();
    item.price = itemJson["price"].toDouble();
    item.stock = itemJson["stock"].toInt();
    item.weight = itemJson["weight"].toDouble();
    double cartWeight = itemJson["cart_weight"].toDouble();


    // 6. 성공 시 시그널 방출
    emit itemFetched(item,cartWeight);

    reply->deleteLater();
}

