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
    // 1. HTTP 상태 코드
    int statusCode =
        reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

    // 2. 네트워크 오류
    if (reply->error() != QNetworkReply::NoError) {
        emit fetchFailed(QString("네트워크 오류: %1").arg(reply->errorString()));
        reply->deleteLater();
        return;
    }

    if (statusCode >= 400) {
        if (statusCode == 404) {
            emit fetchFailed(QString("상품 ID %1는 존재하지 않습니다.")
                             .arg(reply->url().path().split("/").last()));
        } else {
            emit fetchFailed(QString("서버 오류 발생. Status: %1").arg(statusCode));
        }
        reply->deleteLater();
        return;
    }

    // 3. 응답 데이터 (한 번만!)
    QByteArray responseData = reply->readAll();
    qDebug() << "[SCAN] server replied, raw:" << responseData;

    // 4. JSON 파싱
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(responseData, &parseError);

    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        emit fetchFailed("서버로부터 유효하지 않은 JSON 응답을 받았습니다.");
        reply->deleteLater();
        return;
    }

    QJsonObject obj = doc.object();

    // 5. ✅ 키 존재 검증 (핵심)
    if (!obj.contains("id") ||
        !obj.contains("name") ||
        !obj.contains("price") ||
        !obj.contains("stock") ||
        !obj.contains("weight") ||
        !obj.contains("cart_weight")) {

        emit fetchFailed("서버 응답에 필수 키가 누락되었습니다.");
        reply->deleteLater();
        return;
    }

    // 6. ✅ 타입 검증 (실무 필수)
    if (!obj.value("id").isDouble() ||
        !obj.value("name").isString() ||
        !obj.value("price").isDouble() ||
        !obj.value("stock").isDouble() ||
        !obj.value("weight").isDouble() ||
        !obj.value("cart_weight").isDouble()) {

        emit fetchFailed("서버 응답 데이터 타입이 올바르지 않습니다.");
        reply->deleteLater();
        return;
    }

    // 7. Item 매핑
    Item item;
    item.id     = obj.value("id").toInt();
    item.name   = obj.value("name").toString();
    item.price  = obj.value("price").toDouble();
    item.stock  = obj.value("stock").toInt();
    item.weight = obj.value("weight").toDouble();

    double cartWeight = obj.value("cart_weight").toDouble();

    // 8. 성공 시그널
    emit itemFetched(item, cartWeight);

    reply->deleteLater();
}
