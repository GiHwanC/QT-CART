#include "barcodescanner.h"
#include <QNetworkRequest>
#include <QUrl>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>

BarcodeScanner::BarcodeScanner(QObject *parent)
    : QObject(parent),
      manager(new QNetworkAccessManager(this))
{
    connect(manager, &QNetworkAccessManager::finished,
            this, &BarcodeScanner::onNetworkReply);
}

// -------------------------------------------------
// 상품 추가 (스캔)
// -------------------------------------------------
void BarcodeScanner::fetchItemDetails(const QString& itemId)
{
    // 서버 주소 확인
    QUrl url(QString("http://192.168.123.43:8000/cart/add/%1").arg(itemId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    
    // POST 요청
    manager->post(request, QByteArray());

    qDebug() << "[Scanner] Requesting ADD Item ID:" << itemId;
}

// -------------------------------------------------
// 상품 제거
// -------------------------------------------------
void BarcodeScanner::removeItem(int itemId)
{
    QUrl url(QString("http://192.168.123.43:8000/cart/remove/%1").arg(itemId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    manager->post(request, QByteArray());

    qDebug() << "[Scanner] Requesting REMOVE Item ID:" << itemId;
}

// -------------------------------------------------
// 서버 응답 처리 (핵심 수정 부분)
// -------------------------------------------------
void BarcodeScanner::onNetworkReply(QNetworkReply *reply)
{
    // 1. 에러 체크
    if (reply->error() != QNetworkReply::NoError) {
        qDebug() << "[Scanner] Error:" << reply->errorString();
        emit fetchFailed(reply->errorString());
        reply->deleteLater();
        return;
    }

    // 2. 데이터 수신 및 JSON 파싱
    QByteArray data = reply->readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    
    if (!doc.isObject()) {
        qDebug() << "[Scanner] Invalid JSON response";
        reply->deleteLater();
        return;
    }

    QJsonObject obj = doc.object();
    QString action = obj["action"].toString(); 

    // 3. 데이터 처리 ("add" 액션)
    if (action == "add") {
        Item item;

        // [핵심] 서버가 보내준 데이터를 그대로 사용합니다.
        
        // (1) 상품명 파싱 ("item" 또는 "name" 키 확인)
        if (obj.contains("item")) {
            item.name = obj["item"].toString();
        } else if (obj.contains("name")) {
            item.name = obj["name"].toString();
        } else {
            item.name = "알수없음";
        }

        // (2) ID 파싱 (서버가 보내준 id 사용, 없으면 0)
        if (obj.contains("id")) {
            item.id = obj["id"].toInt();
        } else {
            item.id = 0; 
        }

        // (3) 가격 파싱 (서버가 보내준 price 사용!)
        if (obj.contains("price")) {
            item.price = obj["price"].toInt();
        } else {
            // 서버가 가격을 안 보냈을 경우 디버그 로그 출력
            qDebug() << "[Warning] Server response missing 'price' field!";
            item.price = 0; 
        }

        double expectedWeight = obj["expected_weight"].toDouble();

        qDebug() << "[Scanner] Fetched from DB -> Name:" << item.name 
                 << ", Price:" << item.price 
                 << ", ID:" << item.id;

        // UI 갱신 신호 전송
        emit itemFetched(item, expectedWeight);
    }
    else if (action == "remove") {
        qDebug() << "[Scanner] Remove confirmed.";
    }

    reply->deleteLater();
}